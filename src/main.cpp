#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "data_association.h"
#include "dataset.h"
#include "geometry.h"
#include "initialization.h"
#include "projective_icp.h"
#include "evaluation.h"
#include "visualization.h"

int main() {
    std::ofstream output_file("../visual_odometry_output.txt");

    if (!output_file) {
        std::cerr << "ERROR: Cannot open output file\n";
        return 1;
    }

    std::streambuf* console_buffer = std::cout.rdbuf(output_file.rdbuf());

    try {
        const CameraData camera = loadCamera("../data/camera.dat");
        const Measurement frame0 = loadMeasurement("../data/meas-00000.dat");
        const Measurement frame1 = loadMeasurement("../data/meas-00001.dat");

        const std::vector<Match> matches = matchFeatures(frame0, frame1);
        const Eigen::Matrix3d E =
            estimateEssentialMatrix(frame0, frame1, matches, camera.K);
        const std::vector<MotionCandidate> candidates =
            decomposeEssentialMatrix(E);

        int best_candidate = -1;
        int best_positive_points = -1;

        for (int c = 0; c < static_cast<int>(candidates.size()); ++c) {
            int positive_points = 0;

            for (const Match& match : matches) {
                const Eigen::Vector3d p0 = normalizeImagePoint(
                    frame0.features[match.index_frame0].image_point,
                    camera.K
                );
                const Eigen::Vector3d p1 = normalizeImagePoint(
                    frame1.features[match.index_frame1].image_point,
                    camera.K
                );
                const Eigen::Vector3d X = triangulatePoint(
                    p0, p1, candidates[c].R, candidates[c].t
                );
                const Eigen::Vector3d X1 =
                    candidates[c].R * X + candidates[c].t;

                if (X.z() > 0.0 && X1.z() > 0.0) {
                    positive_points++;
                }
            }

            if (positive_points > best_positive_points) {
                best_positive_points = positive_points;
                best_candidate = c;
            }
        }

        if (best_candidate < 0) {
            throw std::runtime_error("Unable to select an initial motion");
        }

        const MotionCandidate& initial_motion = candidates[best_candidate];
        const std::vector<MapPoint> initial_map = buildInitialMap(
            frame0, frame1, matches, camera.K, initial_motion
        );

        const Eigen::Matrix4d T_c0_w = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_c1_w = Eigen::Matrix4d::Identity();
        T_c1_w.block<3,3>(0,0) = initial_motion.R;
        T_c1_w.block<3,1>(0,3) = initial_motion.t;

        double initialization_error_sum = 0.0;
        for (int i = 0; i < static_cast<int>(initial_map.size()); ++i) {
            const Match& match = matches[i];
            const Eigen::Vector3d& X = initial_map[i].position;
            const Eigen::Vector3d X1 = initial_motion.R * X + initial_motion.t;
            initialization_error_sum +=
                (projectPoint(X, camera.K)
                 - frame0.features[match.index_frame0].image_point).norm();
            initialization_error_sum +=
                (projectPoint(X1, camera.K)
                 - frame1.features[match.index_frame1].image_point).norm();
        }

        if (initial_map.empty()) {
            throw std::runtime_error("The initial map is empty");
        }

        const double initialization_error =
            initialization_error_sum / (2.0 * initial_map.size());

        std::cout
            << "===== INITIALIZATION =====\n"
            << "Matches frame 0-1: " << matches.size() << "\n"
            << "Selected motion candidate: " << best_candidate + 1 << "\n"
            << "Initial map points: " << initial_map.size() << "\n"
            << "Initialization reprojection error: "
            << initialization_error << " px\n";

        std::vector<MapPoint> map = initial_map;

        std::vector<Eigen::Matrix4d> estimated_poses;
        estimated_poses.push_back(T_c0_w);
        estimated_poses.push_back(T_c1_w);

        Eigen::Matrix4d T_prevprev = T_c0_w;
        Eigen::Matrix4d T_prev = T_c1_w;
        Measurement previous_frame = frame1;

        for (int k = 2; k <= 108; ++k) {
            std::ostringstream filename;
            filename
                << "../data/meas-"
                << std::setw(5)
                << std::setfill('0')
                << k
                << ".dat";

            const Measurement current_frame =
                loadMeasurement(filename.str());

            const std::vector<Match> consecutive_matches =
                matchFeatures(previous_frame, current_frame);

            int new_landmark_matches = 0;

            for (const Match& match : consecutive_matches) {
                const Feature& previous_feature =
                    previous_frame.features[match.index_frame0];
                const Feature& current_feature =
                    current_frame.features[match.index_frame1];

                const bool previous_in_map =
                    isFeatureInMap(previous_feature, map);
                const bool current_in_map =
                    isFeatureInMap(current_feature, map);

                if (!previous_in_map && !current_in_map) {
                    new_landmark_matches++;
                }
            }

            int known_features = 0;
            int new_features = 0;

            for (const Feature& feature : current_frame.features) {
                if (isFeatureInMap(feature, map)) {
                    known_features++;
                } else {
                    new_features++;
                }
            }

            const std::vector<MapMatch> current_matches =
                matchMapToFrame(map, current_frame);

            std::cout
                << "\n===== FRAME " << k << " =====\n"
                << "Features: " << current_frame.features.size() << "\n"
                << "Known features: " << known_features << "\n"
                << "New features: " << new_features << "\n"
                << "3D-2D matches: " << current_matches.size() << "\n"
                << "Consecutive frame matches: "
                << consecutive_matches.size() << "\n"
                << "New landmark candidates: "
                << new_landmark_matches << "\n";

            if (current_matches.size() < 6) {
                std::cout << "Not enough matches. Tracking stopped.\n";
                break;
            }

            const Eigen::Matrix4d T_relative =
                T_prev * T_prevprev.inverse();
            const Eigen::Matrix4d T_guess = T_relative * T_prev;
            const double initial_error = computeMeanReprojectionError(
                map,
                current_frame,
                current_matches,
                T_guess,
                camera.K
            );

            std::cout
                << "Initial reprojection error: "
                << initial_error
                << " px\n"
                << "===== PROJECTIVE ICP =====\n";

            const Eigen::Matrix4d T_current = optimizePoseProjectiveICP(
                map,
                current_frame,
                current_matches,
                T_guess,
                camera.K
            );

            const double final_error = computeMeanReprojectionError(
                map,
                current_frame,
                current_matches,
                T_current,
                camera.K
            );

            std::cout << "Final error: " << final_error << " px\n";

            const int added_points = addNewLandmarks(
                previous_frame,
                current_frame,
                consecutive_matches,
                T_prev,
                T_current,
                camera.K,
                map
            );

            std::cout
                << "New map points added: " << added_points << "\n"
                << "Total map points: " << map.size() << "\n";

            estimated_poses.push_back(T_current);
            T_prevprev = T_prev;
            T_prev = T_current;
            previous_frame = current_frame;
        }

        std::vector<TrajectoryEntry> trajectory =
            loadTrajectory("../data/trajectory.dat");

        PoseEvaluationResult pose_result =
            evaluatePoses(
                estimated_poses,
                trajectory,
                camera.cam_transform
            );

        std::cout
            << "\n===== POSE EVALUATION =====\n";

        std::cout
            << "Evaluated pose pairs: "
            << pose_result.evaluated_pairs
            << "\n";

        std::cout
            << "Mean rotation error: "
            << pose_result.mean_rotation_error
            << "\n";

        std::cout
            << "Mean translation scale ratio: "
            << pose_result.mean_scale_ratio
            << "\n";

        std::cout
            << "Scale ratio std: "
            << pose_result.std_scale_ratio
            << "\n";

        std::vector<WorldPoint> gt_world =
            loadWorld("../data/world.dat");

        MapEvaluationResult map_result =
            evaluateMap(
                map,
                gt_world,
                pose_result.mean_scale_ratio,
                trajectory[0].gt_pose,
                camera.cam_transform
            );

        std::cout
            << "\n===== MAP EVALUATION =====\n";

        std::cout
            << "Estimated map points: "
            << map.size()
            << "\n";

        std::cout
            << "Evaluated map points: "
            << map_result.evaluated_points
            << "\n";

        std::cout
            << "Map RMSE: "
            << map_result.rmse
            << "\n";

        saveTrajectoryCsv(
            estimated_poses,
            trajectory,
            pose_result.mean_scale_ratio,
            camera.cam_transform,
            "../trajectory.csv"
        );

        saveMapCsv(
            map,
            gt_world,
            pose_result.mean_scale_ratio,
            trajectory[0].gt_pose,
            camera.cam_transform,
            "../map.csv"
        );

        std::cout
            << "\nVisualization data saved:\n"
            << "../trajectory.csv\n"
            << "../map.csv\n";
    } catch (const std::exception& e) {
        std::cout << "ERROR: " << e.what() << '\n';
        std::cout.flush();
        std::cout.rdbuf(console_buffer);
        return 1;
    }

    std::cout.flush();
    std::cout.rdbuf(console_buffer);

    return 0;
}
