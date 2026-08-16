#include "evaluation.h"

#include <cmath>
#include <stdexcept>

Eigen::Matrix4d planarPoseToSE3(
    const Eigen::Vector3d& pose
) {
    const double x = pose(0);
    const double y = pose(1);
    const double theta = pose(2);

    Eigen::Matrix4d T =
        Eigen::Matrix4d::Identity();

    T(0,0) = std::cos(theta);
    T(0,1) = -std::sin(theta);

    T(1,0) = std::sin(theta);
    T(1,1) = std::cos(theta);

    T(0,3) = x;
    T(1,3) = y;

    return T;
}
PoseEvaluationResult evaluatePoses(
    const std::vector<Eigen::Matrix4d>& estimated_poses,
    const std::vector<TrajectoryEntry>& trajectory,
    const Eigen::Matrix4d& cam_transform
) {

    if (trajectory.size() < estimated_poses.size()) {
        throw std::runtime_error(
            "Not enough ground-truth poses for evaluation"
        );
    }


    double total_rotation_error = 0.0;

    std::vector<double> scale_ratios;


    for (int i = 0;
         i < static_cast<int>(estimated_poses.size()) - 1;
         ++i) {

        Eigen::Matrix4d T_w_c0 =
            estimated_poses[i].inverse();

        Eigen::Matrix4d T_w_c1 =
            estimated_poses[i + 1].inverse();


        Eigen::Matrix4d rel_T =
            T_w_c0.inverse() * T_w_c1;


        Eigen::Matrix4d GT_w_r0 =
            planarPoseToSE3(
                trajectory[i].gt_pose
            );

        Eigen::Matrix4d GT_w_r1 =
            planarPoseToSE3(
                trajectory[i + 1].gt_pose
            );


        Eigen::Matrix4d GT_w_c0 =
            GT_w_r0 * cam_transform;

        Eigen::Matrix4d GT_w_c1 =
            GT_w_r1 * cam_transform;


        Eigen::Matrix4d rel_GT =
            GT_w_c0.inverse() * GT_w_c1;


        Eigen::Matrix4d error_T =
            rel_T.inverse() * rel_GT;


        Eigen::Matrix3d error_R =
            error_T.block<3,3>(0,0);


        double rotation_error =
            (
                Eigen::Matrix3d::Identity()
                - error_R
            ).trace();


        total_rotation_error +=
            rotation_error;


        Eigen::Vector3d t_est =
            rel_T.block<3,1>(0,3);

        Eigen::Vector3d t_gt =
            rel_GT.block<3,1>(0,3);


        double gt_norm =
            t_gt.norm();


        if (gt_norm > 1e-9) {

            double ratio =
                t_est.norm() / gt_norm;

            scale_ratios.push_back(ratio);
        }
    }


    PoseEvaluationResult result;

    result.evaluated_pairs =
        estimated_poses.size() - 1;


    result.mean_rotation_error =
        total_rotation_error /
        result.evaluated_pairs;


    double ratio_sum = 0.0;

    for (double ratio : scale_ratios) {
        ratio_sum += ratio;
    }


    result.mean_scale_ratio =
        ratio_sum / scale_ratios.size();


    double variance = 0.0;

    for (double ratio : scale_ratios) {

        double difference =
            ratio - result.mean_scale_ratio;

        variance +=
            difference * difference;
    }


    variance /= scale_ratios.size();

    result.std_scale_ratio =
        std::sqrt(variance);


    return result;
}


MapEvaluationResult evaluateMap(
    const std::vector<MapPoint>& estimated_map,
    const std::vector<WorldPoint>& gt_world,
    double scale_ratio,
    const Eigen::Vector3d& initial_gt_pose,
    const Eigen::Matrix4d& cam_transform
) {
    if (scale_ratio <= 0.0) {
        throw std::runtime_error("Scale ratio must be positive");
    }

    const Eigen::Matrix4d GT_w_r0 =
        planarPoseToSE3(initial_gt_pose);
    const Eigen::Matrix4d GT_w_c0 =
        GT_w_r0 * cam_transform;

    double sum_squared_error = 0.0;
    int matched_points = 0;

    for (const MapPoint& estimated_point : estimated_map) {
        const WorldPoint* gt_point = nullptr;

        for (const WorldPoint& candidate : gt_world) {
            if (candidate.id == estimated_point.actual_id) {
                gt_point = &candidate;
                break;
            }
        }

        if (gt_point == nullptr) {
            continue;
        }

        const Eigen::Vector3d scaled_position =
            estimated_point.position / scale_ratio;

        Eigen::Vector4d point_c0_h;
        point_c0_h << scaled_position, 1.0;

        const Eigen::Vector4d point_world_h =
            GT_w_c0 * point_c0_h;
        const Eigen::Vector3d error =
            point_world_h.head<3>() - gt_point->position;

        sum_squared_error += error.squaredNorm();
        matched_points++;
    }

    if (matched_points == 0) {
        throw std::runtime_error(
            "No estimated map points could be matched with ground truth"
        );
    }

    MapEvaluationResult result;
    result.evaluated_points = matched_points;
    result.rmse = std::sqrt(sum_squared_error / matched_points);
    return result;
}
