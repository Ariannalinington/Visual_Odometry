#include "projective_icp.h"
#include "geometry.h"

#include <Eigen/Cholesky>
#include <cmath>
#include <iostream>
#include <stdexcept>


namespace {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d result;
    result <<
         0.0, -v.z(),  v.y(),
         v.z(),  0.0, -v.x(),
        -v.y(),  v.x(),  0.0;
    return result;
}


Eigen::Matrix4d se3Exp(const Eigen::Matrix<double, 6, 1>& increment) {
    const Eigen::Vector3d translation = increment.head<3>();
    const Eigen::Vector3d rotation = increment.tail<3>();
    const double angle = rotation.norm();
    const Eigen::Matrix3d W = skew(rotation);

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d V = Eigen::Matrix3d::Identity();

    if (angle < 1e-10) {
        R += W + 0.5 * W * W;
        V += 0.5 * W + (1.0 / 6.0) * W * W;
    } else {
        const double angle2 = angle * angle;
        R += (std::sin(angle) / angle) * W
             + ((1.0 - std::cos(angle)) / angle2) * W * W;
        V += ((1.0 - std::cos(angle)) / angle2) * W
             + ((angle - std::sin(angle)) / (angle2 * angle)) * W * W;
    }

    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3,3>(0,0) = R;
    transform.block<3,1>(0,3) = V * translation;
    return transform;
}

}

double computeMeanReprojectionError(
    const std::vector<MapPoint>& map,
    const Measurement& frame,
    const std::vector<MapMatch>& matches,
    const Eigen::Matrix4d& T_c_w,
    const Eigen::Matrix3d& K
) {

    double total_error = 0.0;
    int valid_points = 0;


    Eigen::Matrix3d R =
        T_c_w.block<3,3>(0,0);

    Eigen::Vector3d t =
        T_c_w.block<3,1>(0,3);


    for (const MapMatch& match : matches) {

        const MapPoint& map_point =
            map[match.map_index];

        const Feature& feature =
            frame.features[match.feature_index];


        Eigen::Vector3d X_camera =
            R * map_point.position + t;


        if (X_camera(2) <= 0) {
            continue;
        }


        Eigen::Vector2d predicted =
            projectPoint(
                X_camera,
                K
            );


        Eigen::Vector2d error =
            feature.image_point - predicted;


        total_error += error.norm();
        valid_points++;
    }


    if (valid_points == 0) {
        return 0.0;
    }


    return total_error / valid_points;
}


Eigen::Matrix4d optimizePoseProjectiveICP(
    const std::vector<MapPoint>& map,
    const Measurement& frame,
    const std::vector<MapMatch>& matches,
    const Eigen::Matrix4d& initial_pose,
    const Eigen::Matrix3d& K,
    int max_iterations
) {
    if (matches.size() < 3) {
        throw std::runtime_error(
            "At least 3 map-to-frame matches are required for pose optimization"
        );
    }

    Eigen::Matrix4d pose = initial_pose;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        Eigen::Matrix<double, 6, 6> H =
            Eigen::Matrix<double, 6, 6>::Zero();
        Eigen::Matrix<double, 6, 1> b =
            Eigen::Matrix<double, 6, 1>::Zero();

        const Eigen::Matrix3d R = pose.block<3,3>(0,0);
        const Eigen::Vector3d t = pose.block<3,1>(0,3);
        int valid_points = 0;

        for (const MapMatch& match : matches) {
            const Eigen::Vector3d X =
                R * map[match.map_index].position + t;

            if (X.z() <= 1e-9) {
                continue;
            }

            const Eigen::Vector3d homogeneous = K * X;
            const double w = homogeneous.z();
            if (std::abs(w) <= 1e-12) {
                continue;
            }

            Eigen::Matrix<double, 2, 3> projection_jacobian;
            projection_jacobian.row(0) =
                (K.row(0) * w - homogeneous.x() * K.row(2)) / (w * w);
            projection_jacobian.row(1) =
                (K.row(1) * w - homogeneous.y() * K.row(2)) / (w * w);

            Eigen::Matrix<double, 3, 6> motion_jacobian;
            motion_jacobian.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
            motion_jacobian.block<3,3>(0,3) = -skew(X);

            const Eigen::Matrix<double, 2, 6> J =
                projection_jacobian * motion_jacobian;
            const Eigen::Vector2d error =
                frame.features[match.feature_index].image_point
                - projectPoint(X, K);

            H.noalias() += J.transpose() * J;
            b.noalias() += J.transpose() * error;
            valid_points++;
        }

        if (valid_points < 3) {
            throw std::runtime_error(
                "Not enough points in front of the camera during ICP"
            );
        }

        std::cout
            << "Iteration " << iteration
            << ": error = "
            << computeMeanReprojectionError(map, frame, matches, pose, K)
            << " pixels\n";

        H.diagonal().array() += 1e-9;
        const Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(H);
        if (solver.info() != Eigen::Success) {
            throw std::runtime_error("Unable to solve ICP normal equations");
        }

        const Eigen::Matrix<double, 6, 1> increment = solver.solve(b);
        if (!increment.allFinite()) {
            throw std::runtime_error("ICP produced a non-finite pose increment");
        }

        const double current_error =
            computeMeanReprojectionError(map, frame, matches, pose, K);
        bool accepted = false;
        double step = 1.0;

        for (int attempt = 0; attempt < 10; ++attempt) {
            const Eigen::Matrix4d candidate_pose =
                se3Exp(step * increment) * pose;
            const double candidate_error = computeMeanReprojectionError(
                map, frame, matches, candidate_pose, K
            );

            if (candidate_error < current_error) {
                pose = candidate_pose;
                accepted = true;
                break;
            }

            step *= 0.5;
        }

        if (!accepted || step * increment.norm() < 1e-10) {
            break;
        }
    }

    return pose;
}
