#include "visualization.h"
#include "evaluation.h"

#include <fstream>
#include <stdexcept>


void saveTrajectoryCsv(
    const std::vector<Eigen::Matrix4d>& estimated_poses,
    const std::vector<TrajectoryEntry>& trajectory,
    double scale_ratio,
    const Eigen::Matrix4d& cam_transform,
    const std::string& filename
) {
    if (trajectory.size() < estimated_poses.size()) {
        throw std::runtime_error(
            "Not enough trajectory entries for visualization"
        );
    }

    if (scale_ratio <= 0.0) {
        throw std::runtime_error("Scale ratio must be positive");
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to create trajectory CSV");
    }

    file << "frame,est_x,est_y,est_z,gt_x,gt_y,gt_z\n";

    const Eigen::Matrix4d GT_w_c0 =
        planarPoseToSE3(trajectory[0].gt_pose) * cam_transform;
    const Eigen::Matrix3d R_w_c0 = GT_w_c0.block<3,3>(0,0);
    const Eigen::Vector3d t_w_c0 = GT_w_c0.block<3,1>(0,3);

    for (int i = 0; i < static_cast<int>(estimated_poses.size()); ++i) {
        const Eigen::Matrix4d T_vo_w_c = estimated_poses[i].inverse();
        const Eigen::Vector3d estimated_position_vo =
            T_vo_w_c.block<3,1>(0,3);
        const Eigen::Vector3d estimated_position_metric =
            estimated_position_vo / scale_ratio;
        const Eigen::Vector3d estimated_position_world =
            R_w_c0 * estimated_position_metric + t_w_c0;

        const Eigen::Matrix4d GT_w_c =
            planarPoseToSE3(trajectory[i].gt_pose) * cam_transform;
        const Eigen::Vector3d gt_position_world =
            GT_w_c.block<3,1>(0,3);

        file
            << i << ","
            << estimated_position_world.x() << ","
            << estimated_position_world.y() << ","
            << estimated_position_world.z() << ","
            << gt_position_world.x() << ","
            << gt_position_world.y() << ","
            << gt_position_world.z() << "\n";
    }
}


void saveMapCsv(
    const std::vector<MapPoint>& estimated_map,
    const std::vector<WorldPoint>& gt_world,
    double scale_ratio,
    const Eigen::Vector3d& initial_gt_pose,
    const Eigen::Matrix4d& cam_transform,
    const std::string& filename
) {
    if (scale_ratio <= 0.0) {
        throw std::runtime_error("Scale ratio must be positive");
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to create map CSV");
    }

    file << "id,est_x,est_y,est_z,gt_x,gt_y,gt_z\n";

    const Eigen::Matrix4d GT_w_c0 =
        planarPoseToSE3(initial_gt_pose) * cam_transform;
    const Eigen::Matrix3d R_w_c0 = GT_w_c0.block<3,3>(0,0);
    const Eigen::Vector3d t_w_c0 = GT_w_c0.block<3,1>(0,3);

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
        const Eigen::Vector3d estimated_world_position =
            R_w_c0 * scaled_position + t_w_c0;

        file
            << estimated_point.actual_id << ","
            << estimated_world_position.x() << ","
            << estimated_world_position.y() << ","
            << estimated_world_position.z() << ","
            << gt_point->position.x() << ","
            << gt_point->position.y() << ","
            << gt_point->position.z() << "\n";
    }
}
