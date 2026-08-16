#pragma once

#include <Eigen/Dense>
#include <vector>

#include "dataset.h"
#include "map.h"


struct PoseEvaluationResult {
    double mean_rotation_error;

    double mean_scale_ratio;
    double std_scale_ratio;

    int evaluated_pairs;
};


struct MapEvaluationResult {
    double rmse;
    int evaluated_points;
};


Eigen::Matrix4d planarPoseToSE3(
    const Eigen::Vector3d& pose
);


PoseEvaluationResult evaluatePoses(
    const std::vector<Eigen::Matrix4d>& estimated_poses,
    const std::vector<TrajectoryEntry>& trajectory,
    const Eigen::Matrix4d& cam_transform
);


MapEvaluationResult evaluateMap(
    const std::vector<MapPoint>& estimated_map,
    const std::vector<WorldPoint>& gt_world,
    double scale_ratio,
    const Eigen::Vector3d& initial_gt_pose,
    const Eigen::Matrix4d& cam_transform
);
