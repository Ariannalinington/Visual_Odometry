#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "dataset.h"
#include "map.h"


void saveTrajectoryCsv(
    const std::vector<Eigen::Matrix4d>& estimated_poses,
    const std::vector<TrajectoryEntry>& trajectory,
    double scale_ratio,
    const Eigen::Matrix4d& cam_transform,
    const std::string& filename
);


void saveMapCsv(
    const std::vector<MapPoint>& estimated_map,
    const std::vector<WorldPoint>& gt_world,
    double scale_ratio,
    const Eigen::Vector3d& initial_gt_pose,
    const Eigen::Matrix4d& cam_transform,
    const std::string& filename
);
