#pragma once

#include <Eigen/Dense>
#include <vector>

#include "dataset.h"
#include "map.h"
#include "data_association.h"


double computeMeanReprojectionError(
    const std::vector<MapPoint>& map,
    const Measurement& frame,
    const std::vector<MapMatch>& matches,
    const Eigen::Matrix4d& T_c_w,
    const Eigen::Matrix3d& K
);


Eigen::Matrix4d optimizePoseProjectiveICP(
    const std::vector<MapPoint>& map,
    const Measurement& frame,
    const std::vector<MapMatch>& matches,
    const Eigen::Matrix4d& initial_pose,
    const Eigen::Matrix3d& K,
    int max_iterations = 10
);
