#pragma once

#include <Eigen/Dense>


Eigen::Vector3d normalizeImagePoint(
    const Eigen::Vector2d& image_point,
    const Eigen::Matrix3d& K
);


Eigen::Vector2d projectPoint(
    const Eigen::Vector3d& point_camera,
    const Eigen::Matrix3d& K
);
