#pragma once

#include <Eigen/Dense>


struct MapPoint {

    Eigen::Vector3d position;

    Eigen::Matrix<double, 10, 1> appearance;


    int actual_id;
};
