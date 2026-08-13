#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>


struct CameraData {
    Eigen::Matrix3d K;
    Eigen::Matrix4d cam_transform;

    double z_near;
    double z_far;

    int width;
    int height;
};


struct Feature {
    int local_id;
    int actual_id;

    Eigen::Vector2d image_point;

    Eigen::Matrix<double, 10, 1> appearance;
};


struct Measurement {
    int seq;

    Eigen::Vector3d gt_pose;
    Eigen::Vector3d odom_pose;

    std::vector<Feature> features;
};


CameraData loadCamera(const std::string& filename);

Measurement loadMeasurement(const std::string& filename);