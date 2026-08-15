#pragma once

#include <Eigen/Dense>
#include <vector>

#include "dataset.h"
#include "data_association.h"
#include "map.h"

struct MotionCandidate {

    Eigen::Matrix3d R;
    Eigen::Vector3d t;
};


Eigen::Matrix3d estimateEssentialMatrix(
    const Measurement& frame0,
    const Measurement& frame1,
    const std::vector<Match>& matches,
    const Eigen::Matrix3d& K
);


std::vector<MotionCandidate> decomposeEssentialMatrix(
    const Eigen::Matrix3d& E
);

Eigen::Vector3d triangulatePoint(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t
);

std::vector<MapPoint> buildInitialMap(
    const Measurement& frame0,
    const Measurement& frame1,
    const std::vector<Match>& matches,
    const Eigen::Matrix3d& K,
    const MotionCandidate& motion
);


int addNewLandmarks(
    const Measurement& previous_frame,
    const Measurement& current_frame,
    const std::vector<Match>& consecutive_matches,
    const Eigen::Matrix4d& T_prev_w,
    const Eigen::Matrix4d& T_current_w,
    const Eigen::Matrix3d& K,
    std::vector<MapPoint>& map
);
