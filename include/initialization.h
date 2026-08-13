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
