#pragma once

#include "dataset.h"
#include "map.h"

#include <vector>


struct Match {
    int index_frame0;
    int index_frame1;

    double distance;
};


struct MapMatch {

    int map_index;
    int feature_index;

    double distance;
};


std::vector<Match> matchFeatures(
    const Measurement& frame0,
    const Measurement& frame1
);


std::vector<MapMatch> matchMapToFrame(
    const std::vector<MapPoint>& map,
    const Measurement& frame
);
