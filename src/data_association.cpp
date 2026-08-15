#include "data_association.h"

#include <limits>


std::vector<Match> matchFeatures(
    const Measurement& frame0,
    const Measurement& frame1
) {

    std::vector<Match> matches;

    const double MATCH_THRESHOLD = 1e-6;


    for (int i = 0; i < frame0.features.size(); ++i) {

        const Feature& feature0 =
            frame0.features[i];


        double best_distance =
            std::numeric_limits<double>::max();

        int best_index = -1;


        for (int j = 0; j < frame1.features.size(); ++j) {

            const Feature& feature1 =
                frame1.features[j];


            double distance =
                (feature0.appearance -
                 feature1.appearance).norm();


            if (distance < best_distance) {

                best_distance = distance;
                best_index = j;
            }
        }


        if (best_index != -1 &&
            best_distance < MATCH_THRESHOLD) {

            Match match;

            match.index_frame0 = i;
            match.index_frame1 = best_index;
            match.distance = best_distance;

            matches.push_back(match);
        }
    }


    return matches;
}


std::vector<MapMatch> matchMapToFrame(
    const std::vector<MapPoint>& map,
    const Measurement& frame
) {

    std::vector<MapMatch> matches;

    const double MATCH_THRESHOLD = 1e-6;


    for (int i = 0; i < static_cast<int>(map.size()); ++i) {

        const MapPoint& map_point = map[i];


        double best_distance =
            std::numeric_limits<double>::max();

        int best_index = -1;


        for (int j = 0;
             j < static_cast<int>(frame.features.size());
             ++j) {

            const Feature& feature = frame.features[j];


            double distance =
                (map_point.appearance -
                 feature.appearance).norm();


            if (distance < best_distance) {

                best_distance = distance;
                best_index = j;
            }
        }


        if (best_index != -1 &&
            best_distance < MATCH_THRESHOLD) {

            MapMatch match;

            match.map_index = i;
            match.feature_index = best_index;
            match.distance = best_distance;

            matches.push_back(match);
        }
    }


    return matches;
}


bool isFeatureInMap(
    const Feature& feature,
    const std::vector<MapPoint>& map
) {
    const double MATCH_THRESHOLD = 1e-6;

    for (const MapPoint& map_point : map) {
        const double distance =
            (feature.appearance - map_point.appearance).norm();

        if (distance < MATCH_THRESHOLD) {
            return true;
        }
    }

    return false;
}
