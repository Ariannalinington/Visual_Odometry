#include "dataset.h"

#include <fstream>
#include <sstream>
#include <stdexcept>


CameraData loadCamera(const std::string& filename) {

    CameraData camera;

    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error(
            "Cannot open camera file: " + filename
        );
    }

    std::string line;

    while (std::getline(file, line)) {

        if (line.find("camera matrix:") != std::string::npos) {

            for (int r = 0; r < 3; ++r) {

                std::getline(file, line);
                std::stringstream ss(line);

                for (int c = 0; c < 3; ++c) {
                    ss >> camera.K(r, c);
                }
            }
        }

        else if (line.find("cam_transform:") != std::string::npos) {

            for (int r = 0; r < 4; ++r) {

                std::getline(file, line);
                std::stringstream ss(line);

                for (int c = 0; c < 4; ++c) {
                    ss >> camera.cam_transform(r, c);
                }
            }
        }

        else if (line.find("z_near:") != std::string::npos) {

            std::stringstream ss(line);
            std::string tag;

            ss >> tag >> camera.z_near;
        }

        else if (line.find("z_far:") != std::string::npos) {

            std::stringstream ss(line);
            std::string tag;

            ss >> tag >> camera.z_far;
        }

        else if (line.find("width:") != std::string::npos) {

            std::stringstream ss(line);
            std::string tag;

            ss >> tag >> camera.width;
        }

        else if (line.find("height:") != std::string::npos) {

            std::stringstream ss(line);
            std::string tag;

            ss >> tag >> camera.height;
        }
    }

    return camera;
}
Measurement loadMeasurement(const std::string& filename) {

    Measurement measurement;

    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error(
            "Cannot open measurement file: " + filename
        );
    }

    std::string line;

    while (std::getline(file, line)) {

        std::stringstream ss(line);

        std::string tag;
        ss >> tag;

        if (tag == "seq:") {

            ss >> measurement.seq;
        }

        else if (tag == "gt_pose:") {

            ss >> measurement.gt_pose(0)
               >> measurement.gt_pose(1)
               >> measurement.gt_pose(2);
        }

        else if (tag == "odom_pose:") {

            ss >> measurement.odom_pose(0)
               >> measurement.odom_pose(1)
               >> measurement.odom_pose(2);
        }

        else if (tag == "point") {

            Feature feature;

            ss >> feature.local_id
               >> feature.actual_id
               >> feature.image_point(0)
               >> feature.image_point(1);

            for (int i = 0; i < 10; ++i) {
                ss >> feature.appearance(i);
            }

            measurement.features.push_back(feature);
        }
    }

    return measurement;
}

std::vector<TrajectoryEntry> loadTrajectory(
    const std::string& filename
) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error(
            "Unable to open trajectory file: " + filename
        );
    }

    std::vector<TrajectoryEntry> trajectory;

    TrajectoryEntry entry;

    while (
        file
        >> entry.pose_id
        >> entry.odom_pose(0)
        >> entry.odom_pose(1)
        >> entry.odom_pose(2)
        >> entry.gt_pose(0)
        >> entry.gt_pose(1)
        >> entry.gt_pose(2)
    ) {
        trajectory.push_back(entry);
    }

    return trajectory;
}


std::vector<WorldPoint> loadWorld(
    const std::string& filename
) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        throw std::runtime_error(
            "Unable to open world file: " + filename
        );
    }

    std::vector<WorldPoint> world;
    WorldPoint point;

    while (
        file
        >> point.id
        >> point.position(0)
        >> point.position(1)
        >> point.position(2)
    ) {
        for (int i = 0; i < 10; ++i) {
            file >> point.appearance(i);
        }

        world.push_back(point);
    }

    return world;
}
