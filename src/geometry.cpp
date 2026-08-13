#include "geometry.h"

Eigen::Vector3d normalizeImagePoint(
    const Eigen::Vector2d& image_point,
    const Eigen::Matrix3d& K
){
    Eigen::Vector3d homogeneous_point;

    homogeneous_point <<
        image_point.x(),
        image_point.y(),
        1.0;

    return K.inverse() * homogeneous_point;

}


Eigen::Vector2d projectPoint(
    const Eigen::Vector3d& point_camera,
    const Eigen::Matrix3d& K
) {

    Eigen::Vector3d projected =
        K * point_camera;

    return Eigen::Vector2d(
        projected(0) / projected(2),
        projected(1) / projected(2)
    );
}
