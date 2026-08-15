#include "initialization.h"
#include "geometry.h"

#include <Eigen/SVD>
#include <stdexcept>


Eigen::Matrix3d estimateEssentialMatrix(
    const Measurement& frame0,
    const Measurement& frame1,
    const std::vector<Match>& matches,
    const Eigen::Matrix3d& K
) {


    if (matches.size() < 8) {
        throw std::runtime_error(
            "At least 8 matches are required to estimate E"
        );
    }


    Eigen::MatrixXd A(matches.size(), 9);


    for (int i = 0; i < matches.size(); ++i) {

        const Match& match = matches[i];

        const Feature& f0 =
            frame0.features[match.index_frame0];

        const Feature& f1 =
            frame1.features[match.index_frame1];


        Eigen::Vector3d p0 =
            normalizeImagePoint(
                f0.image_point,
                K
            );

        Eigen::Vector3d p1 =
            normalizeImagePoint(
                f1.image_point,
                K
            );


        double x  = p0(0);
        double y  = p0(1);

        double xp = p1(0);
        double yp = p1(1);


        A.row(i) <<
            xp * x,
            xp * y,
            xp,
            yp * x,
            yp * y,
            yp,
            x,
            y,
            1.0;
    }


    Eigen::JacobiSVD<Eigen::MatrixXd> svd_A(
        A,
        Eigen::ComputeFullV
    );


    Eigen::VectorXd e =
        svd_A.matrixV().col(8);


    Eigen::Matrix3d E;

    E <<
        e(0), e(1), e(2),
        e(3), e(4), e(5),
        e(6), e(7), e(8);


    Eigen::JacobiSVD<Eigen::Matrix3d> svd_E(
        E,
        Eigen::ComputeFullU |
        Eigen::ComputeFullV
    );


    Eigen::Matrix3d U =
        svd_E.matrixU();

    Eigen::Matrix3d V =
        svd_E.matrixV();

    Eigen::Vector3d singular_values =
        svd_E.singularValues();


    double s =
        0.5 *
        (singular_values(0) +
         singular_values(1));


    Eigen::Matrix3d Sigma =
        Eigen::Matrix3d::Zero();

    Sigma(0, 0) = s;
    Sigma(1, 1) = s;
    Sigma(2, 2) = 0.0;


    E =
        U *
        Sigma *
        V.transpose();


    E /= E.norm();


    return E;
}
std::vector<MotionCandidate> decomposeEssentialMatrix(
    const Eigen::Matrix3d& E
) {

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        E,
        Eigen::ComputeFullU |
        Eigen::ComputeFullV
    );


    Eigen::Matrix3d U =
        svd.matrixU();

    Eigen::Matrix3d V =
        svd.matrixV();


    if (U.determinant() < 0) {
        U.col(2) *= -1;
    }

    if (V.determinant() < 0) {
        V.col(2) *= -1;
    }


    Eigen::Matrix3d W;

    W <<
         0, -1, 0,
         1,  0, 0,
         0,  0, 1;


    Eigen::Matrix3d R1 =
        U * W * V.transpose();

    Eigen::Matrix3d R2 =
        U * W.transpose() * V.transpose();


    Eigen::Vector3d t =
        U.col(2);


    std::vector<MotionCandidate> candidates;


    MotionCandidate c1;
    c1.R = R1;
    c1.t = t;
    candidates.push_back(c1);


    MotionCandidate c2;
    c2.R = R1;
    c2.t = -t;
    candidates.push_back(c2);


    MotionCandidate c3;
    c3.R = R2;
    c3.t = t;
    candidates.push_back(c3);


    MotionCandidate c4;
    c4.R = R2;
    c4.t = -t;
    candidates.push_back(c4);


    return candidates;
}


Eigen::Vector3d triangulatePoint(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t
) {


    Eigen::Matrix<double, 3, 4> P0;
    P0.setZero();
    P0.block<3,3>(0,0) = Eigen::Matrix3d::Identity();


    Eigen::Matrix<double, 3, 4> P1;
    P1.block<3,3>(0,0) = R;
    P1.col(3) = t;


    Eigen::Matrix4d A;

    A.row(0) =
        p0(0) * P0.row(2) - P0.row(0);

    A.row(1) =
        p0(1) * P0.row(2) - P0.row(1);

    A.row(2) =
        p1(0) * P1.row(2) - P1.row(0);

    A.row(3) =
        p1(1) * P1.row(2) - P1.row(1);


    Eigen::JacobiSVD<Eigen::Matrix4d> svd(
        A,
        Eigen::ComputeFullV
    );

    Eigen::Vector4d X_h =
        svd.matrixV().col(3);


    Eigen::Vector3d X =
        X_h.head<3>() / X_h(3);


    return X;
}


std::vector<MapPoint> buildInitialMap(
    const Measurement& frame0,
    const Measurement& frame1,
    const std::vector<Match>& matches,
    const Eigen::Matrix3d& K,
    const MotionCandidate& motion
) {

    std::vector<MapPoint> map;


    for (const Match& match : matches) {

        const Feature& f0 =
            frame0.features[match.index_frame0];

        const Feature& f1 =
            frame1.features[match.index_frame1];


        Eigen::Vector3d p0 =
            normalizeImagePoint(
                f0.image_point,
                K
            );

        Eigen::Vector3d p1 =
            normalizeImagePoint(
                f1.image_point,
                K
            );


        Eigen::Vector3d X =
            triangulatePoint(
                p0,
                p1,
                motion.R,
                motion.t
            );


        Eigen::Vector3d X1 =
            motion.R * X +
            motion.t;


        if (X(2) > 0 &&
            X1(2) > 0) {

            MapPoint map_point;

            map_point.position = X;

            map_point.appearance =
                f0.appearance;


            map_point.actual_id =
                f0.actual_id;


            map.push_back(map_point);
        }
    }


    return map;
}


int addNewLandmarks(
    const Measurement& previous_frame,
    const Measurement& current_frame,
    const std::vector<Match>& consecutive_matches,
    const Eigen::Matrix4d& T_prev_w,
    const Eigen::Matrix4d& T_current_w,
    const Eigen::Matrix3d& K,
    std::vector<MapPoint>& map
) {
    int added_points = 0;

    const Eigen::Matrix4d T_current_prev =
        T_current_w * T_prev_w.inverse();
    const Eigen::Matrix3d R = T_current_prev.block<3,3>(0,0);
    const Eigen::Vector3d t = T_current_prev.block<3,1>(0,3);

    for (const Match& match : consecutive_matches) {
        const Feature& previous_feature =
            previous_frame.features[match.index_frame0];
        const Feature& current_feature =
            current_frame.features[match.index_frame1];

        if (isFeatureInMap(previous_feature, map)) {
            continue;
        }

        const Eigen::Vector3d p_prev = normalizeImagePoint(
            previous_feature.image_point, K
        );
        const Eigen::Vector3d p_current = normalizeImagePoint(
            current_feature.image_point, K
        );
        const Eigen::Vector3d X_prev = triangulatePoint(
            p_prev, p_current, R, t
        );
        const Eigen::Vector3d X_current = R * X_prev + t;

        if (!X_prev.allFinite() || !X_current.allFinite() ||
            X_prev.z() <= 0.0 || X_current.z() <= 0.0) {
            continue;
        }

        Eigen::Vector4d X_prev_h;
        X_prev_h << X_prev, 1.0;
        const Eigen::Vector4d X_world_h =
            T_prev_w.inverse() * X_prev_h;

        MapPoint new_point;
        new_point.position = X_world_h.head<3>();
        new_point.appearance = current_feature.appearance;
        new_point.actual_id = current_feature.actual_id;
        map.push_back(new_point);
        added_points++;
    }

    return added_points;
}
