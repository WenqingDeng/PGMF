#include "../include/PGMF_filter/PGMF_filter.h"

void Filter::MapPoints_initialization(FeatureManager &f_manager, Mat3d Rs[], Vec3d Ps[], Mat3d &ric, Vec3d &tic)
{
    for (auto &feature : f_manager.feature)
    {
        feature.used_num = feature.feature_per_frame.size();
        if (feature.used_num < 2)
            continue;

        Eigen::MatrixXd A(2 * feature.used_num, 4);
        int i = 0;
        int &start_id = feature.start_frame;
        for(int j = 0; j < feature.used_num; j++)
        {
            Vec3d &observe = feature.feature_per_frame[j].point;

            Mat3d R = Rs[start_id + j] * ric;
            Vec3d t = Rs[start_id + j] * tic + Ps[start_id + j];
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;

            A.row(i++) = observe.x() * P.row(2) - P.row(0);
            A.row(i++) = observe.y() * P.row(2) - P.row(1);
        }
        ROS_ASSERT(i == A.rows());
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(A, Eigen::ComputeThinV).matrixV().rightCols<1>();

        Mappoint mappoint;
        mappoint.state = MappointState::Converged;
        mappoint.count = 0;
        mappoint.cov = Mat3d::Identity() * 1e-4;
        mappoint.position = svd_V.head<3>() / svd_V[3];
        MapPoints[feature.feature_id] = mappoint;
    }

};

void Filter::update(FeatureManager &f_manager, Mat3d Rs[], Vec3d Ps[], Mat3d &ric, Vec3d &tic)
{
    for (auto &feature : f_manager.feature)
    {
        if(feature.estimated_depth > 0 && feature.solve_flag == 0)
        {
            Mappoint mappoint;
            mappoint.count = 0;
            mappoint.state = MappointState::Initial;
            mappoint.cov = Mat3d::Identity();
            mappoint.position = feature.estimated_depth * (Rs[feature.start_frame] * ric) * feature.feature_per_frame[0].point + Rs[feature.start_frame] * tic + Ps[feature.start_frame];
            MapPoints[feature.feature_id] = mappoint;
        }
    }
};
