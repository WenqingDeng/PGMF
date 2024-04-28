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
        mappoint.cov = Mat3d::Identity() * 1e-4;
        mappoint.position = svd_V.head<3>() / svd_V[3];
        MapPoints[feature.feature_id] = mappoint;

        feature.solve_flag = 1;
    }

};

void Filter::NewPointGeneration(FeatureManager &f_manager, Mat3d Rs[], Vec3d Ps[], Mat3d &ric, Vec3d &tic)
{
    for (auto &feature : f_manager.feature)
    {
        if(feature.solve_flag == 3)
        {
            Mappoint mappoint;
            mappoint.state = MappointState::Estimate;
            mappoint.cov = Mat3d::Identity();
            mappoint.position = feature.initial_guess_of_position;
            MapPoints[feature.feature_id] = mappoint;
        }
    }
};

void Filter::updateMapPoint(const int &MapPoint_Index, Pose &old_pose, Vec3d &old_point, Pose &new_pose, Vec3d &new_point)
{
    std::map<int, Mappoint>::iterator MapPoint = MapPoints.find(MapPoint_Index);
    if(MapPoint != MapPoints.end())
    {
        if(MapPoint->second.state == MappointState::Converged)
            return;

        Mat3d new_cov;
        Vec3d new_position;
        if(PerpendicularBased_Triangulation(old_pose, old_point, new_pose, new_point, new_position, new_cov))
        {
            if(MapPoint->second.state == MappointState::Estimate)
            {
                GaussianUniform_Mixture_Filter(MapPoint, new_position, new_cov);
                ConvergenceJudgment(MapPoint);
            }
        }
    }
};

bool Filter::PerpendicularBased_Triangulation(Pose &old_pose, Vec3d &old_point, Pose &new_pose, Vec3d &new_point, Vec3d &new_position, Mat3d &new_cov)
{
    //
    return true;
};

void Filter::GaussianUniform_Mixture_Filter(std::map<int, Mappoint>::iterator &MapPoint, Vec3d &new_position, Mat3d &new_cov)
{
    //
};

void Filter::ConvergenceJudgment(std::map<int, Mappoint>::iterator &MapPoint)
{
    //
};

void Filter::Remove_Failures()
{
    //
};

void Filter::Remove_MapPoint(int MapPoint_Index)
{
    std::map<int, Mappoint>::iterator MapPoint = MapPoints.find(MapPoint_Index);
    if(MapPoint != MapPoints.end())
    {
        MapPoints.erase(MapPoint);
    }
}
