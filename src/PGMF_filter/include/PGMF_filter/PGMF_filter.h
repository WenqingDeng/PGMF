#pragma once

#include <map>
#include <ros/ros.h>
// #include <Eigen/Dense>
#include <Eigen/Core>
#include <sophus/so3.hpp>
#include <opencv2/opencv.hpp>
// #include <opencv2/core/eigen.hpp>

#include "../../../vins_estimator/src/parameters.h"
#include "../../../vins_estimator/src/feature_manager.h"

using Mat3d = Eigen::Matrix3d;
using Vec3d = Eigen::Vector3d;
using Vec2d = Eigen::Vector2d;
using SO3 = Sophus::SO3d;

extern double OBSERVATION_SPACE;
extern double VARIANCE_NORMTHRESHOLD;

enum MappointState
{
    Initial = 0,
    Estimate = 1,
    Converged = 2,
    Throw = 3,
};

struct Pose
{
    Pose(){};

    Pose(const Mat3d &R, const Vec3d &t)
        : R(R), t(t)
    {};

    Mat3d R;
    Vec3d t;
};

struct Mappoint
{
    int count = 0;
    MappointState state = MappointState::Initial;

    Vec3d position;
    Mat3d cov;
    double an;
    double bn;
};

// Perpendicular-based 3D Gaussian-Uniform Mixture Filter (PGMF)
class Filter
{
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        Filter()
            : UniformDistributionProbability(1.0 / 30), min_tau(0.1 * sqrt(1e-4))
        {};

        void MapPoints_initialization(FeatureManager &f_manager, Mat3d Rs[], Vec3d Ps[], Mat3d &ric, Vec3d &tic);
        
        void NewPointGeneration(FeatureManager &f_manager, Mat3d Rs[], Vec3d Ps[], Mat3d &ric, Vec3d &tic);

        void updateMapPoint(const int &MapPoint_Index, Pose &old_pose, Vec3d &old_point, Pose &new_pose, Vec3d &new_point);

        bool PerpendicularBased_Triangulation(Pose &old_pose, Vec3d &old_point, Pose &new_pose, Vec3d &new_point, Vec3d &new_position, Mat3d &new_cov);

        void GaussianUniform_Mixture_Filter(std::map<int, Mappoint>::iterator &MapPoint, Vec3d &new_position, Mat3d &new_cov);

        void ConvergenceJudgment(std::map<int, Mappoint>::iterator &MapPoint);

        void Remove_Failures();

        void Remove_MapPoint(int MapPoint_Index);

        std::map<int, Mappoint> MapPoints;

    private:
        double UniformDistributionProbability;
        double min_tau;
};
typedef std::shared_ptr<Filter> FilterPtr;