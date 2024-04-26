#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 1>
{
  public:
    ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j, const Eigen::Vector3d &_Pi, const Eigen::Quaterniond &_Qi)
        : pts_i(_pts_i), pts_j(_pts_j), Pi(_Pi), Qi(_Qi)
    {
    };
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    Eigen::Vector3d pts_i, pts_j;
    Eigen::Vector3d Pi;
    Eigen::Quaterniond Qi;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};
