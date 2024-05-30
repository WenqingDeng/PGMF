#pragma once

#include <ros/assert.h>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include "../utility/utility.h"
#include "../utility/tic_toc.h"
#include "../parameters.h"

class ProjectionFactor : public ceres::SizedCostFunction<2, 7, 7, 3>
{
  public:
    ProjectionFactor(const Eigen::Vector3d &_pts_j)
        : pts_j(_pts_j)
    {
    };
    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    const Eigen::Vector3d pts_j;
    static Eigen::Matrix2d sqrt_info;
    static double sum_t;
};
