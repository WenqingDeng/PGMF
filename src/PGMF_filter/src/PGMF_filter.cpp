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
        mappoint.state = MappointState::Estimate;
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
    // compute new_position
    Vec3d O1X1 = old_pose.R * old_point;
    Vec3d O2X2 = new_pose.R * new_point;
    Vec3d &t1 = old_pose.t;
    Vec3d &t2 = new_pose.t;
    Vec3d t12 = t2 - t1;

    double a11 = O1X1.transpose() * O1X1;
    double a12 = O1X1.transpose() * O2X2;
    double b1 = O1X1.transpose() * t12;
    double a21 = O2X2.transpose() * O1X1;
    double a22 = O2X2.transpose() * O2X2;
    double b2 = O2X2.transpose() * t12;

    double A_det = a12 * a21 - a11 * a22;
    if(std::abs(A_det) < 1e-6)
    {
        // |A| = 0  =>   r(A) = 1  =>  parallel
        return false;
    }
    double s1 = (a12 * b2 - b1 * a22) / A_det; //Cramer's Rule
    double s2 = (a11 * b2 - b1 * a21) / A_det; //Cramer's Rule
    if(s1 <= 0.0 || s2 <= 0.0)
    {
        // ROS_DEBUG_STREAM("compute wrong value s!");
        return false;
    }

    new_position = (s1 * O1X1 + t1 + s2 * O2X2 + t2) / 2.0;

    // compute new_cov
    Vec3d P1P2 = (s2 * O2X2 + t2) - (s1 * O1X1 + t1);
    double tau_alpha = std::max(P1P2.norm(), min_tau); // tau alpha error

    double tau_beta; // tau beta error : one pixel error
    Vec3d t = t12 - P1P2;
    Vec3d a = s1 * O1X1 - t;
    double a_norm = a.norm();
    double t_norm = t.norm();
    double alpha = std::acos(O1X1.dot(t) / (t_norm * O1X1.norm()));
    double beta = std::acos(-a.dot(t) / (t_norm * a_norm)) + std::atan(1.0 / FOCAL);
    double O1P1_plus = t_norm * std::sin(beta) / std::sin(M_PI - alpha - beta);
    tau_beta = std::max(std::abs(O1P1_plus - s1 * O1X1.norm()), min_tau);
    Mat3d tau; tau << tau_beta * tau_beta, 0, 0,
                        0, tau_alpha * tau_alpha, 0,
                        0, 0, tau_beta * tau_beta;

    Vec3d Z_basic = O1X1.normalized();
    Vec3d Y_basic = P1P2.normalized();
    Vec3d X_basic = Y_basic.cross(Z_basic).normalized();
    Mat3d R_basic; R_basic << X_basic, Y_basic, Z_basic;

    new_cov = R_basic * tau * R_basic.transpose();

    return true;
};

void Filter::GaussianUniform_Mixture_Filter(std::map<int, Mappoint>::iterator &MapPoint, Vec3d &new_position, Mat3d &new_cov)
{
    Vec3d &mu = MapPoint->second.position;
    Mat3d &sigma = MapPoint->second.cov;
    double &an = MapPoint->second.an; 
    double &bn = MapPoint->second.bn;

    Mat3d sigma_inv = sigma.inverse();
    Mat3d new_cov_inv = new_cov.inverse();
    Mat3d sigma_prime = (sigma_inv + new_cov_inv).inverse();
    Vec3d mu_prime = sigma_prime * (sigma_inv * mu + new_cov_inv * new_position);

    double C1 = an / (an + bn) * NormalDistribution_PDF(new_position, mu, sigma + new_cov);
    double C2 = bn / (an + bn) * UniformDistributionProbability;

    double C = C1 + C2;
    C1 = C1 / C;
    C2 = C2 / C;

    double f = C1 * (an + 1) / (an + bn + 1) + C2 * an / (an + bn + 1);
    double e = C1 * (an + 1) * (an + 2) / (an + bn + 1) / (an + bn + 2) + C2 * an * (an + 1) / (an + bn + 1) / (an + bn + 2);

    an = (e - f) / (f - e/f);
    bn = (1.0 -f) / f * an;
    Vec3d mu_tem = C1 * mu_prime + C2 * mu;
    sigma = C1 * (sigma_prime + mu_prime * mu_prime.transpose()) + C2 * (sigma + mu * mu.transpose()) - mu_tem * mu_tem.transpose();
    mu = mu_tem;
};

double Filter::NormalDistribution_PDF(const Vec3d& X, const Vec3d& mu, const Mat3d &sigma)
{
    Vec3d diff = X - mu;
    Eigen::LLT<Mat3d> sigma_inv(sigma);
    return double(  1.0 / sqrt(pow(2.0 * M_PI, 3) * sigma.determinant())
                    * exp( -0.5 * diff.transpose() * sigma_inv.solve(diff) )
                );
};

void Filter::ConvergenceJudgment(std::map<int, Mappoint>::iterator &MapPoint)
{
    double pai = MapPoint->second.an / (MapPoint->second.an + MapPoint->second.bn);
    if(pai < OUTLIER_PROBABILITY)
    {
        MapPoint->second.state = MappointState::Throw;
        MapPoints.erase(MapPoint);
    }
    else if(pai > INLIER_PROBABILITY)
    {
        if(MapPoint->second.cov.jacobiSvd(Eigen::EigenvaluesOnly).singularValues()(2) < VARIANCE_NORMTHRESHOLD)
        {
            MapPoint->second.state = MappointState::Converged;
        }
    }
};

void Filter::Remove_MapPoint(int MapPoint_Index)
{
    std::map<int, Mappoint>::iterator MapPoint = MapPoints.find(MapPoint_Index);
    if(MapPoint != MapPoints.end())
    {
        MapPoints.erase(MapPoint);
    }
}
