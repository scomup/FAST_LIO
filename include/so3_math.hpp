#ifndef SO3_MATH_HPP
#define SO3_MATH_HPP

#include <Eigen/Core>
#include <math.h>

#define SKEW_SYM_MATRX(v) 0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0

static Eigen::Matrix3d skewSymMat(const Eigen::Vector3d &v)
{
    Eigen::Matrix3d m;
    m << 0, -v(2), v(1),
        v(2), 0, -v(0),
        -v(1), v(0), 0;
    return m;
}

static Eigen::Matrix3d SO3Expmap(const Eigen::Vector3d &ang_vel, const double &dt)
{
    double ang_vel_norm = ang_vel.norm();
    Eigen::Matrix3d Eye3 = Eigen::Matrix3d::Identity();

    if (ang_vel_norm > 0.0000001)
    {
        Eigen::Vector3d r_axis = ang_vel / ang_vel_norm;
        Eigen::Matrix3d K;

        K << SKEW_SYM_MATRX(r_axis);

        double r_ang = ang_vel_norm * dt;

        /// Roderigous Tranformation
        return Eye3 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
    }
    else
    {
        return Eye3;
    }
}

static Eigen::Matrix3d SO3Expmap(const Eigen::Vector3d &r)
{
    Eigen::Matrix3d expr;
    double theta = r.norm();
    if (theta < 1e-7)
    {
        expr = Eigen::Matrix3d::Identity();
    }
    else
    {
        Eigen::Matrix3d skew = skewSymMat(r / theta);
        expr = Eigen::Matrix3d::Identity() + sin(theta) * skew + (1 - cos(theta)) * skew * skew;
    }
    return expr;
}

static Eigen::Vector3d SO3Logmap(const Eigen::Matrix3d &R)
{
    double theta = (R.trace() > 3 - 1e-6) ? 0 : acos((R.trace() - 1) / 2);
    Eigen::Vector3d r(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
    return fabs(theta) < 0.001 ? (0.5 * r) : (0.5 * theta / sin(theta) * r);
}

#endif // SO3_MATH_HPP