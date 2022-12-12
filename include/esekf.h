#ifndef ESEKFOM_EKF_HPP1
#define ESEKFOM_EKF_HPP1


#include <vector>
#include <cstdlib>
#include <boost/bind.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "state.h"


// 该hpp主要包含：广义加减法，前向传播主函数，计算特征点残差及其雅可比，ESKF主函数
namespace ESEKF
{

  Eigen::Matrix<double, NZ, NZ> processNoiseCov();

  Eigen::Matrix<double, SZ, 1> f_func(State state, InputU in, double dt);

  Eigen::Matrix<double, SZ, SZ> df_dx_func(State s, InputU in, double dt);

  Eigen::Matrix<double, SZ, NZ> df_dw_func(State s, InputU in, double dt);

  // Error State Extended Kalman Filter
  class esekf
  {
  public:
    typedef Eigen::Matrix<double, SZ, SZ> Cov;     // 24X24的协方差矩阵
    typedef Eigen::Matrix<double, SZ, 1> StateVec; // 24X1的向量
    using HFunc = std::function<void(ESEKF::HData&, ESEKF::State&, PointCloud::Ptr&)>;

    esekf(double R, int maximum_iter, HFunc h_model);

    State getState() const;

    Cov get_P() const;

    void change_x(State &input_state);

    void change_P(Cov &input_cov);

    // Forward Propagation  III-C
    void predict(double &dt, Eigen::Matrix<double, NZ, NZ> &Q, const InputU &i_in);

    // update
    void iteratedUpdate(PointCloud::Ptr &cloud_ds);

  private:
    State x_;
    Cov P_ = Cov::Identity();
    double R_;
    int maximum_iter_;
    HFunc h_model_;
  };
}
#endif //  ESEKFOM_EKF_HPP1
