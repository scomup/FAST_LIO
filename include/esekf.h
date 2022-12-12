#ifndef ESEKFOM_EKF_HPP1
#define ESEKFOM_EKF_HPP1


#include <vector>
#include <cstdlib>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "state.h"


// 该hpp主要包含：广义加减法，前向传播主函数，计算特征点残差及其雅可比，ESKF主函数
namespace ESEKF
{

  MatNN processNoiseCov();


  // Error State Extended Kalman Filter
  class Esekf
  {
  public:
    using HFunc = std::function<void(ESEKF::HData&, ESEKF::State&, PointCloud::Ptr&)>;

    Esekf(const double R, const int maximum_iter, const HFunc h_model);

    State getState() const;

    MatSS getP() const;

    void setState(const State& state);

    void setP(const MatSS &input_cov);

    // Forward Propagation  III-C
    void predict(double &dt, MatNN &Q, const InputU &i_in);

    // update
    void iteratedUpdate(PointCloud::Ptr &cloud_ds);

  private:
    VecS f_func(const State& state, const InputU& u, double dt) const;

    MatSS df_dx_func(const State& state, const InputU& u, double dt) const;

    MatSN df_dw_func(const State& state, const InputU& u, double dt) const;

    State x_;
    MatSS P_;
    const double R_inv_;
    const int maximum_iter_;
    const HFunc h_model_;
    const double epsi_ = 0.001;
  };
}
#endif //  ESEKFOM_EKF_HPP1
