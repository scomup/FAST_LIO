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

    Esekf(double R, int maximum_iter, HFunc h_model);

    State getState() const;

    MatSS get_P() const;

    void change_x(State &input_state);

    void change_P(MatSS &input_cov);

    // Forward Propagation  III-C
    void predict(double &dt, MatNN &Q, const InputU &i_in);

    // update
    void iteratedUpdate(PointCloud::Ptr &cloud_ds);

  private:
    VecS f_func(State state, InputU in, double dt);

    MatSS df_dx_func(State s, InputU in, double dt);

    MatSN df_dw_func(State s, InputU in, double dt);

    State x_;
    MatSS P_ = MatSS::Identity();
    double R_inv_;
    int maximum_iter_;
    HFunc h_model_;
    const double epsi_ = 0.001;
  };
}
#endif //  ESEKFOM_EKF_HPP1
