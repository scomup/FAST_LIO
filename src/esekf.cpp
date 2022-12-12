#include "ikd_Tree.h"
#include "so3_math.h"
#include "esekf.h"
#include "state.h"

// 该hpp主要包含：广义加减法，前向传播主函数，计算特征点残差及其雅可比，ESKF主函数
namespace ESEKF
{

  MatNN processNoiseCov()
  {
    MatNN Q = Eigen::MatrixXd::Zero(NZ, NZ);

    Q.block<3, 3>(L_Nw, L_Nw) = 0.0001 * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(L_Na, L_Na) = 0.0001 * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(L_Nbw, L_Nbw) = 0.00001 * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(L_Nba, L_Nba) = 0.00001 * Eigen::Matrix3d::Identity();
    return Q;
  }

  Esekf::Esekf(double R, int maximum_iter, HFunc h_model) : R_inv_(1. / R), maximum_iter_(maximum_iter), h_model_(h_model){};

  // paper (2) f(x, u): kinematic model
  // x: state
  // u: input
  // update state by input(IMU)
  VecS Esekf::f_func(State state, InputU in, double dt)
  {
    VecS f = VecS::Zero();
    f.segment<3>(L_P) = state.vel;                                    // (7) row 2: velocity
    f.segment<3>(L_R) = in.gyro - state.bg;                           // (7) row 1: omega
    f.segment<3>(L_V) = state.rot * (in.acc - state.ba) + state.grav; // (7) row 3: acceleration
    return f * dt;
  }

  // paper (7) df_fx:
  // Partial derivatives of the kinematic model(f) with respect to current state.
  MatSS Esekf::df_dx_func(State s, InputU in, double dt)
  {
    MatSS df_dx = MatSS::Identity();
    df_dx.block<3, 3>(L_P, L_V) = Eigen::Matrix3d::Identity() * dt; // paper (7) Fx(2,3)
    Eigen::Vector3d acc_corrected = (in.acc - s.ba) * dt;
    df_dx.block<3, 3>(L_V, L_R) = -s.rot * skewSymMat(acc_corrected) * dt; // paper(7) Fx(3,1)
    df_dx.block<3, 3>(L_V, L_Ba) = -s.rot * dt;                            // paper(7) Fx(3,5)
    df_dx.block<3, 3>(L_V, L_G) = Eigen::Matrix3d::Identity() * dt;        // paper(7) Fx(3,6)
    df_dx.block<3, 3>(L_R, L_Bw) = -Eigen::Matrix3d::Identity() * dt;      // paper(7) Fx(1,4)
    return df_dx;
  }

  // paper (7) df_fw:
  // Partial derivatives of the kinematic model(f) with respect to noise.
  MatSN Esekf::df_dw_func(State s, InputU in, double dt)
  {
    MatSN df_dw = MatSN::Zero();
    df_dw.block<3, 3>(L_V, L_Na) = -s.rot * dt;                        //  paper (7) Fw(3,2)
    df_dw.block<3, 3>(L_R, L_Nw) = -Eigen::Matrix3d::Identity() * dt;  //  paper (7) Fw(1,1)
    df_dw.block<3, 3>(L_Bw, L_Nbw) = Eigen::Matrix3d::Identity() * dt; //  paper (7) Fw(4,3)
    df_dw.block<3, 3>(L_Ba, L_Nba) = Eigen::Matrix3d::Identity() * dt; //  paper (7) Fw(5,4)
    return df_dw;
  }

  State Esekf::getState() const
  {
    return x_;
  }

  MatSS Esekf::get_P() const
  {
    return P_;
  }

  void Esekf::change_x(State &input_state)
  {
    x_ = input_state;
  }

  void Esekf::change_P(MatSS &input_cov)
  {
    P_ = input_cov;
  }

  // Forward Propagation  III-C
  void Esekf::predict(double &dt, MatNN &Q, const InputU &u)
  {
    VecS f = f_func(x_, u, dt);                              // paper (3) f
    MatSS f_x = df_dx_func(x_, u, dt);   // paper (7) df/dx
    MatSN f_w = df_dw_func(x_, u, dt);   // paper (7) df/dw
    x_ = x_.plus(f);                                             // paper (4)
    P_ = f_x * P_ * f_x.transpose() + f_w * Q * f_w.transpose(); // paper (8) MatSS of Forward Propagation
  }

  // 计算每个特征点的残差及H矩阵

  // ESKF
  void Esekf::iteratedUpdate(PointCloud::Ptr &cloud_ds)
  {
    HData dyn_share;
    dyn_share.valid = true;
    dyn_share.converge = true;
    int t = 0;
    State x_propagated = x_; // 这里的x_和P_分别是经过正向传播后的状态量和协方差矩阵，因为会先调用predict函数再调用这个函数
    MatSS P_propagated = P_;

    VecS dx_new = VecS::Zero(); // 24X1的向量

    for (int i = -1; i < maximum_iter_; i++) // maximum_iter是卡尔曼滤波的最大迭代次数
    {
      dyn_share.valid = true;
      // 计算雅克比，也就是点面残差的导数 H(代码里是h_x)
      h_model_(dyn_share, x_, cloud_ds);

      if (!dyn_share.valid)
      {
        continue;
      }

      dx_new = x_.minus(x_propagated); // 公式(18)中的 x^k - x^

      auto H = dyn_share.h_x;
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;
      K = (H.transpose() * H * R_inv_ + P_.inverse()).inverse() * H.transpose() * R_inv_;           // paper (20)
      VecS dx = K * dyn_share.h + (K * H - MatSS::Identity()) * dx_new; // paper (18)

      x_ = x_.plus(dx); // 公式(18)

      dyn_share.converge = true;
      for (int i = 0; i < SZ; i++)
      {
        if (std::fabs(dx[i]) > epsi_) // 如果dx>epsi_ 认为没有收敛
        {
          dyn_share.converge = false;
          break;
        }
      }

      if (dyn_share.converge)
        t++;

      if (!t && i == maximum_iter_ - 2) // 如果迭代了3次还没收敛 强制令成true，h_model函数中会重新寻找近邻点
      {
        dyn_share.converge = true;
      }

      if (t > 1 || i == maximum_iter_ - 1)
      {
        P_ = (MatSS::Identity() - K * H) * P_; // 公式(19)
        return;
      }
    }
  }

}