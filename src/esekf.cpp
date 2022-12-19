#include "ikd_Tree.h"
#include "so3_math.h"
#include "esekf.h"
#include "state.h"


namespace ESEKF
{

  Esekf::Esekf(const double R, const int maximum_iter, const HFunc h_model)
      : R_inv_(1. / R),
        maximum_iter_(maximum_iter),
        h_model_(h_model),
        P_(MatSS::Identity()){};

  // paper (2) f(x, u): kinematic model
  // x: state
  // u: input
  // update state by input(IMU)
  VecS Esekf::f_func(const State& state, const InputU& u, double dt) const 
  {
    VecS f = VecS::Zero();
    f.segment<3>(L_P) = state.vel;                                    // (7) row 2: velocity
    f.segment<3>(L_R) = u.gyr - state.bg;                           // (7) row 1: omega
    f.segment<3>(L_V) = state.rot * (u.acc - state.ba) + state.grav; // (7) row 3: acceleration
    return f * dt;
  }
  // paper (7) df_fx:
  // Partial derivatives of the kinematic model(f) with respect to current state.
  MatSS Esekf::df_dx_func(const State& s, const InputU& u, double dt) const 
  {
    MatSS df_dx = MatSS::Identity();
    df_dx.block<3, 3>(L_P, L_V) = Eigen::Matrix3d::Identity() * dt; // paper (7) Fx(2,3)
    //df_dx.block<3, 3>(L_R, L_R) = SO3Expmap(-(u.gyr - s.bg) * dt); // paper (7) Fx(1,1)
    df_dx.block<3, 3>(L_R, L_Bw) = -Eigen::Matrix3d::Identity() * dt;      // paper(7) Fx(1,4)
    df_dx.block<3, 3>(L_V, L_R) = -s.rot * skewSymMat((u.acc - s.ba) ) * dt; // paper(7) Fx(3,1)
    df_dx.block<3, 3>(L_V, L_Ba) = -s.rot * dt;                            // paper(7) Fx(3,5)
    df_dx.block<3, 3>(L_V, L_G) = Eigen::Matrix3d::Identity() * dt;        // paper(7) Fx(3,6)
    return df_dx;
  }

  // paper (7) df_fw:
  // Partial derivatives of the kinematic model(f) with respect to noise.
  MatSN Esekf::df_dw_func(const State& s, const InputU& u, double dt) const
  {
    MatSN df_dw = MatSN::Zero();
    df_dw.block<3, 3>(L_R, L_Nw) = -Eigen::Matrix3d::Identity() * dt;  //  paper (7) Fw(1,1)
    df_dw.block<3, 3>(L_V, L_Na) = -s.rot * dt;                        //  paper (7) Fw(3,2)
    df_dw.block<3, 3>(L_Bw, L_Nbw) = Eigen::Matrix3d::Identity() * dt; //  paper (7) Fw(4,3)
    df_dw.block<3, 3>(L_Ba, L_Nba) = Eigen::Matrix3d::Identity() * dt; //  paper (7) Fw(5,4)
    return df_dw;
  }

  State Esekf::getState() const
  {
    return x_;
  }

  MatSS Esekf::getP() const
  {
    return P_;
  }

  void Esekf::setState(const State& state)
  {
    x_ = state;
  }

  void Esekf::setP(const MatSS &input_cov)
  {
    P_ = input_cov;
  }

  // Forward Propagation  III-C
  void Esekf::predict(double &dt, MatNN &Q, const InputU &u)
  {
    VecS f = f_func(x_, u, dt);                                  // paper (3) f
    MatSS f_x = df_dx_func(x_, u, dt);                           // paper (7) df/dx
    MatSN f_w = df_dw_func(x_, u, dt);                           // paper (7) df/dw
    x_ = x_.plus(f);                                             // paper (4)
    P_ = f_x * P_ * f_x.transpose() + f_w * Q * f_w.transpose(); // paper (8) MatSS of Forward Propagation
  }

  // ESKF
  void Esekf::iteratedUpdate(PointCloud::Ptr &cloud_ds)
  {
    HData h_data;
    h_data.valid = true;
    h_data.converge = true;
    int t = 0;
    State x_propagated = x_; //forward propagated state, paper (18) x^
    MatSS P_propagated = P_;

    for (int i = -1; i < maximum_iter_; i++) 
    {
      h_data.valid = true;
      
      h_model_(h_data, x_, cloud_ds);

      if (!h_data.valid)
      {
        continue;
      }

      VecS delta_x = x_.minus(x_propagated); // paper (18) x^k - x^

      auto H = h_data.h;
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;
      K = (H.transpose() * H * R_inv_ + P_.inverse()).inverse() * H.transpose() * R_inv_; // paper (20)
      VecS dx = - K * h_data.z - (MatSS::Identity() - K * H ) * delta_x; // paper (18) notice: J_inv = I

      x_ = x_.plus(dx); // update current state. paper (18)

      h_data.converge = true;
      for (int i = 0; i < SZ; i++)
      {
        if (std::fabs(dx[i]) > epsi_)
        {
          h_data.converge = false;
          break;
        }
      }

      if (h_data.converge)
        t++;

      if (!t && i == maximum_iter_ - 2) 
      {
        h_data.converge = true;
      }

      if (t > 1 || i == maximum_iter_ - 1)
      {
        P_ = (MatSS::Identity() - K * H) * P_; // paper (19)
        return;
      }
    }
  }

  MatNN processNoiseCov()
  {
    MatNN Q = Eigen::MatrixXd::Zero(NZ, NZ);

    Q.block<3, 3>(L_Nw, L_Nw) = 0.0001 * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(L_Na, L_Na) = 0.0001 * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(L_Nbw, L_Nbw) = 0.00001 * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(L_Nba, L_Nba) = 0.00001 * Eigen::Matrix3d::Identity();
    return Q;
  }

}