#ifndef ESEKFOM_EKF_HPP1
#define ESEKFOM_EKF_HPP1

#include <vector>
#include <cstdlib>
#include <boost/bind.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "ikd_Tree.h"
#include "so3_math.hpp"

// 该hpp主要包含：广义加减法，前向传播主函数，计算特征点残差及其雅可比，ESKF主函数
namespace ESEKF
{
  const double epsi = 0.001; // ESKF迭代时，如果dx<epsi 认为收敛

  struct HData
  {
    bool valid;                                                // 有效特征点数量是否满足要求
    bool converge;                                             // 迭代时，是否已经收敛
    Eigen::Matrix<double, Eigen::Dynamic, 1> h;                // 残差	(公式(14)中的z)
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_x; // 雅可比矩阵H (公式(14)中的H)
  };

  // the location of state
  constexpr int SZ = 24; // state size
  constexpr int NZ = 12; // noise size
  // the location of state
  constexpr int L_P = 0;
  constexpr int L_R = 3;
  constexpr int L_Rli = 6;
  constexpr int L_Tli = 9;
  constexpr int L_V = 12;
  constexpr int L_Bw = 15;
  constexpr int L_Ba = 18;
  constexpr int L_G = 21;
  // the location of noise
  constexpr int L_Nw = 0;
  constexpr int L_Na = 3;
  constexpr int L_Nbw = 6;
  constexpr int L_Nba = 9;

  // state
  struct State
  {
    Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0); // imu postion in world frame
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity(); // imu rotation in world frame
    Eigen::Matrix3d Ril = Eigen::Matrix3d::Identity(); // rotation from lidar to imu
    Eigen::Vector3d til = Eigen::Vector3d(0, 0, 0); // translation from lidar to imu
    Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0); 
    Eigen::Vector3d bg = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d ba = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d grav = Eigen::Vector3d(0, 0, -G_m_s2);

    // plus for state
    State plus(Eigen::Matrix<double, SZ, 1>& f) const
    {
      State r;
      r.pos = this->pos + f.segment<3>(L_P);
      r.rot = this->rot * Eigen::Quaterniond(SO3Expmap(f.segment<3>(L_R)));
      r.Ril = this->Ril * Eigen::Quaterniond(SO3Expmap(f.segment<3>(L_Rli)));
      r.til = this->til + f.segment<3>(L_Tli);
      r.vel = this->vel + f.segment<3>(L_V);
      r.bg = this->bg + f.segment<3>(L_Bw);
      r.ba = this->ba + f.segment<3>(L_Ba);
      r.grav = this->grav + f.segment<3>(L_G);
      return r;
    }

    // minus for state
    Eigen::Matrix<double, SZ, 1> minus(const State& x2) const
    {
      Eigen::Matrix<double, SZ, 1> r;
      r.segment<3>(L_P) = this->pos - x2.pos;
      r.segment<3>(L_R) = SO3Logmap(x2.rot.transpose() * this->rot);
      r.segment<3>(L_Rli) = SO3Logmap(x2.Ril.transpose() * this->Ril);
      r.segment<3>(L_Tli) = this->til - x2.til;
      r.segment<3>(L_V) = this->vel - x2.vel;
      r.segment<3>(L_Bw) = this->bg - x2.bg;
      r.segment<3>(L_Ba) = this->ba - x2.ba;
      r.segment<3>(L_G) = this->grav - x2.grav;
      return r;
    }

    Eigen::Affine3d getAffine3d() const
    {
      Eigen::Affine3d affine = Eigen::Translation3d(pos) * rot;
      return affine;
    }
  };

  // Input u (IMU)
  struct InputU
  {
    Eigen::Vector3d acc = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d gyro = Eigen::Vector3d(0, 0, 0);
  };

  Eigen::Matrix<double, NZ, NZ> processNoiseCov()
  {
    Eigen::Matrix<double, NZ, NZ> Q = Eigen::MatrixXd::Zero(NZ, NZ);

    Q.block<3, 3>(L_Nw, L_Nw) = 0.0001 * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(L_Na, L_Na) = 0.0001 * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(L_Nbw, L_Nbw) = 0.00001 * Eigen::Matrix3d::Identity();
    Q.block<3, 3>(L_Nba, L_Nba) = 0.00001 * Eigen::Matrix3d::Identity();
    return Q;
  }

  // paper (2) f(x, u): kinematic model
  // x: state
  // u: input
  // update state by input(IMU)
  Eigen::Matrix<double, SZ, 1> f_func(State state, InputU in, double dt)
  {
    Eigen::Matrix<double, SZ, 1> ret = Eigen::Matrix<double, SZ, 1>::Zero();
    ret.segment<3>(L_P) = state.vel;                                    // (7) row 2: velocity
    ret.segment<3>(L_R) = in.gyro - state.bg;                           // (7) row 1: omega
    ret.segment<3>(L_V) = state.rot * (in.acc - state.ba) + state.grav; // (7) row 3: acceleration
    return ret * dt;
  }

  // paper (7) df_fx:
  // Partial derivatives of the kinematic model(f) with respect to current state.
  Eigen::Matrix<double, SZ, SZ> df_dx_func(State s, InputU in, double dt)
  {
    Eigen::Matrix<double, SZ, SZ> Cov = Eigen::Matrix<double, SZ, SZ>::Identity();
    Cov.block<3, 3>(L_P, L_V) = Eigen::Matrix3d::Identity() * dt;       // paper (7) Fx(2,3)
    Eigen::Vector3d acc_corrected = (in.acc - s.ba) * dt;
    Cov.block<3, 3>(L_V, L_R) = -s.rot * skewSymMat(acc_corrected)* dt; // paper(7) Fx(3,1)
    Cov.block<3, 3>(L_V, L_Ba) = -s.rot* dt;                            // paper(7) Fx(3,5)
    Cov.block<3, 3>(L_V, L_G) = Eigen::Matrix3d::Identity()* dt;        // paper(7) Fx(3,6)
    Cov.block<3, 3>(L_R, L_Bw) = -Eigen::Matrix3d::Identity()* dt;      // paper(7) Fx(1,4)
    return Cov;
  }

  // paper (7) df_fw:
  // Partial derivatives of the kinematic model(f) with respect to noise.
  Eigen::Matrix<double, SZ, NZ> df_dw_func(State s, InputU in, double dt)
  {
    Eigen::Matrix<double, SZ, NZ> Cov = Eigen::Matrix<double, SZ, NZ>::Zero();
    Cov.block<3, 3>(L_V, L_Na) = -s.rot * dt;                        //  paper (7) Fw(3,2)
    Cov.block<3, 3>(L_R, L_Nw) = -Eigen::Matrix3d::Identity() * dt;  //  paper (7) Fw(1,1)
    Cov.block<3, 3>(L_Bw, L_Nbw) = Eigen::Matrix3d::Identity() * dt; //  paper (7) Fw(4,3)
    Cov.block<3, 3>(L_Ba, L_Nba) = Eigen::Matrix3d::Identity() * dt; //  paper (7) Fw(5,4)
    return Cov;
  }

  // Error State Extended Kalman Filter
  class esekf
  {
  public:
    typedef Eigen::Matrix<double, SZ, SZ> Cov;     // 24X24的协方差矩阵
    typedef Eigen::Matrix<double, SZ, 1> StateVec; // 24X1的向量
    using HFunc = std::function<void(ESEKF::HData&, ESEKF::State&, PointCloud::Ptr&)>;

    esekf(double R, int maximum_iter, HFunc h_model) : R_(R), maximum_iter_(maximum_iter), h_model_(h_model){};
    ~esekf(){};

    State getState() const
    {
      return x_;
    }

    Cov get_P() const 
    {
      return P_;
    }

    void change_x(State &input_state)
    {
      x_ = input_state;
    }

    void change_P(Cov &input_cov)
    {
      P_ = input_cov;
    }

    // Forward Propagation  III-C
    void predict(double &dt, Eigen::Matrix<double, NZ, NZ> &Q, const InputU &i_in)
    {
      Eigen::Matrix<double, SZ, 1> f = f_func(x_, i_in, dt) ;       // paper (3) f
      Eigen::Matrix<double, SZ, SZ> f_x = df_dx_func(x_, i_in, dt); // paper (7) df/dx
      Eigen::Matrix<double, SZ, NZ> f_w = df_dw_func(x_, i_in, dt); // paper (7) df/dw
      x_ = x_.plus( f );                                            // paper (4)
      P_ = f_x * P_ * f_x.transpose() + f_w * Q * f_w.transpose();  // paper (8) Cov of Forward Propagation
    }

    // 计算每个特征点的残差及H矩阵

    // ESKF
    void iteratedUpdate(PointCloud::Ptr &cloud_ds)
    {
      HData dyn_share;
      dyn_share.valid = true;
      dyn_share.converge = true;
      int t = 0;
      State x_propagated = x_; // 这里的x_和P_分别是经过正向传播后的状态量和协方差矩阵，因为会先调用predict函数再调用这个函数
      Cov P_propagated = P_;

      StateVec dx_new = StateVec::Zero(); // 24X1的向量

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
        K = (H.transpose() * H / R_ + P_.inverse()).inverse() * H.transpose() / R_;                                         // 卡尔曼增益  这里R视为常数
        Eigen::Matrix<double, SZ, 1> dx = K * dyn_share.h + (K * H - Eigen::Matrix<double, SZ, SZ>::Identity()) * dx_new; // 公式(18)

        x_ = x_.plus(dx); // 公式(18)

        dyn_share.converge = true;
        for (int i = 0; i < SZ; i++)
        {
          if (std::fabs(dx[i]) > epsi) // 如果dx>epsi 认为没有收敛
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
          P_ = (Eigen::Matrix<double, SZ, SZ>::Identity() - K * H) * P_; // 公式(19)
          return;
        }
      }
    }

  private:
    State x_;
    Cov P_ = Cov::Identity();
    double R_;
    int maximum_iter_;
    HFunc h_model_;
  };
}
#endif //  ESEKFOM_EKF_HPP1
