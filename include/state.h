#pragma once

#include "so3_math.h"
#include "common_lib.h"

// 该hpp主要包含：广义加减法，前向传播主函数，计算特征点残差及其雅可比，ESKF主函数
namespace ESEKF
{

  struct HData
  {
    bool valid;
    bool converge;
    Eigen::Matrix<double, Eigen::Dynamic, 1> z;                // residual
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h;   // jacobian H
  };

  // the location of state
  constexpr int SZ = 24; // state size
  constexpr int NZ = 12; // noise size
  // the location of parameters in state vector
  constexpr int L_P = 0;
  constexpr int L_R = 3;
  constexpr int L_Rli = 6;
  constexpr int L_Tli = 9;
  constexpr int L_V = 12;
  constexpr int L_Bw = 15;
  constexpr int L_Ba = 18;
  constexpr int L_G = 21;
  // the location of parameters in noise vector
  constexpr int L_Nw = 0;
  constexpr int L_Na = 3;
  constexpr int L_Nbw = 6;
  constexpr int L_Nba = 9;

  using MatSS = Eigen::Matrix<double, SZ, SZ>; // 24X24 Cov Mat
  using MatSN = Eigen::Matrix<double, SZ, NZ>; // 24X12 Cov Mat
  using MatNN = Eigen::Matrix<double, NZ, NZ>; // 12X12 Cov Mat
  using VecS = Eigen::Matrix<double, SZ, 1>;   // 24X1 Vec

  // Input u (IMU)
  struct InputU
  {
    Vec3 acc = Vec3(0, 0, 0);
    Vec3 gyro = Vec3(0, 0, 0);
  };

  struct State
  {
    Vec3 pos = Vec3(0, 0, 0);    // imu postion in world frame
    Mat3 rot = Mat3::Identity(); // imu rotation in world frame
    Mat3 Ril = Mat3::Identity(); // rotation from lidar to imu
    Vec3 til = Vec3(0, 0, 0);    // translation from lidar to imu
    Vec3 vel = Vec3(0, 0, 0);
    Vec3 bg = Vec3(0, 0, 0);
    Vec3 ba = Vec3(0, 0, 0);
    Vec3 grav = Vec3(0, 0, -G_m_s2);

    // plus for state
    State plus(VecS &f) const
    {
      State r;
      r.pos = this->pos + f.segment<3>(L_P);
      r.rot = this->rot * SO3Expmap(f.segment<3>(L_R));
      r.Ril = this->Ril * SO3Expmap(f.segment<3>(L_Rli));
      r.til = this->til + f.segment<3>(L_Tli);
      r.vel = this->vel + f.segment<3>(L_V);
      r.bg = this->bg + f.segment<3>(L_Bw);
      r.ba = this->ba + f.segment<3>(L_Ba);
      r.grav = this->grav + f.segment<3>(L_G);
      return r;
    }

    // minus for state
    VecS minus(const State &x2) const
    {
      VecS r;
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
  };

}
