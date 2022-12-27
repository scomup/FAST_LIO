#pragma once

#include "so3_math.h"
#include "common.h"


struct State
{
  Vec3 pos = Vec3(0, 0, 0);    // imu postion in world frame
  Mat3 rot = Mat3::Identity(); // imu rotation in world frame
  Mat3 Ril = Mat3::Identity(); // rotation from lidar to imu
  Vec3 til = Vec3(0, 0, 0);    // translation from lidar to imu
  Vec3 vel = Vec3(0, 0, 0);
  Vec3 bg = Vec3(0, 0, 0);
  Vec3 ba = Vec3(0, 0, 0);
  Vec3 grav = Vec3(0, 0, -GRAVITY);

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


