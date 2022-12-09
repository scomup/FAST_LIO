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

  PointCloud::Ptr normvec(new PointCloud(100000, 1));       // 特征点在地图中对应的平面参数(平面的单位法向量,以及当前点到平面距离)
  PointCloud::Ptr laserCloudOri(new PointCloud(100000, 1)); // 有效特征点
  PointCloud::Ptr corr_normvect(new PointCloud(100000, 1)); // 有效特征点对应点法相量
  bool point_selected_surf[100000] = {1};                   // 判断是否是有效特征点

  struct dyn_share_datastruct
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
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity(); // imu rotation in lidar frame
    Eigen::Matrix3d Rli = Eigen::Matrix3d::Identity(); // rotation from imu to lidar
    Eigen::Vector3d tli = Eigen::Vector3d(0, 0, 0); // translation from imu to lidar
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
      r.Rli = this->Rli * Eigen::Quaterniond(SO3Expmap(f.segment<3>(L_Rli)));
      r.tli = this->tli + f.segment<3>(L_Tli);
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
      r.segment<3>(L_Rli) = SO3Logmap(x2.Rli.transpose() * this->Rli);
      r.segment<3>(L_Tli) = this->tli - x2.tli;
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

  Eigen::Matrix<double, NZ, NZ> process_noise_cov()
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

    esekf(){};
    ~esekf(){};

    void init(double R, int maximum_iter, bool extrinsic_est)
    {
      R_ = R;
      maximum_iter_ = maximum_iter;
      extrinsic_est_ = extrinsic_est;
      
    }

    State get_x() const
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
    void h_share_model(dyn_share_datastruct &ekfom_data, PointCloud::Ptr &feats_down_body,
                       KD_TREE<PointType> &ikdtree, std::vector<PointVector> &neighborhoods, bool extrinsic_est)
    {
      int feats_down_size = feats_down_body->points.size();
      laserCloudOri->clear();
      corr_normvect->clear();

      for (int i = 0; i < feats_down_size; i++) // 遍历所有的特征点
      {
        PointType &point_body = feats_down_body->points[i];
        PointType point_world;

        Vec3 p_body(point_body.x, point_body.y, point_body.z);
        // 把Lidar坐标系的点先转到IMU坐标系，再根据前向传播估计的位姿x，转到世界坐标系
        Vec3 p_global(x_.rot * (x_.Rli * p_body + x_.tli) + x_.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto &points_near = neighborhoods[i]; // neighborhoods[i]打印出来发现是按照离point_world距离，从小到大的顺序的vector
        if (ekfom_data.converge)
        {
          // 寻找point_world的最近邻的平面点
          ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
          // 判断是否是有效匹配点，与loam系列类似，要求特征点最近邻的地图点数量>阈值，距离<阈值  满足条件的才置为true
          point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false
                                                                                                                              : true;
        }
        if (!point_selected_surf[i])
          continue; // 如果该点不满足条件  不进行下面步骤

        Eigen::Matrix<float, 4, 1> pabcd; // 平面点信息
        point_selected_surf[i] = false;   // 将该点设置为无效点，用来判断是否满足条件
        // 拟合平面方程ax+by+cz+d=0并求解点到平面距离
        if (esti_plane(pabcd, points_near, 0.1f))
        {
          float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3); // 当前点到平面的距离
          float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());                                                   // 如果残差大于经验阈值，则认为该点是有效点  简言之，距离原点越近的lidar点  要求点到平面的距离越苛刻

          if (s > 0.9) // 如果残差大于阈值，则认为该点是有效点
          {
            point_selected_surf[i] = true;
            normvec->points[i].x = pabcd(0); // 存储平面的单位法向量  以及当前点到平面距离
            normvec->points[i].y = pabcd(1);
            normvec->points[i].z = pabcd(2);
            normvec->points[i].intensity = pd2;
          }
        }
      }

      int effct_feat_num = 0; // 有效特征点的数量
      for (int i = 0; i < feats_down_size; i++)
      {
        if (point_selected_surf[i]) // 对于满足要求的点
        {
          laserCloudOri->points[effct_feat_num] = feats_down_body->points[i]; // 把这些点重新存到laserCloudOri中
          corr_normvect->points[effct_feat_num] = normvec->points[i];         // 存储这些点对应的法向量和到平面的距离
          effct_feat_num++;
        }
      }

      if (effct_feat_num < 1)
      {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
      }

      // 雅可比矩阵H和残差向量的计算
      ekfom_data.h_x = Eigen::MatrixXd::Zero(effct_feat_num, SZ);
      ekfom_data.h.resize(effct_feat_num);

      for (int i = 0; i < effct_feat_num; i++)
      {
        Vec3 point_(laserCloudOri->points[i].x, laserCloudOri->points[i].y, laserCloudOri->points[i].z);
        Mat3 point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_);
        Vec3 point_I_ = x_.Rli * point_ + x_.tli;
        Mat3 point_I_crossmat;
        point_I_crossmat << SKEW_SYM_MATRX(point_I_);

        // 得到对应的平面的法向量
        const PointType &norm_p = corr_normvect->points[i];
        Vec3 norm_vec(norm_p.x, norm_p.y, norm_p.z);

        // 计算雅可比矩阵H
        Vec3 C(x_.rot.transpose() * norm_vec);
        Vec3 A(point_I_crossmat * C);
        if (extrinsic_est)
        {
          Vec3 B(point_crossmat * x_.Rli.transpose() * C);
          ekfom_data.h_x.block<1, NZ>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
          ekfom_data.h_x.block<1, NZ>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        // 残差：点面距离
        ekfom_data.h(i) = -norm_p.intensity;
      }
    }

    // ESKF
    void iterated_update(PointCloud::Ptr &feats_down_body,
                         KD_TREE<PointType> &ikdtree,
                         std::vector<PointVector> &neighborhoods)
    {
      normvec->resize(int(feats_down_body->points.size()));
      neighborhoods.resize(int(feats_down_body->points.size()));

      dyn_share_datastruct dyn_share;
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
        h_share_model(dyn_share, feats_down_body, ikdtree, neighborhoods, extrinsic_est_);

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

        if (!t && i == maximum_iter_ - 2) // 如果迭代了3次还没收敛 强制令成true，h_share_model函数中会重新寻找近邻点
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
    bool extrinsic_est_;
  };

}
#endif //  ESEKFOM_EKF_HPP1
