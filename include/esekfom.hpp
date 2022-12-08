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

const double epsi = 0.001; // ESKF迭代时，如果dx<epsi 认为收敛

PointCloud::Ptr normvec(new PointCloud(100000, 1));		  // 特征点在地图中对应的平面参数(平面的单位法向量,以及当前点到平面距离)
PointCloud::Ptr laserCloudOri(new PointCloud(100000, 1)); // 有效特征点
PointCloud::Ptr corr_normvect(new PointCloud(100000, 1)); // 有效特征点对应点法相量
bool point_selected_surf[100000] = {1};					  // 判断是否是有效特征点

struct dyn_share_datastruct
{
	bool valid;												   // 有效特征点数量是否满足要求
	bool converge;											   // 迭代时，是否已经收敛
	Eigen::Matrix<double, Eigen::Dynamic, 1> h;				   // 残差	(公式(14)中的z)
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_x; // 雅可比矩阵H (公式(14)中的H)
};

// #include "sophus/so3.h"

// the location of state
constexpr int SZ = 24; //state size
constexpr int NZ = 12; //noise size
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

// state of
struct State
{
	Eigen::Vector3d pos = Eigen::Vector3d(0, 0, 0);
	Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d Rli = Eigen::Matrix3d::Identity();
	Eigen::Vector3d tli = Eigen::Vector3d(0, 0, 0);
	Eigen::Vector3d vel = Eigen::Vector3d(0, 0, 0);
	Eigen::Vector3d bg = Eigen::Vector3d(0, 0, 0);
	Eigen::Vector3d ba = Eigen::Vector3d(0, 0, 0);
	Eigen::Vector3d grav = Eigen::Vector3d(0, 0, -G_m_s2);
};

// Input u (IMU)
struct InputU
{
	Eigen::Vector3d acc = Eigen::Vector3d(0, 0, 0);
	Eigen::Vector3d gyro = Eigen::Vector3d(0, 0, 0);
};

Eigen::Matrix<double, 12, 12> process_noise_cov()
{
	Eigen::Matrix<double, 12, 12> Q = Eigen::MatrixXd::Zero(12, 12);

	Q.block<3, 3>(L_Nw, L_Nw) = 0.0001 * Eigen::Matrix3d::Identity();
	Q.block<3, 3>(L_Na, L_Na) = 0.0001 * Eigen::Matrix3d::Identity();
	Q.block<3, 3>(L_Nbw, L_Nbw) = 0.00001 * Eigen::Matrix3d::Identity();
	Q.block<3, 3>(L_Nba, L_Nba) = 0.00001 * Eigen::Matrix3d::Identity();
	return Q;
}

// paper (2) f: kinematic model
// update state by input(IMU)
Eigen::Matrix<double, 24, 1> f_func(State state, InputU in)
{
	Eigen::Matrix<double, 24, 1> ret = Eigen::Matrix<double, 24, 1>::Zero();
	ret.segment<3>(L_P) = state.vel;											 // (7) row 2: velocity
	ret.segment<3>(L_R) = in.gyro - state.bg;									 // (7) row 1: omega
	ret.segment<3>(L_V) = state.rot * (in.acc - state.ba) + state.grav; // (7) row 3: acceleration
	return ret;
}

// 对应公式(7)的Fx  注意该矩阵没乘dt，没加单位阵
Eigen::Matrix<double, 24, 24> df_dx_func(State s, InputU in)
{
	Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
	cov.block<3, 3>(L_P, L_V) = Eigen::Matrix3d::Identity(); // 对应公式(7)第2行第3列   I
	Eigen::Vector3d acc_corrected = in.acc - s.ba;					 // 测量加速度 = a_m - bias

	cov.block<3, 3>(L_V, L_R) = -s.rot * skewSymMat(acc_corrected); // 对应公式(7)第3行第1列
	cov.block<3, 3>(L_V, L_Ba) = -s.rot;							 // 对应公式(7)第3行第5列

	cov.template block<3, 3>(L_V, L_G) = Eigen::Matrix3d::Identity();	// 对应公式(7)第3行第6列   I
	cov.template block<3, 3>(L_R, L_Bw) = -Eigen::Matrix3d::Identity(); // 对应公式(7)第1行第4列 (简化为-I)
	return cov;
}

// 对应公式(7)的Fw  注意该矩阵没乘dt
Eigen::Matrix<double, 24, 12> df_dw_func(State s, InputU in)
{
	Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
	cov.block<3, 3>(L_V, L_Na) = -s.rot;				// 对应公式(7)第3行第2列  -R
	cov.block<3, 3>(L_R, L_Nw) = -Eigen::Matrix3d::Identity();	// 对应公式(7)第1行第1列  -A(w dt)简化为-I
	cov.block<3, 3>(L_Bw, L_Nbw) = Eigen::Matrix3d::Identity(); // 对应公式(7)第4行第3列  I
	cov.block<3, 3>(L_Ba, L_Nba) = Eigen::Matrix3d::Identity(); // 对应公式(7)第5行第4列  I
	return cov;
}

class esekf
{
public:
	typedef Matrix<double, 24, 24> cov;				// 24X24的协方差矩阵
	typedef Matrix<double, 24, 1> vectorized_state; // 24X1的向量

	esekf(){};
	~esekf(){};

	State get_x()
	{
		return x_;
	}

	cov get_P()
	{
		return P_;
	}

	void change_x(State &input_state)
	{
		x_ = input_state;
	}

	void change_P(cov &input_cov)
	{
		P_ = input_cov;
	}

	// 广义加法  公式(4)
	State boxplus(State x, Eigen::Matrix<double, 24, 1> f)
	{
		State x_r;
		x_r.pos = x.pos + f.segment<3>(L_P);
		x_r.rot = x.rot * Eigen::Quaterniond(SO3Expmap(f.segment<3>(L_R)));
		x_r.Rli = x.Rli * Eigen::Quaterniond(SO3Expmap(f.segment<3>(L_Rli)));
		x_r.tli = x.tli + f.segment<3>(L_Tli);
		x_r.vel = x.vel + f.segment<3>(L_V);
		x_r.bg = x.bg + f.segment<3>(L_Bw);
		x_r.ba = x.ba + f.segment<3>(L_Ba);
		x_r.grav = x.grav + f.segment<3>(L_G);
		return x_r;
	}

	// 广义减法
	vectorized_state boxminus(State x1, State x2)
	{
		vectorized_state x_r = vectorized_state::Zero();
		x_r.segment<3>(L_P) = x1.pos - x2.pos;
		x_r.segment<3>(L_R) = SO3Logmap(x2.rot.transpose() * x1.rot);
		x_r.segment<3>(L_Rli) = SO3Logmap(x2.Rli.transpose() * x1.Rli);
		x_r.segment<3>(L_Tli) = x1.tli - x2.tli;
		x_r.segment<3>(L_V) = x1.vel - x2.vel;
		x_r.segment<3>(L_Bw) = x1.bg - x2.bg;
		x_r.segment<3>(L_Ba) = x1.ba - x2.ba;
		x_r.segment<3>(L_G) = x1.grav - x2.grav;
		return x_r;
	}

	// Forward Propagation  III-C
	void predict(double &dt, Eigen::Matrix<double, 12, 12> &Q, const InputU &i_in)
	{
		Eigen::Matrix<double, 24, 1> f = f_func(x_, i_in);		  // paper (3) f
		Eigen::Matrix<double, 24, 24> f_x = df_dx_func(x_, i_in); // paper (7) df/dx
		Eigen::Matrix<double, 24, 12> f_w = df_dw_func(x_, i_in); // paper (7) df/dw
		x_ = boxplus(x_, f * dt); // 前向传播 公式(4)

		f_x = Matrix<double, 24, 24>::Identity() + f_x * dt; // 之前Fx矩阵里的项没加单位阵，没乘dt   这里补上

		P_ = (f_x)*P_ * (f_x).transpose() + (dt * f_w) * Q * (dt * f_w).transpose(); // 传播协方差矩阵，即公式(8)
	}


	// 计算每个特征点的残差及H矩阵
	void h_share_model(dyn_share_datastruct &ekfom_data, PointCloud::Ptr &feats_down_body,
					   KD_TREE<PointType> &ikdtree, vector<PointVector> &Nearest_Points, bool extrinsic_est)
	{
		int feats_down_size = feats_down_body->points.size();
		laserCloudOri->clear();
		corr_normvect->clear();

		for (int i = 0; i < feats_down_size; i++) // 遍历所有的特征点
		{
			PointType &point_body = feats_down_body->points[i];
			PointType point_world;

			V3D p_body(point_body.x, point_body.y, point_body.z);
			// 把Lidar坐标系的点先转到IMU坐标系，再根据前向传播估计的位姿x，转到世界坐标系
			V3D p_global(x_.rot * (x_.Rli * p_body + x_.tli) + x_.pos);
			point_world.x = p_global(0);
			point_world.y = p_global(1);
			point_world.z = p_global(2);
			point_world.intensity = point_body.intensity;

			vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
			auto &points_near = Nearest_Points[i]; // Nearest_Points[i]打印出来发现是按照离point_world距离，从小到大的顺序的vector
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

			Matrix<float, 4, 1> pabcd;		// 平面点信息
			point_selected_surf[i] = false; // 将该点设置为无效点，用来判断是否满足条件
			// 拟合平面方程ax+by+cz+d=0并求解点到平面距离
			if (esti_plane(pabcd, points_near, 0.1f))
			{
				float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3); // 当前点到平面的距离
				float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());												   // 如果残差大于经验阈值，则认为该点是有效点  简言之，距离原点越近的lidar点  要求点到平面的距离越苛刻

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
				corr_normvect->points[effct_feat_num] = normvec->points[i];			// 存储这些点对应的法向量和到平面的距离
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
		ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 24);
		ekfom_data.h.resize(effct_feat_num);

		for (int i = 0; i < effct_feat_num; i++)
		{
			V3D point_(laserCloudOri->points[i].x, laserCloudOri->points[i].y, laserCloudOri->points[i].z);
			M3D point_crossmat;
			point_crossmat << SKEW_SYM_MATRX(point_);
			V3D point_I_ = x_.Rli * point_ + x_.tli;
			M3D point_I_crossmat;
			point_I_crossmat << SKEW_SYM_MATRX(point_I_);

			// 得到对应的平面的法向量
			const PointType &norm_p = corr_normvect->points[i];
			V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

			// 计算雅可比矩阵H
			V3D C(x_.rot.transpose() * norm_vec);
			V3D A(point_I_crossmat * C);
			if (extrinsic_est)
			{
				V3D B(point_crossmat * x_.Rli.transpose() * C);
				ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
			}
			else
			{
				ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
			}

			// 残差：点面距离
			ekfom_data.h(i) = -norm_p.intensity;
		}
	}


	// ESKF
	void update_iterated_dyn_share_modified(double R, PointCloud::Ptr &feats_down_body,
											KD_TREE<PointType> &ikdtree, 
											vector<PointVector> &Nearest_Points, 
											int maximum_iter, 
											bool extrinsic_est)
	{
		normvec->resize(int(feats_down_body->points.size()));

		dyn_share_datastruct dyn_share;
		dyn_share.valid = true;
		dyn_share.converge = true;
		int t = 0;
		State x_propagated = x_; // 这里的x_和P_分别是经过正向传播后的状态量和协方差矩阵，因为会先调用predict函数再调用这个函数
		cov P_propagated = P_;

		vectorized_state dx_new = vectorized_state::Zero(); // 24X1的向量

		for (int i = -1; i < maximum_iter; i++) // maximum_iter是卡尔曼滤波的最大迭代次数
		{
			dyn_share.valid = true;
			// 计算雅克比，也就是点面残差的导数 H(代码里是h_x)
			h_share_model(dyn_share, feats_down_body, ikdtree, Nearest_Points, extrinsic_est);

			if (!dyn_share.valid)
			{
				continue;
			}

			vectorized_state dx;
			dx_new = boxminus(x_, x_propagated); // 公式(18)中的 x^k - x^

			auto H = dyn_share.h_x;
			Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;
			K = (H.transpose() * H / R + P_.inverse()).inverse() * H.transpose() / R;							 // 卡尔曼增益  这里R视为常数
			Matrix<double, 24, 1> dx_ = K * dyn_share.h + (K * H - Matrix<double, 24, 24>::Identity()) * dx_new; // 公式(18)

			x_ = boxplus(x_, dx_); // 公式(18)

			dyn_share.converge = true;
			for (int i = 0; i < 24; i++)
			{
				if (std::fabs(dx_[i]) > epsi) // 如果dx>epsi 认为没有收敛
				{
					dyn_share.converge = false;
					break;
				}
			}

			if (dyn_share.converge)
				t++;

			if (!t && i == maximum_iter - 2) // 如果迭代了3次还没收敛 强制令成true，h_share_model函数中会重新寻找近邻点
			{
				dyn_share.converge = true;
			}

			if (t > 1 || i == maximum_iter - 1)
			{
				P_ = (Matrix<double, 24, 24>::Identity() - K * H) * P_; // 公式(19)
				return;
			}
		}
	}

private:
	State x_;
	cov P_ = cov::Identity();
};

#endif //  ESEKFOM_EKF_HPP1
