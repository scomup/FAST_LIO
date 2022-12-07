#ifndef USE_IKFOM_H
#define USE_IKFOM_H


#include "so3_math.hpp"


//#include "sophus/so3.h"

//该hpp主要包含：状态变量x，输入量u的定义，以及正向传播中相关矩阵的函数

//24维的状态量x
struct state_ikfom
{
	Eigen::Vector3d pos = Eigen::Vector3d(0,0,0);
	Eigen::Quaterniond rot = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
	Eigen::Quaterniond offset_R_L_I = Eigen::Quaterniond(Eigen::Matrix3d::Identity());
	Eigen::Vector3d offset_T_L_I = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d vel = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d bg = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d ba = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d grav = Eigen::Vector3d(0,0,-G_m_s2);
};


//输入u
struct input_ikfom
{
	Eigen::Vector3d acc = Eigen::Vector3d(0,0,0);
	Eigen::Vector3d gyro = Eigen::Vector3d(0,0,0);
};


Eigen::Matrix<double, 12, 12> process_noise_cov()
{
	Eigen::Matrix<double, 12, 12> Q = Eigen::MatrixXd::Zero(12, 12);
	Q.block<3, 3>(0, 0) = 0.0001 * Eigen::Matrix3d::Identity();
	Q.block<3, 3>(3, 3) = 0.0001 * Eigen::Matrix3d::Identity();
	Q.block<3, 3>(6, 6) = 0.00001 * Eigen::Matrix3d::Identity();
	Q.block<3, 3>(9, 9) = 0.00001 * Eigen::Matrix3d::Identity();
	return Q;
}


//paper (2) f: prediction model
//update state by input(IMU)
Eigen::Matrix<double, 24, 1> get_f(state_ikfom state, input_ikfom in)	
{
	//velocity(3)，omega(3),T(3), R(3)，acceleration(3), bw(3), ba(3), p(3)
	Eigen::Matrix<double, 24, 1> f = Eigen::Matrix<double, 24, 1>::Zero();

	f.segment<3>(0) = state.vel; // (7) row 2: velocity
	f.segment<3>(3) = in.gyro - state.bg; // (7) row 1: omega
	f.segment<3>(12) = state.rot.matrix() * (in.acc - state.ba) + state.grav; // (7) row 3: acceleration 

	return f;
}


//对应公式(7)的Fx  注意该矩阵没乘dt，没加单位阵
Eigen::Matrix<double, 24, 24> df_dx(state_ikfom s, input_ikfom in)
{
	Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
	cov.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();	//对应公式(7)第2行第3列   I
	Eigen::Vector3d acc_ = in.acc - s.ba;   	//测量加速度 = a_m - bias	

	cov.block<3, 3>(12, 3) = -s.rot.matrix() * skewSymMat(acc_);		//对应公式(7)第3行第1列
	cov.block<3, 3>(12, 18) = -s.rot.matrix(); 				//对应公式(7)第3行第5列 

	cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();		//对应公式(7)第3行第6列   I
	cov.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();		//对应公式(7)第1行第4列 (简化为-I)
	return cov;
}

//对应公式(7)的Fw  注意该矩阵没乘dt
Eigen::Matrix<double, 24, 12> df_dw(state_ikfom s, input_ikfom in)
{
	Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
	cov.block<3, 3>(12, 3) = -s.rot.matrix();					//对应公式(7)第3行第2列  -R 
	cov.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();		//对应公式(7)第1行第1列  -A(w dt)简化为-I
	cov.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();		//对应公式(7)第4行第3列  I
	cov.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();		//对应公式(7)第5行第4列  I
	return cov;
}


#endif