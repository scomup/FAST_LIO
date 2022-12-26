/** \brief The ndt cell which containing point mean and covariance.
 * \author Liu.yang
 */
#ifndef NDT_CORE_CELL_H
#define NDT_CORE_CELL_H

#include <eigen3/Eigen/Dense>

class Cell
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	Cell() 
	{
		num_ = 0;
		mean_ .setZero();
		icov_ .setZero();
		//icovL_ .setZero();
		norm_.setZero();
		sum_ .setZero();
		ppt_.setIdentity();
	}

	Eigen::Vector3d mean_;
	Eigen::Vector3d sum_;
	Eigen::Vector3d norm_;
	Eigen::Matrix3d icov_;
	//Eigen::Matrix3d icovL_;
	Eigen::Matrix3d ppt_;
    int num_;
};


#endif