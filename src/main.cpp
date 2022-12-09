#include "laserMapping.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;

  bool scan_pub_en = false;
  int max_iteration;

  nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
  nh.param<int>("max_iteration", max_iteration, 4);
  nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
  nh.param<double>("filter_size_corner", filter_size_corner_min, 0.5);
  nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
  nh.param<double>("filter_size_map", filter_size_map_min, 0.5);
  nh.param<double>("cube_side_length", cube_len, 200);
  nh.param<float>("mapping/det_range", DET_RANGE, 300.f);
  nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
  nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
  nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
  nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
  nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
  nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);

  nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT, std::vector<double>());
  nh.param<std::vector<double>>("mapping/extrinsic_R", extrinR, std::vector<double>());

  ESEKF::esekf kf;

  kf.init(LASER_POINT_COV, max_iteration, extrinsic_est_en);

  /*** variables definition ***/
  int effect_feat_num = 0, frame_num = 0;
  double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
  bool flg_EKF_converged, EKF_stop_flg = 0;


  memset(ESEKF::point_selected_surf, true, sizeof(ESEKF::point_selected_surf));
  memset(res_last, -1000.0f, sizeof(res_last));
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);
  p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
  p_imu->set_gyr_cov(Vec3(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov(Vec3(acc_cov, acc_cov, acc_cov));
  p_imu->set_gyr_bias_cov(Vec3(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  p_imu->set_acc_bias_cov(Vec3(b_acc_cov, b_acc_cov, b_acc_cov));


  /*** ROS subscribe initialization ***/
  ros::Subscriber sub_pcl = nh.subscribe("/velodyne_points", 200000, standard_pcl_cbk);
  ros::Subscriber sub_imu = nh.subscribe("/imu/data", 200000, imu_cbk);
  ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
  ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
  ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
  ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
  ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
  //------------------------------------------------------------------------------------------------------

  MeasureGroup Measures;


  signal(SIGINT, SigHandle);
  ros::Rate rate(5000);
  bool status = ros::ok();
  while (status)
  {
    if (flg_exit)
      break;
    ros::spinOnce();
    if (sync_packages(Measures))
    {
      if (flg_first_scan)
      {
        first_lidar_time = Measures.lidar_beg_time;
        p_imu->first_lidar_time = first_lidar_time;
        flg_first_scan = false;
        continue;
      }


      p_imu->Process(Measures, kf, feats_undistort); // deskew lidar points. by backward propagation

      state_point = kf.get_x();
      pos_lid = state_point.pos + state_point.rot * state_point.tli; // Lidar point in global frame.

      if (feats_undistort->empty() || (feats_undistort == NULL))
      {
        ROS_WARN("No point, skip this scan!\n");
        continue;
      }

      /*** Segment the map in lidar FOV ***/
      lasermap_fov_segment();

      /*** downsample the feature points in a scan ***/
      downSizeFilterSurf.setInputCloud(feats_undistort);
      downSizeFilterSurf.filter(*feats_down_body);
      feats_down_size = feats_down_body->points.size();
      /*** initialize the map kdtree ***/
      if (ikdtree.Root_Node == nullptr)
      {
        if (feats_down_size > 5)
        {
          ikdtree.set_downsample_param(filter_size_map_min);
          feats_down_world->resize(feats_down_size);
          for (int i = 0; i < feats_down_size; i++)
          {
            pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
          }
          ikdtree.Build(feats_down_world->points);
        }
        continue;
      }
      int featsFromMapNum = ikdtree.validnum();
      kdtree_size_st = ikdtree.size();

      // std::cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<std::endl;

      /*** ICP and iterated Kalman filter update ***/
      if (feats_down_size < 5)
      {
        ROS_WARN("No point, skip this scan!\n");
        continue;
      }

      feats_down_world->resize(feats_down_size);

      pointSearchInd_surf.resize(feats_down_size);
      Nearest_Points.resize(feats_down_size);


      /*** iterated state estimation ***/
      
      kf.iterated_update(feats_down_body, ikdtree, Nearest_Points);

      state_point = kf.get_x();
      Eigen::Quaterniond q = Eigen::Quaterniond(state_point.rot);
      geoQuat.x = q.x();
      geoQuat.y = q.y();
      geoQuat.z = q.z();
      geoQuat.w = q.w();

      double t_update_end = omp_get_wtime();

      /******* Publish odometry *******/
      publish_odometry(pubOdomAftMapped, kf);

      /*** add the feature points to map kdtree ***/
      map_incremental();

      /******* Publish points *******/
      if (scan_pub_en)
        publish_frame_world(pubLaserCloudFull);
    }

    status = ros::ok();
    rate.sleep();
  }



  return 0;
}
