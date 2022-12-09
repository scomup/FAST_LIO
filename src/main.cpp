#include "laserMapping.hpp"

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;

  bool scan_pub_en = false;
  int max_iteration;
  double filter_size_surf_min = 0;
  std::vector<double> extrinT(3, 0);
  std::vector<double> extrinR = {1, 0, 0, 0, 1, 0, 0, 0, 1};

  nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
  nh.param<int>("max_iteration", max_iteration, 4);
  nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
  nh.param<double>("filter_size_map", filter_size_map_min, 0.5);
  nh.param<double>("cube_side_length", cube_len, 200);
  nh.param<float>("mapping/det_range", det_range, 300.f);
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


  memset(ESEKF::point_selected_surf, true, sizeof(ESEKF::point_selected_surf));

  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

  Vec3 tli;
  Mat3 Rli;

  tli << VEC_FROM_ARRAY(extrinT);
  Rli << MAT_FROM_ARRAY(extrinR);
  p_imu->set_extrinsic(tli, Rli);
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
  PointCloud::Ptr feats_undistort(new PointCloud());
  PointCloud::Ptr feats_down_body(new PointCloud());


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


      if (feats_undistort->empty() || (feats_undistort == NULL))
      {
        ROS_WARN("No point, skip this scan!\n");
        continue;
      }

      /*** Segment the map in lidar FOV ***/
      state_point = kf.get_x();
      Vec3 pos_lid = state_point.pos + state_point.rot * state_point.tli; // Lidar point in global frame.
      lasermap_fov_segment(pos_lid);

      /*** downsample the feature points in a scan ***/
      downSizeFilterSurf.setInputCloud(feats_undistort);
      downSizeFilterSurf.filter(*feats_down_body);

      if(init_map(feats_down_body))
      {
        continue;
      }

      int feats_down_size = feats_down_body->points.size();

      /*** ICP and iterated Kalman filter update ***/
      if (feats_down_size < 5)
      {
        ROS_WARN("No point, skip this scan!\n");
        continue;
      }

      /*** iterated state estimation ***/
      kf.iterated_update(feats_down_body, ikdtree, neighbor_array);

      /******* Publish odometry *******/
      publish_odometry(pubOdomAftMapped, kf);

      /*** add the feature points to map kdtree ***/
      map_incremental(feats_down_body);

      /******* Publish points *******/
      if (scan_pub_en)
        publish_frame_world(pubLaserCloudFull, feats_undistort);
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
