#include "laserMapping.hpp"

#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)

bool flg_exit = false;

void SigHandle(int sig)
{
  flg_exit = true;
  ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;

  bool scan_pub_en = false;
  int max_iteration;
  double filter_size_surf_min = 0;
  bool extrinsic_est_en = true;
  double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;


  std::vector<double> extrinT = {0, 0, 0};
  std::vector<double> extrinR = {1, 0, 0, 0, 1, 0, 0, 0, 1};

  nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
  nh.param<int>("max_iteration", max_iteration, 4);
  nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
  nh.param<double>("filter_size_map", filter_size_map_, 0.5);
  nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
  nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
  nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
  nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
  nh.param<double>("preprocess/blind", p_pre_->blind, 0.01);
  nh.param<int>("preprocess/timestamp_unit", p_pre_->time_unit, US);
  nh.param<int>("point_filter_num", p_pre_->point_filter_num, 2);
  nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);

  nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT, std::vector<double>());
  nh.param<std::vector<double>>("mapping/extrinsic_R", extrinR, std::vector<double>());

  ESEKF::esekf kf;

  kf.init(LASER_POINT_COV, max_iteration, extrinsic_est_en);

  std::shared_ptr<ImuProcess> p_imu(new ImuProcess());


  memset(ESEKF::point_selected_surf, true, sizeof(ESEKF::point_selected_surf));

  pcl::VoxelGrid<PointType> downsampe_filter;
  downsampe_filter.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

  Vec3 til;
  Mat3 Ril;
  til << VEC_FROM_ARRAY(extrinT);
  Ril << MAT_FROM_ARRAY(extrinR);
  p_imu->set_extrinsic(til, Ril);
  p_imu->set_gyr_cov(Vec3(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov(Vec3(acc_cov, acc_cov, acc_cov));
  p_imu->set_gyr_bias_cov(Vec3(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  p_imu->set_acc_bias_cov(Vec3(b_acc_cov, b_acc_cov, b_acc_cov));

  /*** ROS subscribe initialization ***/
  ros::Subscriber sub_pcl = nh.subscribe("/velodyne_points", 200000, cloudCB);
  ros::Subscriber sub_imu = nh.subscribe("/imu/data", 200000, imuCB);
  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
  ros::Publisher pub_cloud_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);
  ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
  ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
  ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
  //------------------------------------------------------------------------------------------------------

  SensorData sensor_data;
  PointCloud::Ptr cloud_deskew(new PointCloud());
  PointCloud::Ptr cloud_ds(new PointCloud());
  double first_lidar_time = 0.0;
  bool flg_first_scan = true;

  signal(SIGINT, SigHandle);
  ros::Rate rate(5000);
  bool status = ros::ok();
  while (status)
  {
    if (flg_exit)
      break;
    ros::spinOnce();
    if (syncData(sensor_data))
    {
      if (flg_first_scan)
      {
        first_lidar_time = sensor_data.lidar_beg_time;
        p_imu->first_lidar_time = first_lidar_time;
        flg_first_scan = false;
        continue;
      }


      p_imu->Process(sensor_data, kf, cloud_deskew); // deskew lidar points. by backward propagation


      if (cloud_deskew->empty() || (cloud_deskew == NULL))
      {
        ROS_WARN("No point, skip this scan!\n");
        continue;
      }
      ekf_inited_ = (sensor_data.lidar_beg_time - first_lidar_time) < INIT_TIME ? false : true;

      /*** Segment the map in lidar FOV ***/
      state_ = kf.get_x();
      //Vec3 pos_lid = state_.pos + state_.rot * state_.til; // Lidar point in global frame.
      //lasermap_fov_segment(pos_lid);

      /*** downsample the feature points in a scan ***/
      downsampe_filter.setInputCloud(cloud_deskew);
      downsampe_filter.filter(*cloud_ds);

      if(initMap(cloud_ds))
      {
        continue;
      }

      int cloud_size = cloud_ds->points.size();

      /*** ICP and iterated Kalman filter update ***/
      if (cloud_size < 5)
      {
        ROS_WARN("No point, skip this scan!\n");
        continue;
      }

      /*** iterated state estimation ***/
      kf.iterated_update(cloud_ds, ikdtree_, neighbor_array_);

      /******* Publish odometry *******/
      publish_odometry(pubOdomAftMapped, kf);

      /*** add the feature points to map kdtree ***/
      updateMap(cloud_ds);

      /******* Publish points *******/
      if (scan_pub_en)
        pubCloud(pub_cloud, cloud_deskew);
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
