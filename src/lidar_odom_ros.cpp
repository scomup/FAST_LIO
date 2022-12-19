#define PCL_NO_PRECOMPILE

#include "lidar_odom_ros.h"

const bool time_cmp(PointType &x, PointType &y) { return (x.time < y.time); };
bool flg_exit = false;

void SigHandle(int sig)
{
  flg_exit = true;
  ROS_WARN("catch sig %d", sig);
}


LidarOdomROS::LidarOdomROS()
{
  int max_iteration;
  double filter_size_map;
  double filter_size_surf_min = 0;
  double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
  bool extrinsic_est = true;
  std::vector<double> extrin_trans = {0, 0, 0};
  std::vector<double> extrin_rot = {1, 0, 0, 0, 1, 0, 0, 0, 1};

  double blind;
  int time_unit;
  int point_filter_num;

  nh_.param<bool>("publish/scan_publish_en", scan_pub_, true);
  nh_.param<int>("max_iteration", max_iteration, 4);
  nh_.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
  nh_.param<double>("filter_size_map", filter_size_map, 0.5);
  nh_.param<double>("mapping_/gyr_cov", gyr_cov, 0.1);
  nh_.param<double>("mapping_/acc_cov", acc_cov, 0.1);
  nh_.param<double>("mapping_/b_gyr_cov", b_gyr_cov, 0.0001);
  nh_.param<double>("mapping_/b_acc_cov", b_acc_cov, 0.0001);
  nh_.param<double>("preprocess/blind", blind, 0.01);
  nh_.param<int>("preprocess/timestamp_unit", time_unit, US);
  nh_.param<int>("point_filter_num", point_filter_num, 2);
  nh_.param<bool>("mapping_/extrinsic_est_en", extrinsic_est, false);
  nh_.param<std::vector<double>>("mapping_/extrinsic_T", extrin_trans, std::vector<double>());
  nh_.param<std::vector<double>>("mapping_/extrinsic_R", extrin_rot, std::vector<double>());


  p_pre_ = boost::make_shared<Preprocess>(blind, time_unit, point_filter_num);;
  mapping_ = boost::make_shared<Mapping>(extrinsic_est, filter_size_map);
  auto h_model = [this](ESEKF::HData &ekfom_data,
                            ESEKF::State &x,
                            PointCloud::Ptr &cloud)                       
    {this->mapping_->point2PlaneModel(ekfom_data, x, cloud); };

  kf_ = boost::make_shared<ESEKF::Esekf>(0.001, max_iteration, h_model);
  p_imu_ = boost::make_shared<IMUPropagation>();

  downsampe_filter_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

  Vec3 til;
  Mat3 Ril;
  til << VEC_FROM_ARRAY(extrin_trans);
  Ril << MAT_FROM_ARRAY(extrin_rot);
  p_imu_->setExtrinsic(til, Ril);
  p_imu_->setGyrCov(Vec3(gyr_cov, gyr_cov, gyr_cov));
  p_imu_->setAccCov(Vec3(acc_cov, acc_cov, acc_cov));
  p_imu_->setGyrBiasCov(Vec3(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  p_imu_->setAccBiasCov(Vec3(b_acc_cov, b_acc_cov, b_acc_cov));

  // ROS subscribe initialization
  sub_pcl_ = nh_.subscribe("/velodyne_points", 200000, &LidarOdomROS::cloudCB, this);
  sub_imu_ = nh_.subscribe("/imu/data", 200000, &LidarOdomROS::imuCB, this);
  pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
  // ros::Publisher pub_cloud2 = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_cmp", 100000);
  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/Odometry", 100000);
  run_timer_ = nh_.createTimer(ros::Duration(0.001), &LidarOdomROS::runCB, this );
}

void LidarOdomROS::cloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (msg->header.stamp.toSec() < last_timestamp_lidar_)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_buffer_.clear();
  }

  PointCloud::Ptr ptr(new PointCloud());
  p_pre_->process(msg, ptr);
  std::lock_guard<std::mutex> lock(mtx_);
  lidar_buffer_.push_back(ptr);
  time_buffer_.push_back(msg->header.stamp.toSec());
  last_timestamp_lidar_ = msg->header.stamp.toSec();
  
}

void LidarOdomROS::imuCB(const sensor_msgs::Imu::ConstPtr &msg_in)
{
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

  double timestamp = msg->header.stamp.toSec();

  std::lock_guard<std::mutex> lock(mtx_);

  if (timestamp < newest_imu_stamp_)
  {
    ROS_WARN("imu loop back, clear buffer");
    imu_buffer_.clear();
  }

  newest_imu_stamp_ = timestamp;

  imu_buffer_.push_back(msg);
}

bool LidarOdomROS::syncData(SensorData &sensor_data)
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (lidar_buffer_.empty() || imu_buffer_.empty())
  {
    return false;
  }

  // add lidar
  auto &cloud = lidar_buffer_.front();
  std::sort(cloud->points.begin(), cloud->points.end(), time_cmp);

  sensor_data.lidar = cloud;
  double stamp = time_buffer_.front();
  sensor_data.lidar_end_time = stamp + cloud->points.back().time;
  lidar_end_time_ = sensor_data.lidar_end_time;

  if (newest_imu_stamp_ < lidar_end_time_)
  {
    return false;
  }

  // push imu data, and pop from imu buffer
  double imu_time = imu_buffer_.front()->header.stamp.toSec();
  sensor_data.imu.clear();
  while ((!imu_buffer_.empty()) && (imu_time < lidar_end_time_))
  {
    imu_time = imu_buffer_.front()->header.stamp.toSec();
    if (imu_time > lidar_end_time_)
      break;
    sensor_data.imu.push_back(imu_buffer_.front());
    imu_buffer_.pop_front();
  }

  lidar_buffer_.pop_front();
  time_buffer_.pop_front();
  return true;
}

void LidarOdomROS::runCB(const ros::TimerEvent &e)
{
  SensorData sensor_data;

  if (syncData(sensor_data))
  {
    if (flg_first_scan_)
    {

      flg_first_scan_ = false;
      return;
    }
    PointCloud::Ptr cloud_deskew(new PointCloud());

    p_imu_->process(sensor_data, *kf_, cloud_deskew); // deskew lidar points. by backward propagation

    /*
    PointCloud::Ptr cloud_deskew_cmp(new PointCloud());
    for (auto p : sensor_data.lidar->points)
    {
      p.intensity = 0;
      cloud_deskew_cmp->push_back(p);
    }
    for (auto p : cloud_deskew->points)
    {
      p.intensity = 1;
      cloud_deskew_cmp->push_back(p);
    }
    */

    if (cloud_deskew->empty() || (cloud_deskew == NULL))
    {
      ROS_WARN("No point, skip this scan!\n");
      return;
    }

    // Segment the map in lidar FOV
    auto state = kf_->getState();
    mapping_->setState(state);
    // Vec3 pos_lid = state_.pos + state_.rot * state_.til; // Lidar point in global frame.
    // updateMapArea(pos_lid);

    // downsample the feature points in a scan
    PointCloud::Ptr cloud_ds(new PointCloud());
    downsampe_filter_.setInputCloud(cloud_deskew);
    downsampe_filter_.filter(*cloud_ds);

    if (mapping_->initMap(cloud_ds))
    {
      return;
    }

    int cloud_size = cloud_ds->points.size();

    //  ICP and iterated Kalman filter update
    if (cloud_size < 5)
    {
      ROS_WARN("No point, skip this scan!\n");
      return;
    }

    //  iterated state estimation
    kf_->iteratedUpdate(cloud_ds);

    //  Publish odometry
    mapping_->pubOdom(pub_odom_, *kf_, lidar_end_time_);

    //  add the feature points to map kdtree
    mapping_->updateMap(cloud_ds);

    //  Publish points
    if (scan_pub_)
    {
      mapping_->pubCloud(pub_cloud_, cloud_deskew, lidar_end_time_);
      // mapping_->pubCloud(pub_cloud2, cloud_deskew_cmp, lidar_end_time_);
    }
  }
}