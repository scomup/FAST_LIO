#define PCL_NO_PRECOMPILE

#include "lidar_odom_ros.h"
#include <visualization_msgs/MarkerArray.h>


template <typename T>
void LidarOdomROS::setOdomMsg(T &out)
{
  out.pose.position.x = state_.pos(0);
  out.pose.position.y = state_.pos(1);
  out.pose.position.z = state_.pos(2);
  Eigen::Quaterniond q = Eigen::Quaterniond(state_.rot);
  out.pose.orientation.x = q.x();
  out.pose.orientation.y = q.y();
  out.pose.orientation.z = q.z();
  out.pose.orientation.w = q.w();
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
  auto h_model = [this](HData &ekfom_data,
                            State &x,
                            PointCloud::Ptr &cloud) -> bool                   
    {bool vaild = this->mapping_->H2Model(ekfom_data, x, cloud); return vaild;};

  kf_ = boost::make_shared<Esekf>(0.001, max_iteration, h_model);

  downsampe_filter_.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

  Vec3 til;
  Mat3 Ril;
  til << VEC_FROM_ARRAY(extrin_trans);
  Ril << MAT_FROM_ARRAY(extrin_rot);
  kf_->setExtrinsic(til, Ril);
  kf_->setGyrCov(Vec3(gyr_cov, gyr_cov, gyr_cov));
  kf_->setAccCov(Vec3(acc_cov, acc_cov, acc_cov));
  kf_->setGyrBiasCov(Vec3(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  kf_->setAccBiasCov(Vec3(b_acc_cov, b_acc_cov, b_acc_cov));

  // ROS subscribe initialization
  sub_pcl_ = nh_.subscribe("/velodyne_points", 200000, &LidarOdomROS::cloudCB, this);
  sub_imu_ = nh_.subscribe("/imu/data", 200000, &LidarOdomROS::imuCB, this);
  pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
  // ros::Publisher pub_cloud2 = nh_.advertise<sensor_msgs::PointCloud2>("/cloud_cmp", 100000);
  pub_odom_ = nh_.advertise<nav_msgs::Odometry>("/Odometry", 100000);
  pub_maker_ = nh_.advertise<visualization_msgs::MarkerArray>("/maker", 100000);
  run_timer_ = nh_.createTimer(ros::Duration(0.001), &LidarOdomROS::runCB, this );
}

void LidarOdomROS::cloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (msg->header.stamp.toSec() < last_timestamp_lidar_)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_q_.clear();
  }

  PointCloud::Ptr ptr(new PointCloud());
  p_pre_->process(msg, ptr);
  std::lock_guard<std::mutex> lock(mtx_);
  lidar_q_.push_back({ptr,msg->header.stamp.toSec()});
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
    imu_q_.clear();
  }

  newest_imu_stamp_ = timestamp;

  imu_q_.push_back(msg);
}

bool LidarOdomROS::getSensorData(SensorData &sensor_data)
{
  std::lock_guard<std::mutex> lock(mtx_);

  if (lidar_q_.empty() || imu_q_.empty())
  {
    return false;
  }

  // add lidar
  auto &cloud_info = lidar_q_.front();
  sensor_data.lidar = cloud_info.cloud;
  sensor_data.lidar_stamp = cloud_info.stamp + cloud_info.cloud->points.back().time;
  lidar_end_time_ = sensor_data.lidar_stamp;

  if (newest_imu_stamp_ < lidar_end_time_)
  {
    return false;
  }

  // push imu data, and pop from imu buffer
  double imu_time = imu_q_.front()->header.stamp.toSec();
  sensor_data.imu.clear();
  while (!imu_q_.empty())
  {
    imu_time = imu_q_.front()->header.stamp.toSec();
    if (imu_time > lidar_end_time_)
      break;
    sensor_data.imu.push_back(imu_q_.front());
    imu_q_.pop_front();
  }

  lidar_q_.pop_front();
  return true;
}

void LidarOdomROS::runCB(const ros::TimerEvent &e)
{
  SensorData sensor_data;

  if (getSensorData(sensor_data))
  {
    if (flg_first_scan_)
    {

      flg_first_scan_ = false;
      return;
    }
    PointCloud::Ptr cloud_deskew(new PointCloud());

    // imu forward propagation, and deskew lidar points. 
    kf_->propagation(sensor_data, cloud_deskew); 

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
    state_ = kf_->getState();
    // Vec3 pose_lidar = state_.pos + state_.rot * state_.til; // Lidar point in global frame.
    // updateMapArea(pose_lidar);

    // downsample the feature points in a scan
    PointCloud::Ptr cloud_ds(new PointCloud());
    downsampe_filter_.setInputCloud(cloud_deskew);
    downsampe_filter_.filter(*cloud_ds);

    if (mapping_->initMap(cloud_ds, state_))
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
    auto t0 = omp_get_wtime();
    kf_->iteratedUpdate(cloud_ds);
    auto t1 = omp_get_wtime();
    printf("iteratedUpdate: %f ms\n", (t1-t0)*1000.);

    //  add the feature points to map kdtree
    mapping_->updateMap(cloud_ds, state_);
    //auto maker = mapping_->makeMarkerArray();
    //pub_maker_.publish(maker);

    //  Publish odometry
    pubOdom(pub_odom_, *kf_, lidar_end_time_);

    //  Publish points
    if (scan_pub_)
    {
      pubCloud(pub_cloud_, cloud_deskew, lidar_end_time_);
    }
  }
}

void LidarOdomROS::pubOdom(const ros::Publisher &pub, const Esekf &kf, double time)
{
  nav_msgs::Odometry odom;

  odom.header.frame_id = "map";
  odom.child_frame_id = "imu";
  odom.header.stamp = ros::Time().fromSec(time);
  setOdomMsg(odom.pose);
  pub.publish(odom);
  auto P = kf.getP();
  for (int i = 0; i < 6; i++)
  {
    int k = i < 3 ? i + 3 : i - 3;
    odom.pose.covariance[i * 6 + 0] = P(k, 3);
    odom.pose.covariance[i * 6 + 1] = P(k, 4);
    odom.pose.covariance[i * 6 + 2] = P(k, 5);
    odom.pose.covariance[i * 6 + 3] = P(k, 0);
    odom.pose.covariance[i * 6 + 4] = P(k, 1);
    odom.pose.covariance[i * 6 + 5] = P(k, 2);
  }

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(tf::Vector3(odom.pose.pose.position.x,
                                  odom.pose.pose.position.y,
                                  odom.pose.pose.position.z));
  q.setW(odom.pose.pose.orientation.w);
  q.setX(odom.pose.pose.orientation.x);
  q.setY(odom.pose.pose.orientation.y);
  q.setZ(odom.pose.pose.orientation.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, odom.header.stamp, "map", "imu"));
}

void LidarOdomROS::pubCloud(const ros::Publisher &pub_cloud, PointCloud::Ptr &cloud, double time)
{
  int size = cloud->points.size();
  PointCloud::Ptr cloud_world(new PointCloud());
  auto state = kf_->getState();

  cloudL2W(cloud, cloud_world, state);

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_world, cloud_msg);
  cloud_msg.header.stamp = ros::Time().fromSec(time);
  cloud_msg.header.frame_id = "map";
  pub_cloud.publish(cloud_msg);
}
