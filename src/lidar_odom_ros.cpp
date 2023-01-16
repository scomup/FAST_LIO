#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "lidar_odom_ros.h"
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>



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
  int point_filter_num;

  nh_.param<bool>("publish/scan_publish_en", scan_pub_, true);
  nh_.param<int>("max_iteration", max_iteration, 4);
  nh_.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
  nh_.param<double>("filter_size_map", filter_size_map, 0.5);
  nh_.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
  nh_.param<double>("mapping/acc_cov", acc_cov, 0.1);
  nh_.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
  nh_.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
  nh_.param<double>("blind", blind, 2);
  nh_.param<int>("point_filter_num", point_filter_num, 2);
  nh_.param<bool>("mapping/extrinsic_est_en", extrinsic_est, false);
  nh_.param<std::vector<double>>("mapping/extrinsic_T", extrin_trans, std::vector<double>());
  nh_.param<std::vector<double>>("mapping/extrinsic_R", extrin_rot, std::vector<double>());


  scan_process_ = boost::make_shared<ScanProcess>(blind, point_filter_num, filter_size_surf_min);;
  mapping_ = boost::make_shared<Mapping>(extrinsic_est, filter_size_map);
  auto h_model = [this](HData &ekfom_data,
                            State &x,
                            CloudPtr &cloud) -> bool                   
    {bool vaild = this->mapping_->point2PlaneModel(ekfom_data, x, cloud); return vaild;};

  kf_ = boost::make_shared<Esekf>(0.001, max_iteration, h_model);

  Vec3 til;
  Mat3 Ril;
  til = Eigen::Map<Vec3>(extrin_trans.data());
  Ril = Eigen::Map<Mat3>(extrin_rot.data());
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
  run_timer_ = nh_.createTimer(ros::Duration(0.001), &LidarOdomROS::runCB, this );
}

void LidarOdomROS::cloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (msg->header.stamp.toSec() < last_timestamp_lidar_)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    scan_q_.clear();
  }

  CloudPtr ptr(new Cloud());
  scan_process_->process(msg, ptr);
  std::lock_guard<std::mutex> lock(mtx_);
  scan_q_.push_back({ptr,msg->header.stamp.toSec()});
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

  if (scan_q_.empty() || imu_q_.empty())
  {
    return false;
  }

  // add lidar
  auto &cloud_info = scan_q_.front();
  sensor_data.cloud = cloud_info.cloud;
  sensor_data.stamp = cloud_info.stamp + cloud_info.cloud->points.back().time;
  lidar_end_time_ = sensor_data.stamp;

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

  scan_q_.pop_front();
  return true;
}

void LidarOdomROS::runCB(const ros::TimerEvent &e)
{
  SensorData sensor_data;

  if (getSensorData(sensor_data))
  {
    // imu forward propagation, and deskew lidar points. 
    kf_->predict(sensor_data); 

    CloudPtr cloud_deskew = kf_->undistortCloud(sensor_data); 

    /*
    CloudPtr cloud_deskew_cmp(new Cloud());
    for (auto p : sensor_data.cloud->points)
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

    // updateMapArea(pose);

    // downsample the feature points in a scan

    auto cloud_ds = scan_process_->downsample(cloud_deskew);

    if (cloud_ds->points.size() < 5)
    {
      ROS_WARN("No point, skip this scan!\n");
      return;
    }

    if (mapping_->initMap(cloud_ds, kf_->getState()))
    {
      return;
    }

    //  iterated state estimation
    kf_->iteratedUpdate(cloud_ds);

    //  add the feature points to map kdtree
    mapping_->updateMap(cloud_ds, kf_->getState());

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

void LidarOdomROS::pubCloud(const ros::Publisher &pub_cloud, CloudPtr &cloud, double time)
{
  CloudPtr cloud_world(new Cloud());
  auto state = kf_->getState();
  pcl::transformPointCloud(*cloud, *cloud_world, state.getTwl());

  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(*cloud_world, cloud_msg);
  cloud_msg.header.stamp = ros::Time().fromSec(time);
  cloud_msg.header.frame_id = "map";
  pub_cloud.publish(cloud_msg);
}
