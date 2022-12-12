#define PCL_NO_PRECOMPILE

#include "mapping.h"
#include "backpropagation.h"
#include "preprocess.h"


#define INIT_TIME (0.1)
#define LASER_POINT_COV (0.001)

bool flg_exit = false;

std::mutex mtx_;

double last_timestamp_lidar_ = 0, last_timestamp_imu_ = -1.0;
double lidar_end_time_ = 0;

std::deque<double> time_buffer_;
std::deque<PointCloud::Ptr> lidar_buffer_;
std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;

double lidar_mean_scantime = 0.0;
int scan_num = 0;
bool lidar_pushed_;
std::shared_ptr<Preprocess> p_pre_g;

void SigHandle(int sig)
{
  flg_exit = true;
  ROS_WARN("catch sig %d", sig);
}

void cloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  mtx_.lock();
  if (msg->header.stamp.toSec() < last_timestamp_lidar_)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_buffer_.clear();
  }

  PointCloud::Ptr ptr(new PointCloud());
  p_pre_g->process(msg, ptr);
  lidar_buffer_.push_back(ptr);
  time_buffer_.push_back(msg->header.stamp.toSec());
  last_timestamp_lidar_ = msg->header.stamp.toSec();
  mtx_.unlock();
}

void imuCB(const sensor_msgs::Imu::ConstPtr &msg_in)
{
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

  double timestamp = msg->header.stamp.toSec();

  mtx_.lock();

  if (timestamp < last_timestamp_imu_)
  {
    ROS_WARN("imu loop back, clear buffer");
    imu_buffer_.clear();
  }

  last_timestamp_imu_ = timestamp;

  imu_buffer_.push_back(msg);
  mtx_.unlock();
}

bool syncData(SensorData &sensor_data)
{
  if (lidar_buffer_.empty() || imu_buffer_.empty())
  {
    return false;
  }

  /*** push a lidar scan ***/
  if (!lidar_pushed_)
  {
    sensor_data.lidar = lidar_buffer_.front();
    sensor_data.lidar_beg_time = time_buffer_.front();
    if (sensor_data.lidar->points.size() <= 1) // time too little
    {
      lidar_end_time_ = sensor_data.lidar_beg_time + lidar_mean_scantime;
      ROS_WARN("Too few input point cloud!\n");
    }
    else if (sensor_data.lidar->points.back().time / double(1000) < 0.5 * lidar_mean_scantime)
    {
      lidar_end_time_ = sensor_data.lidar_beg_time + lidar_mean_scantime;
    }
    else
    {
      scan_num++;
      lidar_end_time_ = sensor_data.lidar_beg_time + sensor_data.lidar->points.back().time / double(1000);
      lidar_mean_scantime += (sensor_data.lidar->points.back().time / double(1000) - lidar_mean_scantime) / scan_num;
    }

    sensor_data.lidar_end_time = lidar_end_time_;

    lidar_pushed_ = true;
  }

  if (last_timestamp_imu_ < lidar_end_time_)
  {
    return false;
  }

  /*** push imu data, and pop from imu buffer ***/
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
  lidar_pushed_ = false;
  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;

  bool scan_pub_en = false;
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

  nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
  nh.param<int>("max_iteration", max_iteration, 4);
  nh.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
  nh.param<double>("filter_size_map", filter_size_map, 0.5);
  nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
  nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
  nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
  nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
  nh.param<double>("preprocess/blind", blind, 0.01);
  nh.param<int>("preprocess/timestamp_unit", time_unit, US);
  nh.param<int>("point_filter_num", point_filter_num, 2);
  nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est, false);
  nh.param<std::vector<double>>("mapping/extrinsic_T", extrin_trans, std::vector<double>());
  nh.param<std::vector<double>>("mapping/extrinsic_R", extrin_rot, std::vector<double>());

  std::shared_ptr<Preprocess> p_pre(new Preprocess(blind, time_unit, point_filter_num));

  p_pre_g = p_pre;

  Mapping* mapping = new Mapping(extrinsic_est, filter_size_map);

  auto h_model = [mapping](ESEKF::HData &ekfom_data,
                           ESEKF::State &x,
                           PointCloud::Ptr &cloud){mapping->hModel(ekfom_data, x, cloud); };

  ESEKF::Esekf kf(LASER_POINT_COV, max_iteration, h_model);

  std::shared_ptr<BacKPropagationIMU> p_imu(new BacKPropagationIMU());

  pcl::VoxelGrid<PointType> downsampe_filter;
  downsampe_filter.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);

  Vec3 til;
  Mat3 Ril;
  til << VEC_FROM_ARRAY(extrin_trans);
  Ril << MAT_FROM_ARRAY(extrin_rot);
  p_imu->setExtrinsic(til, Ril);
  p_imu->setGyrCov(Vec3(gyr_cov, gyr_cov, gyr_cov));
  p_imu->setAccCov(Vec3(acc_cov, acc_cov, acc_cov));
  p_imu->setGyrBiasCov(Vec3(b_gyr_cov, b_gyr_cov, b_gyr_cov));
  p_imu->setAccBiasCov(Vec3(b_acc_cov, b_acc_cov, b_acc_cov));

  // ROS subscribe initialization
  ros::Subscriber sub_pcl = nh.subscribe("/velodyne_points", 200000, cloudCB);
  ros::Subscriber sub_imu = nh.subscribe("/imu/data", 200000, imuCB);
  ros::Publisher pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);
  ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("/Odometry", 100000);
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


      p_imu->process(sensor_data, kf, cloud_deskew); // deskew lidar points. by backward propagation


      if (cloud_deskew->empty() || (cloud_deskew == NULL))
      {
        ROS_WARN("No point, skip this scan!\n");
        continue;
      }

      // Segment the map in lidar FOV 
      auto state = kf.getState();
      mapping->setState(state);
      //Vec3 pos_lid = state_.pos + state_.rot * state_.til; // Lidar point in global frame.
      //updateMapArea(pos_lid);

      // downsample the feature points in a scan 
      downsampe_filter.setInputCloud(cloud_deskew);
      downsampe_filter.filter(*cloud_ds);

      if(mapping->initMap(cloud_ds))
      {
        continue;
      }

      int cloud_size = cloud_ds->points.size();

      //  ICP and iterated Kalman filter update 
      if (cloud_size < 5)
      {
        ROS_WARN("No point, skip this scan!\n");
        continue;
      }

      //  iterated state estimation
      kf.iteratedUpdate(cloud_ds);

      //  Publish odometry 
      mapping->pubOdom(pub_odom, kf, lidar_end_time_);

      //  add the feature points to map kdtree 
      mapping->updateMap(cloud_ds);

      //  Publish points 
      if (scan_pub_en)
        mapping->pubCloud(pub_cloud, cloud_deskew, lidar_end_time_);
    }

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
