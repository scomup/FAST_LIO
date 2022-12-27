#include "lidar_odom_ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_odom");
  LidarOdomROS node;
  ros::spin();
}
