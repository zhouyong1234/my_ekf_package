#ifndef MY_EKF_IMU_THETA_DATA_HPP_
#define MY_EKF_IMU_THETA_DATA_HPP_


#include "ros/ros.h"
#include "std_msgs/Float64.h"

namespace my_ekf_package
{
class ImuThetaStamped {
public:
  ros::Time timestamp_;
  std_msgs::Float64 imu_theta_;

};
}


#endif