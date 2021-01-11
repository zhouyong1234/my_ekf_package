#ifndef MY_EKF_PF_FILTER_SCORE_DATA_HPP_
#define MY_EKF_PF_FILTER_SCORE_DATA_HPP_

#include "std_msgs/Float64.h"
#include "ros/ros.h"

namespace my_ekf_package
{
class PfScoreStamped {
public:
  ros::Time timestamp_;
  std_msgs::Float64 pf_score_;

};
}

#endif
