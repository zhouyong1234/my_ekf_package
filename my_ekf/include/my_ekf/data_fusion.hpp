#ifndef MY_EKF_DATA_FUSION_HPP_
#define MY_EKF_DATA_FUSION_HPP_


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "Eigen/Eigen"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "my_ekf/measurement.h"
#include "my_ekf/kalman_filter.hpp"

#include <algorithm>

namespace my_ekf_package
{
class DataFusion{
public:
    DataFusion();
    ~DataFusion();

    KalmanFilter kf_;

    void Process(Measurement measurement);

private:

    void spin(const ros::TimerEvent&);

    void wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& wheel_odom);

    void icpOdomCallback(const nav_msgs::Odometry::ConstPtr& icp_odom);

    bool is_initialized_;
    bool odom_active_;
    bool odom_initializing_;
    long last_timestamp_;
    Eigen::MatrixXd R_lidar_;
    Eigen::MatrixXd R_radar_;
    Eigen::MatrixXd H_lidar_;

    ros::Timer timer_;

    std::string output_frame_, base_footprint_frame_;

    tf::Transform wheel_odom_meas_, icp_odom_meas_;

    ros::Publisher pose_pub_;
    ros::Subscriber wheel_odom_sub_, icp_odom_sub_;

    ros::Time wheel_odom_stamp_, icp_odom_stamp_, filter_stamp_;
    ros::Time wheel_odom_init_stamp_;

    Eigen::MatrixXd wheel_odom_covariance_;
    Eigen::MatrixXd icp_odom_covariance_;

    Eigen::Vector4d wheel_odom_mean;
    Eigen::Vector2d icp_odom_mean;

};
}

#endif