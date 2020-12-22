#ifndef MY_EKF_DATA_FUSION_HPP_
#define MY_EKF_DATA_FUSION_HPP_


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include "Eigen/Eigen"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
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

private:

    void spin(const ros::TimerEvent&);

    void imuDataCallback(const sensor_msgs::Imu::ConstPtr& imu_data);

    void wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& wheel_odom);

    void icpOdomCallback(const nav_msgs::Odometry::ConstPtr& icp_odom);

    void gnssCallback(const geometry_msgs::PoseStamped::ConstPtr& gnss);

    void broadcastPose();

    bool is_initialized_;
    bool odom_active_;
    bool odom_initializing_;

    bool wheel_odom_used_, icp_odom_used_, imu_used_, gps_used_;

    ros::Timer timer_;

    std::string output_frame_, base_footprint_frame_;

    ros::Publisher pose_pub_;
    ros::Subscriber wheel_odom_sub_, icp_odom_sub_, imu_data_sub_, gnss_data_sub_;

    ros::Time filter_stamp_, current_stamp_;
    ros::Time wheel_odom_init_stamp_;

    Eigen::Vector3d gyro_, acc_;
    Eigen::Vector4d orientation_;
    Eigen::MatrixXd previous_odom_mat_;
    Eigen::VectorXd wheel_odom_init_;
    Eigen::Vector3d wheel_odom_var_, icp_odom_var_, gnss_var_;
    Eigen::Vector4d imu_var_;

    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped current_pose_odom_;

    tf::TransformBroadcaster tfb_;

};
}

#endif