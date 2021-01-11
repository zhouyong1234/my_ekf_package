#ifndef MY_EKF_DATA_FUSION_HPP_
#define MY_EKF_DATA_FUSION_HPP_


#include <math.h>
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
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "my_ekf/pf_filter_score_data.h"
#include "my_ekf/imu_theta_data.h"
#include "my_ekf/kalman_filter.hpp"

#include <algorithm>

namespace my_ekf_package
{
class DataFusion{
public:
    DataFusion();
    ~DataFusion();

    bool run();

    KalmanFilter kf_;

private:

    bool readData();

    bool hasData();

    bool validTime();

    bool validOdomData();

    bool validGNSSData();

    bool validOdomGNSSData();

    void predict();

    void update();

    void parseWheelOdomData(std::deque<nav_msgs::Odometry::ConstPtr>& deque_wheel_odom_data);

    void parseGNSSData(std::deque<geometry_msgs::PoseStamped::ConstPtr>& deque_gnss_data);

    void parsePfScoreData(std::deque<PfScoreStamped>& deque_pf_score_data);

    void parseIMUThetaData(std::deque<ImuThetaStamped>& deque_imu_theta_data);

    void spin(const ros::TimerEvent&);

    void imuDataCallback(const std_msgs::Float64& imu_theta);

    // void imuDataCallback(const sensor_msgs::Imu::ConstPtr& imu_data);

    void wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& wheel_odom);

    void icpOdomCallback(const nav_msgs::Odometry::ConstPtr& icp_odom);

    void gnssCallback(const geometry_msgs::PoseStamped::ConstPtr& gnss);

    void pfFilterScoreCallback(const std_msgs::Float64& pf_filter_score);

    void broadcastPose();

    bool is_initialized_;
    bool odom_active_;
    bool odom_initializing_;

    bool wheel_odom_used_, icp_odom_used_, imu_used_, gps_used_, pf_score_used_;

    double global_gnss_var_x_, global_gnss_var_y_;

    ros::Timer timer_;

    std::string output_frame_, base_footprint_frame_;

    ros::Publisher pose_pub_;
    ros::Subscriber wheel_odom_sub_, icp_odom_sub_, imu_data_sub_, gnss_data_sub_;
    ros::Subscriber pf_filter_score_sub_;

    ros::Time filter_stamp_, current_stamp_;
    ros::Time wheel_odom_init_stamp_;

    std::deque<nav_msgs::Odometry::ConstPtr> valid_wheel_odom_data_;
    std::deque<nav_msgs::Odometry::ConstPtr> new_wheel_odom_data_;
    std::deque<geometry_msgs::PoseStamped::ConstPtr> valid_gnss_data_;
    std::deque<geometry_msgs::PoseStamped::ConstPtr> new_gnss_data_;
    std::deque<PfScoreStamped> new_pf_filter_score_data_;
    std::deque<ImuThetaStamped> new_imu_theta_data_;

    std::deque<nav_msgs::Odometry::ConstPtr> wheel_odom_data_buff_;
    std::deque<geometry_msgs::PoseStamped::ConstPtr> gnss_data_buff_;
    std::deque<PfScoreStamped> pf_filter_score_buff_;
    std::deque<ImuThetaStamped> imu_theta_data_buff_;


    nav_msgs::Odometry::ConstPtr current_wheel_odom_data_;
    geometry_msgs::PoseStamped::ConstPtr current_gnss_data_;
    PfScoreStamped current_pf_filter_score_data_;
    ImuThetaStamped current_imu_theta_data_;

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