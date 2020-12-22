#include <iostream>
#include "my_ekf/data_fusion.hpp"

namespace my_ekf_package
{
DataFusion::DataFusion()
{
    is_initialized_ = false;
    odom_active_ = false;
    odom_initializing_ = false;
    previous_odom_mat_ = Eigen::Matrix4d::Identity();

    // 观测值方差
    wheel_odom_var_ << 0.2, 0.2, 0.2;  
    icp_odom_var_ <<  0.2, 0.2, 0.2;
    gnss_var_ << 0.2, 0.2, 0.2;
    imu_var_ << 0.1, 0.1, 0.1, 0.1;

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    nh.param("output_frame", output_frame_, std::string("odom_combined"));
    nh.param("base_footprint_frame", base_footprint_frame_, std::string("base_footprint"));
    nh_private.param("wheel_odom_used", wheel_odom_used_, true);
    nh_private.param("icp_odom_used", icp_odom_used_, true);
    nh_private.param("imu_used", imu_used_, true);
    nh_private.param("gps_used", gps_used_, true);

    // ROS_INFO_STREAM("output_frame: " << output_frame_);

    std::cout << "imu_used_: " << imu_used_ << std::endl;

    timer_ = nh.createTimer(ros::Duration(0.1), &DataFusion::spin, this);

    // ROS_INFO_STREAM("1111111111111111111");

    filter_stamp_ = ros::Time::now();

    pose_pub_ = nh.advertise<nav_msgs::Odometry>("odom_combined", 10);

    imu_data_sub_ = nh.subscribe("imu_data", 10, &DataFusion::imuDataCallback, this);

    wheel_odom_sub_ = nh.subscribe("wheel_odom", 10, &DataFusion::wheelOdomCallback, this);

    icp_odom_sub_ = nh.subscribe("icp_odom", 10, &DataFusion::icpOdomCallback, this);

    gnss_data_sub_ = nh.subscribe("gnss", 10, &DataFusion::gnssCallback, this);

    // ROS_INFO_STREAM("2222222222222222222");

}

DataFusion::~DataFusion() {}


/*******************************************************
IMU回调函数
********************************************************/
void DataFusion::imuDataCallback(const sensor_msgs::Imu::ConstPtr& imu_data)
{

    current_stamp_ = imu_data->header.stamp;

    if(is_initialized_ && imu_used_)
    {
        double current_time_imu = imu_data->header.stamp.sec + imu_data->header.stamp.nsec * 1e-9;
        gyro_ = Eigen::Vector3d(imu_data->angular_velocity.x, imu_data->angular_velocity.y, imu_data->angular_velocity.z);
        acc_ = Eigen::Vector3d(imu_data->linear_acceleration.x, imu_data->linear_acceleration.y, imu_data->linear_acceleration.z);

        orientation_ = Eigen::Vector4d(imu_data->orientation.x, imu_data->orientation.y, imu_data->orientation.z, imu_data->orientation.w);
        // kf_.Prediction(current_time_imu, gyro, acc);
        // 使用IMU做更新
        kf_.KFUpdate(orientation_, imu_var_);
    }
}

/*******************************************************
轮式里程计回调函数
********************************************************/
void DataFusion::wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& wheel_odom)
{
    // ROS_INFO_STREAM("WheelOdom callback at time: " << ros::Time::now().toSec());

    

    current_stamp_ = wheel_odom->header.stamp;

    if(is_initialized_ && wheel_odom_used_)
    {
        double current_time_wheel_odom = wheel_odom->header.stamp.sec + wheel_odom->header.stamp.nsec * 1e-9;

        Eigen::Affine3d aff;
        tf2::fromMsg(wheel_odom->pose.pose, aff);
        Eigen::Matrix4d odom_mat = aff.matrix();

        // std::cout << "odom_mat: " << std::endl << odom_mat << std::endl;

        if(previous_odom_mat_ == Eigen::Matrix4d::Identity())
        {
            // 初始帧位姿
            current_pose_odom_ = current_pose_;
            // 初始帧位姿矩阵
            previous_odom_mat_ = odom_mat;
            return;
        }

        Eigen::Affine3d current_aff;
        // 计算相对运动，由前两帧的位姿增量计算下一帧的位姿
        tf2::fromMsg(current_pose_odom_.pose, current_aff);
        // 初始帧的位姿
        Eigen::Matrix4d current_trans = current_aff.matrix();
        // 下一帧位姿 = 位姿增量 x 当前帧位姿

        // std::cout << "位姿增量: " << std::endl << current_trans * previous_odom_mat_.inverse() << std::endl;

        current_trans = current_trans * previous_odom_mat_.inverse() * odom_mat;

        Eigen::Vector3d pose_msg;
        // pose_msg(0) = current_trans(0, 3);
        // pose_msg(1) = current_trans(1, 3);
        // pose_msg(2) = current_trans(2, 3);
        pose_msg(0) = wheel_odom->pose.pose.position.x;
        pose_msg(1) = wheel_odom->pose.pose.position.y;
        pose_msg(2) = wheel_odom->pose.pose.position.z;

        Eigen::Vector4d orientation_msg;
        orientation_msg(0) = wheel_odom->pose.pose.orientation.x;
        orientation_msg(1) = wheel_odom->pose.pose.orientation.y;
        orientation_msg(2) = wheel_odom->pose.pose.orientation.z;
        orientation_msg(3) = wheel_odom->pose.pose.orientation.w;

        // std::cout << "下一帧位姿: " << current_trans(0, 3) << ", " << current_trans(1, 3) << ", " << current_trans(2, 3) << std::endl;
        // 使用轮式里程计做预测
        kf_.Prediction(current_time_wheel_odom, pose_msg, orientation_msg);

        // kf_.KFUpdate(pose_msg, wheel_odom_var_);

        current_pose_odom_ = current_pose_;
        previous_odom_mat_ = odom_mat;
    }

    if(!odom_active_)
    {
        if(!odom_initializing_)
        {
            odom_initializing_ = true;
            wheel_odom_init_stamp_ = current_stamp_;
            // ROS_INFO_STREAM("Initializing odom");
        }

        // ROS_INFO_STREAM("filter_stamp_: " << filter_stamp_);
        // ROS_INFO_STREAM("wheel_odom_init_stamp_: " << wheel_odom_init_stamp_);

        if(filter_stamp_ >= wheel_odom_init_stamp_)
        {

            current_pose_.header.stamp = wheel_odom->header.stamp;
            current_pose_.pose.position.x = wheel_odom->pose.pose.position.x;
            current_pose_.pose.position.y = wheel_odom->pose.pose.position.y;
            current_pose_.pose.position.z = wheel_odom->pose.pose.position.z;
            current_pose_.pose.orientation.z = wheel_odom->pose.pose.orientation.x;
            current_pose_.pose.orientation.y = wheel_odom->pose.pose.orientation.y;
            current_pose_.pose.orientation.z = wheel_odom->pose.pose.orientation.z;
            current_pose_.pose.orientation.w = wheel_odom->pose.pose.orientation.w;

            wheel_odom_init_ = Eigen::VectorXd::Zero(10);
            wheel_odom_init_(0) = wheel_odom->pose.pose.position.x;
            wheel_odom_init_(1) = wheel_odom->pose.pose.position.y;
            wheel_odom_init_(2) = wheel_odom->pose.pose.position.z;
            wheel_odom_init_(3) = wheel_odom->twist.twist.linear.x;
            wheel_odom_init_(4) = wheel_odom->twist.twist.linear.y;
            wheel_odom_init_(5) = wheel_odom->twist.twist.linear.z;
            wheel_odom_init_(6) = wheel_odom->pose.pose.orientation.x;
            wheel_odom_init_(7) = wheel_odom->pose.pose.orientation.y;
            wheel_odom_init_(8) = wheel_odom->pose.pose.orientation.z;
            wheel_odom_init_(9) = wheel_odom->pose.pose.orientation.w;
            odom_active_ = true;
            odom_initializing_ = false;
            // ROS_INFO_STREAM("Odom activated");
        }
    }

}

/*******************************************************
激光里程计回调函数
********************************************************/
void DataFusion::icpOdomCallback(const nav_msgs::Odometry::ConstPtr& icp_odom)
{
    // ROS_INFO_STREAM("IcpOdom callback at time: " << ros::Time::now().toSec());

    current_stamp_ = icp_odom->header.stamp;

    if(is_initialized_ && icp_odom_used_)
    {
        // 当前里程计位姿矩阵
        Eigen::Affine3d aff;
        tf2::fromMsg(icp_odom->pose.pose, aff);
        Eigen::Matrix4d odom_mat = aff.matrix();
        if(previous_odom_mat_ == Eigen::Matrix4d::Identity())
        {
            current_pose_odom_ = current_pose_;
            previous_odom_mat_ = odom_mat;
            return;
        }

        Eigen::Affine3d current_aff;
        // 计算相对运动，由前两帧的位姿增量计算下一帧的位姿
        tf2::fromMsg(current_pose_odom_.pose, current_aff);
        // 初始帧的位姿
        Eigen::Matrix4d current_trans = current_aff.matrix();
        // 下一帧位姿 = 位姿增量 x 当前帧位姿
        current_trans = current_trans * previous_odom_mat_.inverse() * odom_mat;

        Eigen::Vector3d pose_msg;
        pose_msg(0) = current_trans(0, 3);
        pose_msg(1) = current_trans(1, 3);
        pose_msg(2) = current_trans(2, 3);

        // 使用激光里程计做更新
        kf_.KFUpdate(pose_msg, icp_odom_var_);

        current_pose_odom_ = current_pose_;
        previous_odom_mat_ = odom_mat;
    }

}

/*******************************************************
GNSS回调函数
********************************************************/
void DataFusion::gnssCallback(const geometry_msgs::PoseStamped::ConstPtr& gnss)
{
    current_stamp_ = gnss->header.stamp;

    if(is_initialized_ && gps_used_)
    {
        Eigen::Vector3d pose_msg;
        pose_msg(0) = gnss->pose.position.x;
        pose_msg(1) = gnss->pose.position.y;
        pose_msg(2) = gnss->pose.position.z;

        // 使用gnss做更新
        kf_.KFUpdate(pose_msg, gnss_var_);
    }
}

/*******************************************************
发布融合后的里程计信息和tf变换
********************************************************/
void DataFusion::broadcastPose()
{
    // 发布融合后的位姿
    nav_msgs::Odometry combined_odom_msg;
    combined_odom_msg.header.stamp = current_stamp_;
    combined_odom_msg.header.frame_id= "odom";
    combined_odom_msg.child_frame_id = "base_footprint";
    combined_odom_msg.pose.pose.position.x = kf_.GetX()(0);
    combined_odom_msg.pose.pose.position.y = kf_.GetX()(1);
    combined_odom_msg.pose.pose.position.z = kf_.GetX()(2);
    combined_odom_msg.twist.twist.linear.x = kf_.GetX()(3);
    combined_odom_msg.twist.twist.linear.y = kf_.GetX()(4);
    combined_odom_msg.twist.twist.linear.z = kf_.GetX()(5);
    combined_odom_msg.pose.pose.orientation.x = kf_.GetX()(6);
    combined_odom_msg.pose.pose.orientation.y = kf_.GetX()(7);
    combined_odom_msg.pose.pose.orientation.z = kf_.GetX()(8);
    combined_odom_msg.pose.pose.orientation.w = kf_.GetX()(9);
    pose_pub_.publish(combined_odom_msg);

    std::cout << "combined_odom_msg: " << combined_odom_msg.pose.pose.position.x << ", " << 
                                          combined_odom_msg.pose.pose.position.y << ", " <<
                                          combined_odom_msg.pose.pose.position.z << ", " <<
                                          combined_odom_msg.twist.twist.linear.x << ", " <<
                                          combined_odom_msg.twist.twist.linear.y << ", " <<
                                          combined_odom_msg.twist.twist.linear.z << ", " <<
                                          combined_odom_msg.pose.pose.orientation.x << ", " <<
                                          combined_odom_msg.pose.pose.orientation.y << ", " <<
                                          combined_odom_msg.pose.pose.orientation.z << ", " <<
                                          combined_odom_msg.pose.pose.orientation.w << std::endl;

    // 发布tf变换
    tfb_.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                tf::Quaternion(
                            combined_odom_msg.pose.pose.orientation.x,
                            combined_odom_msg.pose.pose.orientation.y,
                            combined_odom_msg.pose.pose.orientation.z,
                            combined_odom_msg.pose.pose.orientation.w).normalized(), 
                tf::Vector3(
                    combined_odom_msg.pose.pose.position.x,
                    combined_odom_msg.pose.pose.position.y,
                    combined_odom_msg.pose.pose.position.z)), 
            ros::Time::now(), "odom", "base"));
}

void DataFusion::spin(const ros::TimerEvent& e)
{
    // ROS_INFO_STREAM("Spin function at time: " << ros::Time::now().toSec());

    filter_stamp_ = ros::Time::now();


    if(is_initialized_)
    {
        broadcastPose();
    }

    // 第一帧数据用于初始化 Kalman 滤波器
    if (odom_active_ && !is_initialized_) {

        // 避免运算时，0作为被除数
        if (fabs(wheel_odom_init_(0)) < 0.001) {
            wheel_odom_init_(0) = 0.001;
        }
        if (fabs(wheel_odom_init_(1)) < 0.001) {
            wheel_odom_init_(1) = 0.001;
        }
        if(fabs(wheel_odom_init_(2)) < 0.001) {
            wheel_odom_init_(2) = 0.001;
        }

        // 使用里程计位姿初始化Kalman滤波器
        kf_.Initialization(wheel_odom_init_);

        ROS_INFO_STREAM("Initialized......");
        is_initialized_ = true;
        return;
    }

}
}


