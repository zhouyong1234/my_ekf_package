#include <iostream>
#include "my_ekf/data_fusion.hpp"

namespace my_ekf_package
{
DataFusion::DataFusion()
{
    is_initialized_ = false;
    odom_active_ = false;
    odom_initializing_ = false;
    last_timestamp_ = 0.0;

    // 初始化激光雷达的测量矩阵 H_lidar_
    // Set Lidar's measurement matrix H_lidar_
    H_lidar_ = Eigen::MatrixXd(2, 4);
    H_lidar_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // 设置传感器的测量噪声矩阵，一般由传感器厂商提供，如不提供，也可通过有经验的工程师调试得到
    // Set R. R is provided by Sensor supplier, in sensor datasheet
    // set measurement covariance matrix
    R_lidar_ = Eigen::MatrixXd(2, 2);
    R_lidar_ << 0.0225, 0,
                0, 0.0225;

    // Measurement covariance matrix - radar
    R_radar_ = Eigen::MatrixXd(3, 3);
    R_radar_ << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    double freq;
    nh.param("freq", freq, 30.0);

    nh.param("output_frame", output_frame_, std::string("odom_combined"));
    nh.param("base_footprint_frame", base_footprint_frame_, std::string("base_footprint"));

    ROS_INFO_STREAM("output_frame: " << output_frame_);

    // timer_ = nh_private.createTimer(r
    timer_ = nh.createTimer(ros::Duration(0.1), &DataFusion::spin, this);

    // ROS_INFO_STREAM("1111111111111111111");

    filter_stamp_ = ros::Time::now();

    pose_pub_ = nh.advertise<nav_msgs::Odometry>("odom_combined", 10);

    wheel_odom_sub_ = nh.subscribe("wheel_odom", 10, &DataFusion::wheelOdomCallback, this);

    icp_odom_sub_ = nh.subscribe("icp_odom", 10, &DataFusion::icpOdomCallback, this);

    // ROS_INFO_STREAM("2222222222222222222");

}

DataFusion::~DataFusion()
{

}

void DataFusion::wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& wheel_odom)
{
    // ROS_INFO_STREAM("WheelOdom callback at time: " << ros::Time::now().toSec());

    wheel_odom_stamp_ = wheel_odom->header.stamp;

    // ROS_INFO_STREAM("wheel_odom_stamp_: " << wheel_odom_stamp_ );

    tf::Quaternion q;
    tf::quaternionMsgToTF(wheel_odom->pose.pose.orientation, q);

    wheel_odom_mean = Eigen::Vector4d(wheel_odom->pose.pose.position.x, wheel_odom->pose.pose.position.y, wheel_odom->twist.twist.linear.x, wheel_odom->twist.twist.linear.y);

    // std::cout << "wheel_odom_mean: " << std::endl << wheel_odom_mean << std::endl;

    wheel_odom_meas_ = tf::Transform(q, tf::Vector3(wheel_odom->pose.pose.position.x, wheel_odom->pose.pose.position.y, 0));

    // std::cout << "position.x: " << wheel_odom->pose.pose.position.x << std::endl;

    wheel_odom_covariance_ = Eigen::MatrixXd(6, 6);
    for(unsigned int i = 0; i < 6; i++)
        for(unsigned int j = 0; j < 6; j++)
            wheel_odom_covariance_(i, j) = wheel_odom->pose.covariance[6*i+j];

    // std::cout << "wheel_odom_covariance_: " << std::endl << wheel_odom_covariance_ << std::endl;

    kf_.AddMeasurement(tf::StampedTransform(wheel_odom_meas_.inverse(), wheel_odom_stamp_, base_footprint_frame_, "wheel_odom"), wheel_odom_covariance_);

    if(!odom_active_)
    {
        if(!odom_initializing_)
        {
            odom_initializing_ = true;
            wheel_odom_init_stamp_ = wheel_odom_stamp_;
            ROS_INFO_STREAM("Initializing odom");
        }

        // ROS_INFO_STREAM("filter_stamp_: " << filter_stamp_);
        // ROS_INFO_STREAM("wheel_odom_init_stamp_: " << wheel_odom_init_stamp_);

        if(filter_stamp_ >= wheel_odom_init_stamp_)
        {
            odom_active_ = true;
            odom_initializing_ = false;
            ROS_INFO_STREAM("Odom activated");
        }
    }

}

void DataFusion::icpOdomCallback(const nav_msgs::Odometry::ConstPtr& icp_odom)
{
    // ROS_INFO_STREAM("IcpOdom callback at time: " << ros::Time::now().toSec());

    icp_odom_stamp_ = icp_odom->header.stamp;
    tf::Quaternion q;
    tf::quaternionMsgToTF(icp_odom->pose.pose.orientation, q);

    icp_odom_mean = Eigen::Vector2d(icp_odom->pose.pose.position.x, icp_odom->pose.pose.position.y);

    icp_odom_meas_ = tf::Transform(q, tf::Vector3(icp_odom->pose.pose.position.x, icp_odom->pose.pose.position.y, 0));

    icp_odom_covariance_ = Eigen::MatrixXd(6, 6);
    for(unsigned int i = 0; i < 6; i++)
        for(unsigned int j = 0; j < 6; j++)
            icp_odom_covariance_(i, j) = icp_odom->pose.covariance[6*i+j];

    // std::cout << "odom_covariance_: " << std::endl << icp_odom_covariance_ << std::endl;

    kf_.AddMeasurement(tf::StampedTransform(icp_odom_meas_.inverse(), icp_odom_stamp_, base_footprint_frame_, "odom"), icp_odom_covariance_);
}

void DataFusion::spin(const ros::TimerEvent& e)
{
    // ROS_INFO_STREAM("Spin function at time: " << ros::Time::now().toSec());

    filter_stamp_ = ros::Time::now();


    if(is_initialized_)
    {
        // 求前后两帧的时间差，数据包中的时间戳单位为微秒，处以1e6，转换为秒
        double delta_t = wheel_odom_stamp_.toSec() - last_timestamp_;
        last_timestamp_ = wheel_odom_stamp_.toSec();

        // std::cout << "last_timestamp_: " << last_timestamp_ << std::endl;


        // 设置状态转移矩阵F
        Eigen::MatrixXd F = Eigen::MatrixXd(4, 4);
        F << 1.0, 0.0, delta_t, 0.0,
             0.0, 1.0, 0.0, delta_t,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;

        // std::cout << "delta_t: " << std::endl << delta_t << std::endl;

        kf_.SetF(F);

        // 预测
        kf_.Prediction();

        // 更新
        kf_.SetH(H_lidar_);
        kf_.SetR(R_lidar_);
        kf_.KFUpdate(icp_odom_mean);

    }

    // 第一帧数据用于初始化 Kalman 滤波器
    if (odom_active_ && !is_initialized_) {

        // 避免运算时，0作为被除数
        if (fabs(wheel_odom_mean(0)) < 0.001) {
            wheel_odom_mean(0) = 0.001;
        }
        if (fabs(wheel_odom_mean(1)) < 0.001) {
            wheel_odom_mean(1) = 0.001;
        }

        // 初始化Kalman滤波器
        kf_.Initialization(wheel_odom_mean);

        // std::cout << "wheel_odom_mean: " << std::endl << wheel_odom_mean << std::endl;

        // 设置协方差矩阵P
        Eigen::MatrixXd P = Eigen::MatrixXd(4, 4);
        P << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1000.0, 0.0,
             0.0, 0.0, 0.0, 1000.0;
        kf_.SetP(P);

        // 设置过程噪声Q
        Eigen::MatrixXd Q = Eigen::MatrixXd(4, 4);
        Q << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;
        kf_.SetQ(Q);

        // // 存储第一帧的时间戳，供下一帧数据使用
        last_timestamp_ = wheel_odom_init_stamp_.toSec();
        is_initialized_ = true;
        return;
    }


    std::cout << "KFUpdate_after x_: " << kf_.GetX()(0) << ", " << kf_.GetX()(1) << std::endl;

    nav_msgs::Odometry combined_odom_msg;
    combined_odom_msg.header.frame_id= "odom";
    combined_odom_msg.pose.pose.position.x = kf_.GetX()(0);
    combined_odom_msg.pose.pose.position.y = kf_.GetX()(1);
    pose_pub_.publish(combined_odom_msg);

}

void DataFusion::Process(Measurement measurement)
{
    // 第一帧数据用于初始化 Kalman 滤波器
    if (!is_initialized_) {
        Eigen::Vector4d x;
        if (measurement.sensor_type_ == Measurement::LASER) {
            // 如果第一帧数据是激光雷达数据，没有速度信息，因此初始化时只能传入位置，速度设置为0
            x << measurement.raw_measurements_[0], measurement.raw_measurements_[1], 0, 0;
        } else if (measurement.sensor_type_ == Measurement::RADAR) {
            // 如果第一帧数据是毫米波雷达，可以通过三角函数算出x-y坐标系下的位置和速度
            float rho = measurement.raw_measurements_[0];
            float phi = measurement.raw_measurements_[1];
            float rho_dot = measurement.raw_measurements_[2];
            float position_x = rho * cos(phi);
            if (position_x < 0.0001) {
                position_x = 0.0001;
            }
            float position_y = rho * sin(phi);
            if (position_y < 0.0001) {
                position_y = 0.0001;
            }
            float velocity_x = rho_dot * cos(phi);
            float velocity_y = rho_dot * sin(phi);
            x << position_x, position_y, velocity_x , velocity_y;
        }

        // std::cout << "x " << x << std::endl;
        
        // 避免运算时，0作为被除数
        if (fabs(x(0)) < 0.001) {
            x(0) = 0.001;
        }
        if (fabs(x(1)) < 0.001) {
            x(1) = 0.001;
        }
        // 初始化Kalman滤波器
        kf_.Initialization(x);

        // 设置协方差矩阵P
        Eigen::MatrixXd P = Eigen::MatrixXd(4, 4);
        P << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1000.0, 0.0,
             0.0, 0.0, 0.0, 1000.0;
        kf_.SetP(P);

        // 设置过程噪声Q
        Eigen::MatrixXd Q = Eigen::MatrixXd(4, 4);
        Q << 1.0, 0.0, 0.0, 0.0,
             0.0, 1.0, 0.0, 0.0,
             0.0, 0.0, 1.0, 0.0,
             0.0, 0.0, 0.0, 1.0;
        kf_.SetQ(Q);

        // 存储第一帧的时间戳，供下一帧数据使用
        // last_timestamp_ = measurement.timestamp_;
        is_initialized_ = true;
        return;
    }

    // 求前后两帧的时间差，数据包中的时间戳单位为微秒，处以1e6，转换为秒
    // double delta_t = (measurement.timestamp_ - last_timestamp_) / 1000000.0; // unit : s
    // last_timestamp_ = measurement.timestamp_;

    double delta_t = 1.0;

    // 设置状态转移矩阵F
    Eigen::MatrixXd F = Eigen::MatrixXd(4, 4);
    F << 1.0, 0.0, delta_t, 0.0,
         0.0, 1.0, 0.0, delta_t,
         0.0, 0.0, 1.0, 0.0,
         0.0, 0.0, 0.0, 1.0;
    kf_.SetF(F);

    // 预测
    kf_.Prediction();

    // 更新
    if (measurement.sensor_type_ == Measurement::LASER) {
        kf_.SetH(H_lidar_);
        kf_.SetR(R_lidar_);
        kf_.KFUpdate(measurement.raw_measurements_);
    } else if (measurement.sensor_type_ == Measurement::RADAR) {
        kf_.SetR(R_radar_);
        // Jocobian矩阵Hj的运算已包含在EKFUpdate中
        kf_.EKFUpdate(measurement.raw_measurements_);
    }
}
}


