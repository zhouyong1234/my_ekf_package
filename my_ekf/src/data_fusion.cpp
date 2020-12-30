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

    std::cout << "gps_used_: " << gps_used_ << std::endl;

    // timer_ = nh.createTimer(ros::Duration(0.1), &DataFusion::spin, this);

    // ROS_INFO_STREAM("1111111111111111111");

    filter_stamp_ = ros::Time::now();

    pose_pub_ = nh.advertise<nav_msgs::Odometry>("odom_combined", 10);

    imu_data_sub_ = nh.subscribe("IMU_data", 10, &DataFusion::imuDataCallback, this);

    wheel_odom_sub_ = nh.subscribe("wheel_odom", 10, &DataFusion::wheelOdomCallback, this);

    // icp_odom_sub_ = nh.subscribe("icp_odom", 10, &DataFusion::icpOdomCallback, this);

    gnss_data_sub_ = nh.subscribe("gnss_pose", 10, &DataFusion::gnssCallback, this);

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
        // double current_time_imu = imu_data->header.stamp.sec + imu_data->header.stamp.nsec * 1e-9;
        // gyro_ = Eigen::Vector3d(imu_data->angular_velocity.x, imu_data->angular_velocity.y, imu_data->angular_velocity.z);
        // acc_ = Eigen::Vector3d(imu_data->linear_acceleration.x, imu_data->linear_acceleration.y, imu_data->linear_acceleration.z);

        // orientation_ = Eigen::Vector4d(imu_data->orientation.x, imu_data->orientation.y, imu_data->orientation.z, imu_data->orientation.w);
        // kf_.Prediction(current_time_imu, gyro, acc);

        double current_yaw = tf::getYaw(imu_data->orientation);

        // std::cout << "current_yaw: " << current_yaw << std::endl;

        current_yaw = atan2(sin(current_yaw), cos(current_yaw));

        Eigen::Vector2d z(current_yaw, 0.0);


        // std::cout << "current_yaw: " << current_yaw << std::endl;

        // 使用IMU做更新
        kf_.IMUEKFUpdate(z);
        // kf_.KFUpdate(orientation_, imu_var_);
    }
}

/*******************************************************
轮式里程计回调函数
********************************************************/
void DataFusion::wheelOdomCallback(const nav_msgs::Odometry::ConstPtr& wheel_odom)
{

    // std::cout << "wheelOdomCallback............." << std::endl;
    new_wheel_odom_data_.push_back(wheel_odom);
}

/*******************************************************
激光里程计回调函数
********************************************************/
// void DataFusion::icpOdomCallback(const nav_msgs::Odometry::ConstPtr& icp_odom)
// {
//     // ROS_INFO_STREAM("IcpOdom callback at time: " << ros::Time::now().toSec());

//     current_stamp_ = icp_odom->header.stamp;

//     if(is_initialized_ && icp_odom_used_)
//     {
//         // 当前里程计位姿矩阵
//         Eigen::Affine3d aff;
//         tf2::fromMsg(icp_odom->pose.pose, aff);
//         Eigen::Matrix4d odom_mat = aff.matrix();
//         if(previous_odom_mat_ == Eigen::Matrix4d::Identity())
//         {
//             current_pose_odom_ = current_pose_;
//             previous_odom_mat_ = odom_mat;
//             return;
//         }

//         Eigen::Affine3d current_aff;
//         // 计算相对运动，由前两帧的位姿增量计算下一帧的位姿
//         tf2::fromMsg(current_pose_odom_.pose, current_aff);
//         // 初始帧的位姿
//         Eigen::Matrix4d current_trans = current_aff.matrix();
//         // 下一帧位姿 = 位姿增量 x 当前帧位姿
//         current_trans = current_trans * previous_odom_mat_.inverse() * odom_mat;

//         Eigen::Vector3d pose_msg;
//         pose_msg(0) = current_trans(0, 3);
//         pose_msg(1) = current_trans(1, 3);
//         pose_msg(2) = current_trans(2, 3);

//         // 使用激光里程计做更新
//         kf_.KFUpdate(pose_msg, icp_odom_var_);

//         current_pose_odom_ = current_pose_;
//         previous_odom_mat_ = odom_mat;
//     }

// }

/*******************************************************
GNSS回调函数
********************************************************/
void DataFusion::gnssCallback(const geometry_msgs::PoseStamped::ConstPtr& gnss)
{
    new_gnss_data_.push_back(gnss);
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
    combined_odom_msg.twist.twist.linear.x = kf_.GetX()(3) * cos(kf_.GetX()(2));
    combined_odom_msg.twist.twist.linear.y = kf_.GetX()(3) * sin(kf_.GetX()(2));

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, kf_.GetX()(2));

    combined_odom_msg.pose.pose.orientation.x = q.x();
    combined_odom_msg.pose.pose.orientation.y = q.y();
    combined_odom_msg.pose.pose.orientation.z = q.z();
    combined_odom_msg.pose.pose.orientation.w = q.w();
    pose_pub_.publish(combined_odom_msg);

    // std::cout << "combined_odom_msg: " << combined_odom_msg.pose.pose.position.x << ", " << 
    //                                       combined_odom_msg.pose.pose.position.y << ", " <<
    //                                       combined_odom_msg.pose.pose.position.z << ", " <<
    //                                       combined_odom_msg.twist.twist.linear.x << ", " <<
    //                                       combined_odom_msg.twist.twist.linear.y << ", " <<
    //                                       combined_odom_msg.twist.twist.linear.z << ", " <<
    //                                       combined_odom_msg.pose.pose.orientation.x << ", " <<
    //                                       combined_odom_msg.pose.pose.orientation.y << ", " <<
    //                                       combined_odom_msg.pose.pose.orientation.z << ", " <<
    //                                       combined_odom_msg.pose.pose.orientation.w << std::endl;

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



/*******************************************************
启动函数：
1、读取数据
2、验证是否有数据
3、验证数据是否有效，时间同步和位置同步
4、如果无效只做预测更新，如果有效做观测更新
********************************************************/
bool DataFusion::run()
{

    // std::cout << "run........................" << std::endl;

    readData();

    if(!is_initialized_)
    {

        if(wheel_odom_data_buff_.size() > 0)
        {
            current_wheel_odom_data_ = wheel_odom_data_buff_.front();

            double current_yaw = tf::getYaw(current_wheel_odom_data_->pose.pose.orientation);
            current_yaw = atan2(sin(current_yaw), cos(current_yaw));

            wheel_odom_init_ = Eigen::VectorXd::Zero(4);
            wheel_odom_init_(0) = current_wheel_odom_data_->pose.pose.position.x;
            wheel_odom_init_(1) = current_wheel_odom_data_->pose.pose.position.y;
            wheel_odom_init_(2) = current_yaw;

            // std::cout << "wheel_odom_init_: " << std::endl << wheel_odom_init_ << std::endl;
    
            // 使用里程计位姿初始化Kalman滤波器
            kf_.Initialization(wheel_odom_init_);

            ROS_INFO_STREAM("Initialized......");
            is_initialized_ = true;
            return true;
        }  
    }

    if(hasData())
    {
        if(!validData())
        {
            std::cout << "--------------->predict<---------------" << std::endl;
            predict();
            broadcastPose();
            return false;
        }
            
        std::cout << "--------------->update<---------------" << std::endl;
        update();
        broadcastPose();
    }

    // while(hasData())
    // {
    //     if(!validData())
    //         predict();
    //         broadcastPose();
    //         continue;

    //     // update();
    //     // broadcastPose();
    // }

    return true;

}

/*******************************************************
订阅不同的传感器数据，放入对应的传感器数据队列中
********************************************************/
bool DataFusion::readData()
{
    parseWheelOdomData(wheel_odom_data_buff_);
    parseGNSSData(gnss_data_buff_);

    // std::cout << "--------------read_data--------------" << std::endl;

    // std::cout << "wheel_odom_data_buff_.size(): " << wheel_odom_data_buff_.size() << std::endl;
    // std::cout << "gnss_data_buff_.size(): " << gnss_data_buff_.size() << std::endl;

    return true;
}

/*******************************************************
校验对应的传感器数据队列是否有数据
********************************************************/
bool DataFusion::hasData()
{
    if(wheel_odom_data_buff_.size() == 0)
        return false;
    // if(gnss_data_buff_.size() == 0)
    //     return false;

    return true;
}

/*******************************************************
校验对应的传感器数据队列是否有效，实现传感器数据时间对齐和位置对齐
********************************************************/
bool DataFusion::validData()
{
    current_wheel_odom_data_ = wheel_odom_data_buff_.front();
    if(gnss_data_buff_.size() > 0)
    {
        current_gnss_data_ = gnss_data_buff_.front();

        double diff_time = current_wheel_odom_data_->header.stamp.toSec() - current_gnss_data_->header.stamp.toSec();
        double diff_distance_x = current_wheel_odom_data_->pose.pose.position.x - current_gnss_data_->pose.position.x;
        double diff_distance_y = current_wheel_odom_data_->pose.pose.position.y - current_gnss_data_->pose.position.y;


        // std::cout << "diff_time: " << diff_time << std::endl;

        // std::cout << "diff_distance_x: " << diff_distance_x << std::endl;

        // std::cout << "diff_distance_y: " << diff_distance_y << std::endl;

        // 时间同步，如果时间戳差值太大，则舍弃时间戳早的那个
        if(diff_time < -0.2)
        {
            wheel_odom_data_buff_.pop_front();
            return false;
        }

        if(diff_time > 0.2)
        {
            gnss_data_buff_.pop_front();
            return false;
        }

        // 位置同步，如果轮式里程计数据和gnss数据位置差值太大，则舍弃gnss数据
        if (fabs(diff_distance_x) >= 1.0 || fabs(diff_distance_y) >= 1.0)
        {
            gnss_data_buff_.pop_front();
            return false;
        }

        wheel_odom_data_buff_.pop_front();
        gnss_data_buff_.pop_front();

        // std::cout << "---------------return true---------------" << std::endl;

        return true;
    }
    else
    {
        wheel_odom_data_buff_.pop_front();
        return false;
    }
    
    return false;
    
}

/*******************************************************
将轮式里程计数据放入队列中
********************************************************/
void DataFusion::parseWheelOdomData(std::deque<nav_msgs::Odometry::ConstPtr>& wheel_odom_data_buff)
{
    if(new_wheel_odom_data_.size() > 0)
    {
        wheel_odom_data_buff.insert(wheel_odom_data_buff.end(), new_wheel_odom_data_.begin(), new_wheel_odom_data_.end());
        new_wheel_odom_data_.clear();
    }
}

/*******************************************************
将GNSS数据放入队列中
********************************************************/
void DataFusion::parseGNSSData(std::deque<geometry_msgs::PoseStamped::ConstPtr>& gnss_data_buff)
{
    if(new_gnss_data_.size() > 0)
    {
        gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
        new_gnss_data_.clear();
    }
}

void DataFusion::predict()
{
    static double last_yaw = 0;
    static double first_yaw = 0;
    static bool first_predict = true;
    static double last_time = ros::Time::now().toSec();
    static Eigen::Vector3d first_odom = Eigen::Vector3d::Zero();
    static Eigen::Vector3d last_odom = Eigen::Vector3d::Zero();

    double current_yaw = tf::getYaw(current_wheel_odom_data_->pose.pose.orientation);
    current_yaw -= first_yaw;
    current_yaw = atan2(sin(current_yaw), cos(current_yaw));

    Eigen::Vector3d current_odom(current_wheel_odom_data_->pose.pose.position.x, current_wheel_odom_data_->pose.pose.position.y, current_wheel_odom_data_->pose.pose.position.z);

    // std::cout << "current_odom: " << std::endl << current_odom << std::endl;

    Eigen::Vector3d current_v(current_wheel_odom_data_->twist.twist.linear.x, current_wheel_odom_data_->twist.twist.linear.y, current_wheel_odom_data_->twist.twist.linear.z);

    current_odom -= first_odom;

    Eigen::AngleAxis<double> first_yaw_rotation(-first_yaw, Eigen::Vector3d::UnitZ());
    current_odom = first_yaw_rotation * current_odom;

    

    current_stamp_ = current_wheel_odom_data_->header.stamp;

    if(!first_predict && is_initialized_ && wheel_odom_used_)
    {

        // std::cout << "current_wheel_odom_data_: " << std::endl << current_wheel_odom_data_->pose.pose << std::endl;

        double current_time = current_wheel_odom_data_->header.stamp.sec + current_wheel_odom_data_->header.stamp.nsec * 1e-9;

        double delta_t = current_time - last_time;

        if (fabs(delta_t) < 0.001) {
            delta_t = 0.001;
        }

        // std::cout << "current_odom: " << std::endl << current_odom << std::endl;
        // std::cout << "last_odom: " << std::endl << last_odom << std::endl;

        // v = dx / t
        double current_velocity = (current_odom - last_odom).norm() / delta_t;

        if(current_odom.norm() <= last_odom.norm())
        {
            current_velocity = -current_velocity;
        }

        // w = dΘ / t
        double current_omega = (current_yaw - last_yaw) / delta_t;

        Eigen::Vector2d u(current_velocity, current_omega);

        // std::cout << "u: " << std::endl << u << std::endl;

        kf_.Prediction(u, delta_t);

        last_time = current_time;
        last_odom = current_odom;
        last_yaw = current_yaw;
    }
    else
    {

        first_odom = current_odom;
        last_odom = current_odom - first_odom;
        first_yaw = current_yaw;
        last_yaw = current_yaw - first_yaw;
        first_predict = false;
    }

    // if(!odom_active_)
    // {
    //     if(!odom_initializing_)
    //     {
    //         odom_initializing_ = true;
    //         wheel_odom_init_stamp_ = current_stamp_;
    //         // ROS_INFO_STREAM("Initializing odom");
    //     }

    //     if(filter_stamp_ >= wheel_odom_init_stamp_)
    //     {

    //         wheel_odom_init_ = Eigen::VectorXd::Zero(4);
    //         wheel_odom_init_(0) = current_odom(0);
    //         wheel_odom_init_(1) = current_odom(1);
    //         wheel_odom_init_(2) = current_yaw;
    //         wheel_odom_init_(3) = current_v.norm();
    //         odom_active_ = true;
    //         odom_initializing_ = false;

    //         first_odom = current_odom;
    //         last_odom = current_odom - first_odom;
    //         first_yaw = current_yaw;
    //         last_yaw = current_yaw - first_yaw;
            
    //         // ROS_INFO_STREAM("Odom activated");
    //     }
    // }
}

void DataFusion::update()
{
    // std::cout << "new_gnss_data_.size(): " << new_gnss_data_.size() << std::endl;

    current_stamp_ = current_gnss_data_->header.stamp;

    // std::cout << "current_current_gnss_data__stamp_: " << gnss->header.stamp.sec << std::endl;

    if(is_initialized_ && gps_used_)
    {
        Eigen::Vector2d pose_msg;
        pose_msg(0) = current_gnss_data_->pose.position.x;
        pose_msg(1) = current_gnss_data_->pose.position.y;

        // 使用gnss做更新
        kf_.GNSSEKFUpdate(pose_msg);
    }
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
    // if (odom_active_ && !is_initialized_) {
    if(!is_initialized_){

        // 避免运算时，0作为被除数
        // if (fabs(wheel_odom_init_(0)) < 0.001) {
        //     wheel_odom_init_(0) = 0.001;
        // }
        // if (fabs(wheel_odom_init_(1)) < 0.001) {
        //     wheel_odom_init_(1) = 0.001;
        // }
        // if(fabs(wheel_odom_init_(2)) < 0.001) {
        //     wheel_odom_init_(2) = 0.001;
        // }

        wheel_odom_init_ = Eigen::VectorXd::Zero(4);

        // 使用里程计位姿初始化Kalman滤波器
        kf_.Initialization(wheel_odom_init_);

        // std::cout << "wheel_odom_init_: " << wheel_odom_init_ << std::endl;

        ROS_INFO_STREAM("Initialized......");
        is_initialized_ = true;
        return;
    }

}
}


