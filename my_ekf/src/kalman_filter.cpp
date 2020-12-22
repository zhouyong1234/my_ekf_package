#include <iostream>
#include "my_ekf/kalman_filter.hpp"

namespace my_ekf_package
{
KalmanFilter::KalmanFilter()
{
    ros::Time::init();
    is_initialized_ = false;
    previous_time_imu_ = ros::Time::now().toSec();
    previous_time_wheel_odom_ = ros::Time::now().toSec();
    x_ << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
    gravity_ = Eigen::Vector3d(0, 0, 9.8);
    P_ = Eigen::Matrix<double, 10, 10>::Identity() * 100;
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Initialization(Eigen::VectorXd x_in)
{
    x_ = x_in;

    // std::cout << "x_: " << std::endl << x_ << std::endl;

}


bool KalmanFilter::IsInitialized()
{
    return is_initialized_;
}


/*******************************************************
通过里程计预测位姿
********************************************************/
void KalmanFilter::Prediction(const double current_time_wheel_odom, const Eigen::Vector3d& pose_msg, const Eigen::Vector4d& orientation_msg)
{
    double delta_t = current_time_wheel_odom - previous_time_wheel_odom_;
    previous_time_wheel_odom_ = current_time_wheel_odom;

    x_.segment(0, 3) = pose_msg;
    // x_.segment(6, 4) = orientation_msg;

    Eigen::Matrix<double, 10, 10> F = Eigen::Matrix<double, 10, 10>::Identity();

    Eigen::MatrixXd Q = Eigen::Matrix<double, 10, 10>::Identity();
    Q.block<3,3>(0,0) = 0.33 * Q.block<3,3>(0,0);
    Q.block<4,4>(6,6) = 1000 * Q.block<4,4>(6,6);

    P_ = F * P_ * F.transpose() + Q;

    // std::cout << "P_: " << std::endl << P_ << std::endl;
}


/*******************************************************
通过IMU预测位姿
********************************************************/
// void KalmanFilter::Prediction(const double current_time_imu, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc)
// {
//     double delta_t = current_time_imu - previous_time_imu_;
//     previous_time_imu_ = current_time_imu;

//     // std::cout << "delta_t: " << delta_t << std::endl;

//     if(delta_t > 0.5)
//     {
//         std::cout << "IMU time interval is too large..." << std::endl;
//         return;
//     }

//     // 欧拉角转四元数 角度 = 角速度 x 时间 quaternion = yawAngle * pitchAngle * rollAngle
//     Eigen::Quaterniond quat_wdt = Eigen::Quaterniond(
//         Eigen::AngleAxisd(gyro.x() * delta_t, Eigen::Vector3d::UnitX()) *
//         Eigen::AngleAxisd(gyro.y() * delta_t, Eigen::Vector3d::UnitY()) *
//         Eigen::AngleAxisd(gyro.z() * delta_t, Eigen::Vector3d::UnitZ()));

//     // std::cout << "acc: " << std::endl << acc << std::endl;

//     // Eigen::Vector3d acc = Eigen::Vector3d(acc.x(), acc.y(), acc.z());

//     // std::cout << "before x_: " << std::endl << x_(6) << x_(7) << x_(8) << x_(9) << std::endl;

//     Eigen::Quaterniond previous_quat = Eigen::Quaterniond(x_(9), x_(6), x_(7), x_(8));

//     // std::cout << "previous_quat: " << std::endl << previous_quat.x() << ", " << previous_quat.y() << ", " << previous_quat.z() << ", " << previous_quat.w() << std::endl;

//     // std::cout << "quat_wdt: " << std::endl << quat_wdt.x() << ", " << quat_wdt.y() << ", " << quat_wdt.z() << ", " << quat_wdt.w() << std::endl;

//     // 四元数转旋转矩阵
//     Eigen::MatrixXd rot_mat = previous_quat.toRotationMatrix();

//     // std::cout << "rot_mat: " << std::endl << rot_mat << std::endl;

//     // 预测四元数 = 当前四元数 x 初始四元数 两个四元数的乘积也表示一个旋转
//     Eigen::Quaterniond predicted_quat = quat_wdt * previous_quat;

//     // std::cout << "predicted_quat: " << std::endl << predicted_quat.x() << ", " << predicted_quat.y() << ", " << predicted_quat.z() << ", " << predicted_quat.w() << std::endl;

//     // pose update
//     /**********************************************
//     目前为单轴IMU，无法通过加速度积分出位移和速度
//     ***********************************************/
//     // x_.segment(0, 3) = x_.segment(0, 3) + x_.segment(3, 3) * delta_t + 0.5 * (rot_mat * acc - gravity_) * delta_t * delta_t;
//     // x_.segment(3, 3) = x_.segment(3, 3) + (rot_mat * acc - gravity_) * delta_t;
//     x_.segment(6, 4) = Eigen::Vector4d(predicted_quat.x(), predicted_quat.y(), predicted_quat.z(), predicted_quat.w());

//     // std::cout << "x_: " << std::endl << x_ << std::endl;

//     Eigen::Matrix<double, 9, 9> F = Eigen::Matrix<double, 9, 9>::Identity();
//     // F.block<3, 3>(0, 3) = delta_t * Eigen::Matrix3d::Identity();

//     // Eigen::Matrix3d acc_skew;
//     // acc_skew << 
//     //     0, -acc(2), acc(1),
//     //     acc(2), 0, -acc(0),
//     //     -acc(1), acc(0), 0;
//     // F.block<3,3>(3, 6) = rot_mat * (-acc_skew) * delta_t;

//     Eigen::MatrixXd Q = Eigen::Matrix<double, 6, 6>::Identity();
//     Q.block<3,3>(0,0) = 0.33 * Q.block<3,3>(0,0);
//     Q.block<3,3>(3,3) = 0.33 * Q.block<3,3>(3,3);

//     Q = Q * (delta_t * delta_t);

//     Eigen::MatrixXd L = Eigen::Matrix<double, 9, 6>::Zero();
//     L.block<3,3>(3,0) = Eigen::Matrix3d::Identity();
//     L.block<3,3>(6,3) = Eigen::Matrix3d::Identity();

//     P_ = F * P_ * F.transpose() + L * Q * L.transpose();

//     // std::cout << "P_: " << std::endl << P_ << std::endl;

// }

void KalmanFilter::Prediction()
{

    // Eigen::VectorXd x_ = Eigen::VectorXd(4, 1);
    // Eigen::MatrixXd F_ = Eigen::MatrixXd(4, 4);
    // Eigen::MatrixXd P_ = Eigen::MatrixXd(4, 4);
    // Eigen::MatrixXd Q_ = Eigen::MatrixXd(4, 4);

    // std::cout << "Prediction_before x_: " << x_(0) << " " << x_(1) << " " << x_(2) << " " << " " << x_(3) << std::endl;
    // std::cout << "Prediction F_: " << std::endl << F_ << std::endl;
    // std::cout << "Prediction P_: " << std::endl << P_ << std::endl;
    // std::cout << "Prediction Q_: " << std::endl << Q_ << std::endl;

    x_ = F_ * x_;
    P_ = F_ * P_ * F_.transpose() + Q_;
    // std::cout << "Prediction_after x_: " << x_(0) << " " << x_(1) << " " << x_(2) << " " << " " << x_(3) << std::endl;
    

}

void KalmanFilter::KFUpdate(Eigen::VectorXd z){

    // std::cout << "z: " << std::endl << z << std::endl;

    // std::cout << "H_: " << std::endl << H_ << std::endl;

    // std::cout << "x_: " << std::endl << x_ << std::endl;

    // std::cout << "after_kf x_: " << std::endl << x_ << std::endl;

    // std::cout << "KFUpdate_before x_: " << x_(0) << " " << x_(1) << " " << x_(2) << " " << " " << x_(3) << std::endl;

    // std::cout << "KFUpdate H_: " << std::endl << H_ << std::endl;

    // Eigen::VectorXd x_ = Eigen::VectorXd(4, 1);
    // Eigen::MatrixXd H_ = Eigen::MatrixXd(2, 4);
    // Eigen::VectorXd y_ = Eigen::VectorXd(2, 1);
    // Eigen::MatrixXd S_ = Eigen::MatrixXd(2, 2);
    // Eigen::MatrixXd P_ = Eigen::MatrixXd(4, 4);
    // Eigen::MatrixXd R_ = Eigen::MatrixXd(2, 2);
    // Eigen::MatrixXd K_ = Eigen::MatrixXd(4, 2);
    // Eigen::MatrixXd I_ = Eigen::MatrixXd(4, 4);

    y_ = z - H_ * x_;
    S_ = H_ * P_ * H_.transpose() + R_;
    K_ = P_ * H_.transpose() * S_.inverse();
    x_ = x_ + (K_ * y_);

    int x_size = x_.size();
    I_ = Eigen::MatrixXd::Identity(x_size, x_size);
    P_ = (I_ - K_ * H_) * P_;

    // std::cout << "KFUpdate_after x_: " << std::endl << x_.segment(0,2) << std::endl;

    // std::cout << "KFUpdate_after P_: " << std::endl << P_ << std::endl;

    // Eigen::MatrixXd F = Eigen::Matrix<double, 10, 10>::Identity();

    // std::cout << "F: " << std::endl << F << std::endl;

    // Eigen::Affine3d aff = Eigen::Affine3d::Identity();

    // aff.translation() = Eigen::Vector3d(0.1,0.1,0.1);

    // std::cout << "aff: " << std::endl << aff.translation() << std::endl;


}

/*******************************************************
通过里程计更新位姿
********************************************************/
void KalmanFilter::KFUpdate(const Eigen::Vector3d& pose_msg, const Eigen::Vector3d& variance)
{
    Eigen::Matrix3d R;
    R << variance.x(), 0, 0,
         0, variance.y(), 0,
         0, 0, variance.z();

    Eigen::MatrixXd H = Eigen::Matrix<double, 3, 10>::Zero();
    H.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();

    Eigen::VectorXd dx = K * (pose_msg - x_.segment(0, 3));

    // std::cout << "dx: " << std::endl << dx << std::endl;

    x_.segment(0, 3) = x_.segment(0, 3) + dx.segment(0, 3);
    x_.segment(3, 3) = x_.segment(3, 3) + dx.segment(3, 3);
    x_.segment(6, 4) = x_.segment(6, 4) + dx.segment(6, 4);

    // double norm_quat = sqrt(
    //     pow(dx(6), 2) + 
    //     pow(dx(7), 2) +
    //     pow(dx(8), 2));

    // // std::cout << "norm_quat: " << std::endl << norm_quat << std::endl;

    // if(norm_quat < 1e-10)
    // {
    //     x_.segment(6, 4) = Eigen::Vector4d(0, 0, 0, cos(norm_quat / 2));
    // }    
    // else
    // {
    //     // 等效旋转轴方向向量 K = [Kx, Ky, Kz] 等效旋转角Θ，则四元数q=(x, y, z,w), x = Kx . sin(Θ/2), y = Ky . sin(Θ/2), z = Kz . sin(Θ/2), w = cos(Θ/2)
    //     x_.segment(6, 4) = Eigen::Vector4d(
    //         sin(norm_quat / 2) * dx(6) / norm_quat,
    //         sin(norm_quat / 2) * dx(7) / norm_quat,
    //         sin(norm_quat / 2) * dx(8) / norm_quat,
    //         cos(norm_quat / 2));
    // }

    P_ = (Eigen::Matrix<double, 10, 10>::Identity() - K * H) * P_;

    // std::cout << "P_: " << std::endl << P_ << std::endl;
}

/*******************************************************
通过IMU更新位姿
********************************************************/
void KalmanFilter::KFUpdate(const Eigen::Vector4d& orientation_msg, const Eigen::Vector4d& variance)
{
    Eigen::Matrix4d R;
    R << variance.x(), 0, 0, 0,
         0, variance.y(), 0, 0,
         0, 0, variance.z(), 0,
         0, 0, 0, variance.w();

    Eigen::Quaterniond previous_quat = Eigen::Quaterniond(x_(9), x_(6), x_(7), x_(8));

    // std::cout << "previous_quat: " << previous_quat.x() << ", " << previous_quat.y() << ", " << previous_quat.z() << ", " << previous_quat.w() << std::endl;

    Eigen::Quaterniond current_quat = Eigen::Quaterniond(orientation_msg(3), orientation_msg(0), orientation_msg(1), orientation_msg(2));

    // std::cout << "current_quat: " << current_quat.x() << ", " << current_quat.y() << ", " << current_quat.z() << ", " << current_quat.w() << std::endl;

    // 预测四元数 = 当前四元数 x 初始四元数 两个四元数的乘积也表示一个旋转
    Eigen::Quaterniond predicted_quat = current_quat * previous_quat;

    // Eigen::Vector4d y = Eigen::Vector4d(predicted_quat.x(), predicted_quat.y(), predicted_quat.z(), predicted_quat.w());
    Eigen::Vector4d y = Eigen::Vector4d(current_quat.x(), current_quat.y(), current_quat.z(), current_quat.w());

    // std::cout << "y: " << std::endl << y << std::endl;

    Eigen::MatrixXd H = Eigen::Matrix<double, 4, 10>::Zero();
    H.block<4,4>(0,6) = Eigen::Matrix4d::Identity();

    Eigen::MatrixXd K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();

    Eigen::VectorXd dx = K * (y - x_.segment(6, 4));

    x_.segment(0, 3) = x_.segment(0, 3) + dx.segment(0, 3);
    x_.segment(3, 3) = x_.segment(3, 3) + dx.segment(3, 3);
    x_.segment(6, 4) = x_.segment(6, 4) + dx.segment(6, 4);

    P_ = (Eigen::Matrix<double, 10, 10>::Identity() - K * H) * P_;
}

Eigen::VectorXd KalmanFilter::GetX()
{
    return x_;
}
}


