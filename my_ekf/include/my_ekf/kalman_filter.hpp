#ifndef MY_EKF_KALMAN_FILTER_HPP_
#define MY_EKF_KALMAN_FILTER_HPP_

#include <tf/tf.h>
#include <ros/ros.h>
#include "Eigen/Eigen"
#include "geometry_msgs/PoseStamped.h"


namespace my_ekf_package
{
class KalmanFilter{
public:
    KalmanFilter();
    ~KalmanFilter();

    bool IsInitialized();
    void Initialization(Eigen::Vector4d x_in);
    // void Prediction();
    void Prediction(const Eigen::Vector2d& input, const double delta_t, const double pf_inverse);
    Eigen::Matrix4d CalculateJacobian(const Eigen::Vector4d& x, const Eigen::Vector2d& input, const double delta_t);
    void IMUEKFUpdate(const Eigen::Vector2d& z);
    void GNSSEKFUpdate(const Eigen::Vector2d& z);
    // void Prediction(const double current_time_wheel_odom, const Eigen::Vector3d& pose_msg, const Eigen::Vector4d& orientation_msg);
    // void Prediction(const double current_time_imu, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc);
    // void KFUpdate(Eigen::VectorXd z);
    // void KFUpdate(const Eigen::Vector4d& orientation_msg, const Eigen::Vector4d& variance);
    // void KFUpdate(const Eigen::Vector3d& pose_msg, const Eigen::Vector3d& variance);
    Eigen::Vector4d GetX();

    
private:
    bool is_initialized_;
    double previous_time_imu_;
    double previous_time_wheel_odom_;

    Eigen::Vector3d gravity_;
    // Eigen::Matrix<double, 10, 1> x_;
    // Eigen::Matrix<double, 10, 10> P_;

    Eigen::Vector4d x_;
    Eigen::Matrix<double, 4, 4> P_;

    // Eigen::Vector4d x_;
    // Eigen::MatrixXd F_;
    // Eigen::MatrixXd P_;
    // Eigen::MatrixXd Q_;
    // Eigen::MatrixXd H_;
    // Eigen::MatrixXd R_;
    // Eigen::VectorXd y_;
    // Eigen::MatrixXd S_;
    // Eigen::MatrixXd K_;
    // Eigen::MatrixXd I_;

};
}

#endif