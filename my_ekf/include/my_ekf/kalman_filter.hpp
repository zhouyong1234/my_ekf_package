#ifndef MY_EKF_KALMAN_FILTER_HPP_
#define MY_EKF_KALMAN_FILTER_HPP_

#include <tf/tf.h>
#include <ros/ros.h>
#include "Eigen/Eigen"

namespace my_ekf_package
{
class KalmanFilter{
public:
    KalmanFilter();
    ~KalmanFilter();

    bool IsInitialized();

    void Initialization(Eigen::Vector4d x_in);

    void Initialize(const tf::Transform& transform, const ros::Time& time);

    void SetF(Eigen::MatrixXd F_in);
    void SetP(Eigen::MatrixXd P_in);
    void SetQ(Eigen::MatrixXd Q_in);
    void SetH(Eigen::MatrixXd H_in);
    void SetR(Eigen::MatrixXd R_in);

    void AddMeasurement(const tf::StampedTransform& meas);

    void AddMeasurement(const tf::StampedTransform& meas, const Eigen::MatrixXd& cov);

    void Prediction();
    void KFUpdate(Eigen::Vector2d z);
    void EKFUpdate(Eigen::VectorXd z);
    Eigen::VectorXd GetX();

    
private:
    void CalculateJacobianMatrix();

    bool is_initialized_;

    Eigen::Vector4d x_;
    Eigen::MatrixXd F_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd H_;
    Eigen::MatrixXd R_;
    Eigen::VectorXd y_;
    Eigen::MatrixXd S_;
    Eigen::MatrixXd K_;
    Eigen::MatrixXd I_;

    tf::Transformer transformer_;
};
}

#endif