#include <iostream>
#include "my_ekf/kalman_filter.hpp"

namespace my_ekf_package
{
KalmanFilter::KalmanFilter()
{
    is_initialized_ = false;
}

KalmanFilter::~KalmanFilter()
{

}

void KalmanFilter::Initialization(Eigen::Vector4d x_in)
{
    x_ = x_in;

    std::cout << "x_: " << x_(0) << " " << x_(1) << " " << x_(2) << " " << " " << x_(3) << std::endl;
}

void KalmanFilter::Initialize(const tf::Transform& transform, const ros::Time& time)
{

}

bool KalmanFilter::IsInitialized()
{
    return is_initialized_;
}

void KalmanFilter::SetF(Eigen::MatrixXd F_in)
{
    F_ = F_in;
    // std::cout << "F_: " << std::endl << F_ << std::endl;
}

void KalmanFilter::SetP(Eigen::MatrixXd P_in)
{
    P_ = P_in;
    // std::cout << "before_kf P_: " << std::endl << P_ << std::endl;
}

void KalmanFilter::SetQ(Eigen::MatrixXd Q_in)
{
    Q_ = Q_in;
}

void KalmanFilter::SetH(Eigen::MatrixXd H_in)
{
    H_ = H_in;
}

void KalmanFilter::SetR(Eigen::MatrixXd R_in)
{
    R_ = R_in;
}

void KalmanFilter::AddMeasurement(const tf::StampedTransform& meas)
{
    ROS_DEBUG("AddMeasurement from %s to %s:  (%f, %f, %f)  (%f, %f, %f, %f)",
              meas.frame_id_.c_str(), meas.child_frame_id_.c_str(),
              meas.getOrigin().x(), meas.getOrigin().y(), meas.getOrigin().z(),
              meas.getRotation().x(),  meas.getRotation().y(), 
              meas.getRotation().z(), meas.getRotation().w());
    transformer_.setTransform(meas);
}

void KalmanFilter::AddMeasurement(const tf::StampedTransform& meas, const Eigen::MatrixXd& cov)
{
    // for(unsigned int i = 0; i < cov.rows(); i++)
    // {
    //     if(cov(i, i) == 0){
    //         ROS_ERROR("Covariance specified for measurement on topic %s is zero", meas.child_frame_id_.c_str());
    //         return;
    //     }
    // }

    AddMeasurement(meas);
}

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

void KalmanFilter::KFUpdate(Eigen::Vector2d z){

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

    // std::cout << "KFUpdate_after x_: " << x_(0) << ", " << x_(1) << std::endl;

    // std::cout << "KFUpdate_after P_: " << std::endl << P_ << std::endl;


}

void KalmanFilter::EKFUpdate(Eigen::VectorXd z)
{
    double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
    double theta = atan2(x_(1), x_(0));
    double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
    Eigen::VectorXd h = Eigen::VectorXd(3);
    h << rho, theta, rho_dot;
    y_ = z - h;

    CalculateJacobianMatrix();

    S_ = H_ * P_ * H_.transpose() + R_;
    K_ = P_ * H_.transpose() * S_.inverse();
    x_ = x_ + (K_ * y_);
    int x_size = x_.size();
    I_ = Eigen::MatrixXd::Identity(x_size, x_size);
    P_ = (I_ - K_ * H_) * P_;

}

Eigen::VectorXd KalmanFilter::GetX()
{
    // Eigen::VectorXd x_ = Eigen::VectorXd(4, 1);
    return x_;
}

void KalmanFilter::CalculateJacobianMatrix()
{
    Eigen::MatrixXd Hj(3, 4);

    // get state parameters
    float px = x_(0);
    float py = x_(1);
    float vx = x_(2);
    float vy = x_(3);

    // pre-compute a set of terms to avoid repeated calculation
    float c1 = px * px + py * py;
    float c2 = sqrt(c1);
    float c3 = (c1 * c2);

    // Check division by zero
    if(fabs(c1) < 0.0001){
        H_ = Hj;
        return;
    }

    Hj << (px/c2), (py/c2), 0, 0,
         -(py/c1), (px/c1), 0, 0,
          py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
    H_ = Hj;
}
}

