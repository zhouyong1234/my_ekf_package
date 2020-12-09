#ifndef MY_EKF_MEASUREMENT_HPP_
#define MY_EKF_MEASUREMENT_HPP_

#include "Eigen/Dense"

namespace my_ekf_package
{
class Measurement {
public:
  long long timestamp_;

  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  Eigen::VectorXd raw_measurements_;
};
}

#endif
