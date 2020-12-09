#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include "Eigen/Eigen"
#include "my_ekf/measurement.h"
#include "my_ekf/data_fusion.hpp"
#include "my_ekf/kalman_filter.hpp"

using namespace my_ekf_package;

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "my_ekf_node");

    DataFusion fuser;

    ros::spin();

    return 0;

    // for (size_t i = 0; i < measurement_list.size(); ++i) {
    //     // fuser.Process(measurement_list[i]);
    //     Eigen::Vector4d x_out = fuser.kf_.GetX();

    //     std::cout << "x " << x_out(0)
    //               << " y " << x_out(1)
    //               << " vx " << x_out(2)
    //               << " vy " << x_out(3) 
    //               << std::endl;
    // }
}