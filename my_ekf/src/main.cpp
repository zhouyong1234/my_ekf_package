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

    ros::Rate rate(10);
    while(ros::ok())
    {
        ros::spinOnce();

        fuser.run();

        rate.sleep();
    }

    // ros::spin();
    return 0;

}