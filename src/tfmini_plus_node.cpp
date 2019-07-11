#include <string>
#include <vector>
#include <memory>

#include "ros/ros.h"
#include "serial/serial.h"
// #include "tfm_driver/tfmData.h"
#include "tfmini_plus.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tfmini_plus_driver");

    ros::NodeHandle nh("~");


    ros::Rate loop(10);
    while (ros::Time::now().toSec()==0 && ros::ok()) {
        ROS_INFO("wait for /clock");
        loop.sleep();
    }

    std::shared_ptr<TF_mini_plus> tf_mini_plus;

    tf_mini_plus.reset(new TF_mini_plus(nh));

    // ros::Rate loop(1);
    // loop.sleep();

    ros::spin();
}