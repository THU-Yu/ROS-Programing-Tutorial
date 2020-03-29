#include "ros/ros.h"
#include "std_msgs/String.h"
#include "husky_rcl_controller/HuskyRosApplication.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "husky_rcl_controller");
    ros::NodeHandle nodeHandle("~");

    husky_rcl_controller::HuskyRosApplication HuskyRosApplication(nodeHandle);
    //Wait for topic/service callback.
    ros::spin();
    
    return 0;
}