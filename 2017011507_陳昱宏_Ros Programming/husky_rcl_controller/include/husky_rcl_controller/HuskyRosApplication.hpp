#ifndef _HUSKYROSAPPLICATION_H_
#define _HUSKYROSAPPLICATION_H_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "husky_rcl_controller/HuskyController.hpp"
#include "husky_rcl_controller_srv/husky_rcl_controller_srv.h"

namespace husky_rcl_controller{
    class HuskyRosApplication
    {
    public:
        HuskyRosApplication(ros::NodeHandle n);
        bool serviceCallback(husky_rcl_controller_srv::husky_rcl_controller_srv::Request& request,
                husky_rcl_controller_srv::husky_rcl_controller_srv::Response& response);
        void topicCallback(const nav_msgs::Odometry& message);
        void huskyCmdPublish();
    private:
        ros::Subscriber sub_;
        ros::Publisher pub_;
        HuskyController controller_;
        ros::ServiceServer server_;
        double posNow_[2] = {0};
        double yawNow_ = 0;
        double targetX_ = 0;
        double targetY_ = 0;
        double targetOrientationZ_ = 0;
        double linear_x = 0;
        double angular_z = 0;
    };
}
#endif