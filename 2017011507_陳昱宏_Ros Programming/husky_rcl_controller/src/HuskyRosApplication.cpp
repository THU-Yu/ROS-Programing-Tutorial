#include "tf/transform_datatypes.h"
#include <geometry_msgs/Twist.h>
#include "husky_rcl_controller/HuskyRosApplication.hpp"

namespace husky_rcl_controller
{

// 构造函数
HuskyRosApplication::HuskyRosApplication(ros::NodeHandle n)
{
    // 初始化订阅、发布、服务器和控制器
    this->sub_ = n.subscribe("/odometry/filtered", 1000, &HuskyRosApplication::topicCallback, this);
    this->pub_ = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    this->server_ = n.advertiseService("husky_controller_server", &HuskyRosApplication::serviceCallback, this);
    this->controller_ = HuskyController();
}

// topic回调函数
void HuskyRosApplication::topicCallback(const nav_msgs::Odometry& message)
{
    posNow_[0] = message.pose.pose.position.x;
    posNow_[1] = message.pose.pose.position.y;
    // 四元数转欧拉角
    tf::Quaternion quat;
    tf::quaternionMsgToTF(message.pose.pose.orientation, quat);
    double roll, pitch;
    tf::Matrix3x3(quat).getRPY(roll,pitch,yawNow_);
    // 更新控制器当前位置
    controller_.updateOdom(posNow_[0], posNow_[1], yawNow_);
}

// service回调函数
bool HuskyRosApplication::serviceCallback(husky_rcl_controller_srv::husky_rcl_controller_srv::Request& request,
                    husky_rcl_controller_srv::husky_rcl_controller_srv::Response& response)
{
    // 如果有其他命令正在控制，忽略新命令
    if (this->controller_.controling_)
    {
        ROS_INFO("Please Wait For Other Control Mission Completed!");
        return true;
    }
    // 更新控制器的当前目标
    controller_.updateTarget(request.targetX, request.targetY, request.targetOrientationZ);
    controller_.SetParameter();
    // 持续进行PID控制
    while(!controller_.EndControl())
    {
        huskyCmdPublish();
        ros::spinOnce();
    }

    // 结束控制更新状态
    this->controller_.controling_ = false;
    // 读取控制结果
    response.result = controller_.result_;
    response.timeCost = controller_.timeCost_;
    // 根据不同结果显示不同提示信息
    if (response.result)
    {
        ROS_INFO("Result: Finished\nTime Cost: %lf",response.timeCost);
    }
    else
    {
        ROS_INFO("Result: Failed\nTime Cost: %lf",response.timeCost);
    }
    return true;
}

// 发布函数
void HuskyRosApplication::huskyCmdPublish()
{
    // 根据PID控制结果发布线速度和转速的信息
    this->controller_.PIDControl(this->linear_x, this->angular_z);
    geometry_msgs::Twist msg;
    msg.linear.x = this->linear_x;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = this->angular_z;
    this->pub_.publish(msg);
}

}