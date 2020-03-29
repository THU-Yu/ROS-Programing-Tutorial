#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "husky_rcl_controller/HuskyController.hpp"

#include <stdio.h>
#include <math.h>

#define PI 3.14159265

namespace husky_rcl_controller
{
// 构造函数
HuskyController::HuskyController()
{
    // 初始化变量
    this->endControl_ = true;
    this->controling_ = false;
    this->odomX_ = 0;
    this->odomY_ = 0;
    this->odomYaw_ = 0;
    this->targetX_ = 0;
    this->targetY_ = 0;
    this->targetYaw_ = 0;
    // 设置PID控制器参数，角度使用PD控制器，速度用PD控制器
    this->proportion_vel_ = 0.1;
    this->proportion_ang_ = 0.4;
    this->intergral_vel_ = 0;
    this->intergral_ang_ = 0;
    this->derivative_vel_ = 0.1;
    this->derivative_ang_ = 0.4;
}

// 更新当前坐标与转向
void HuskyController::updateOdom(double OdomX, double OdomY, double OdomYaw)
{
    this->odomX_ = OdomX;
    this->odomY_ = OdomY;
    this->odomYaw_ = OdomYaw;
}

// 更新目标位置
void HuskyController::updateTarget(double TargetX, double TargetY, double TargetYaw)
{
    // 当endControl_为真时，才允许更新目标
    if (this->endControl_)
    {
        this->targetX_ = TargetX;
        this->targetY_ = TargetY;
        this->targetYaw_ = TargetYaw;
        this->startTime = ros::Time::now();
        this->endControl_ = false;
        this->controling_ = true;
    }
    
}

bool HuskyController::EndControl()
{
    return this->endControl_;
}

// 设置误差参数
void HuskyController::SetParameter()
{
    this->lastError_vel_ = 0;
    this->prevError_vel_ = 0;
    this->sumError_vel_ = 0;
    this->lastError_ang_ = 0;
    this->prevError_ang_ = 0;
    this->sumError_ang_ = 0;
}

// PID控制器 
void HuskyController::PIDControl(double& linear_x, double& angular_z)
{
    // 计算各类误差
    double targetAng, targetDistance;
    // 目标与自己的夹角
    targetAng = atan2((this->targetY_ - this->odomY_), (this->targetX_ - this->odomX_));
    // 目标与自己的直线距离
    targetDistance = sqrt(pow(this->targetX_ - this->odomX_, 2.0) + pow(this->targetY_ - this->odomY_, 2.0));
    // 采用增量式PID控制器，计算各类误差
    this->prevError_vel_ = this->lastError_vel_;
    this->lastError_vel_ = targetDistance;
    this->sumError_vel_ += this->lastError_vel_;
    this->prevError_ang_ = this->lastError_ang_;
    this->lastError_ang_ = targetAng - this->odomYaw_;
    this->sumError_ang_ += this->lastError_ang_;
    // 当角度误差较小时，代表不需要再旋转，可以开始慢慢加速
    // 反之要先慢慢减速保证连续，以低速先将角度调整好
    if (fabs(this->lastError_ang_) < 1e-1)
    {
        angular_z = 0;
        this->proportion_vel_ = this->proportion_vel_ + 0.1; // 此处加速值可调
        if (this->proportion_vel_ >= 0.5)
        {
            this->proportion_vel_ = 0.5;
        }
    }
    else
    {
        angular_z = (this->proportion_ang_ * this->lastError_ang_ + this->derivative_ang_ * (this->lastError_ang_ - this->prevError_ang_) + this->intergral_ang_ * this->sumError_ang_);
        this->proportion_vel_ = this->proportion_vel_ - 0.1;
        if (this->proportion_vel_ <= 0.1)
        {
            this->proportion_vel_ = 0.1;
        }
    }
    // 计算线速度
    linear_x = this->proportion_vel_ * this->lastError_vel_ + this->derivative_vel_ * (this->lastError_vel_ - this->prevError_vel_) + this->intergral_vel_ * this->sumError_vel_;
    
    // 结束终止条件，误差小于1e-2或是时间超过10秒
    if (this->lastError_vel_ < 1e-2)
    {
        this->endControl_ = true;
        this->result_ = true;
        ros::Duration d = ros::Time::now() - this->startTime;
        this->timeCost_  = (double)d.sec;
    }
    else if (ros::Time::now() - this->startTime >= ros::Duration(10))
    {
        this->endControl_ = true;
        this->result_ = false;
        ros::Duration d = ros::Time::now() - this->startTime;
        this->timeCost_  = (double)d.sec;
    }
}

} // namespace husky_rcl_controller
