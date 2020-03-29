#ifndef _HUSKYCONTROLLER_H_
#define _HUSKYCONTROLLER_H_
#include <ros/ros.h>
namespace husky_rcl_controller{
    class HuskyController
    {
    public:
        HuskyController();
        void updateOdom(double OdomX, double OdomY, double OdomYaw);
        void updateTarget(double TargetX, double TargetY, double TargetYaw);
        bool EndControl();
        void PIDControl(double& linear_x, double& angular_z);
        void SetParameter();
        ros::Time startTime;
        bool result_;
        double timeCost_;
        bool controling_;
    private:
        double targetX_, targetY_, targetYaw_;
        double odomX_, odomY_, odomYaw_;
        bool endControl_;
        double proportion_vel_,  intergral_vel_, derivative_vel_;
        double proportion_ang_,  intergral_ang_, derivative_ang_;
        double lastError_vel_, prevError_vel_, sumError_vel_;
        double lastError_ang_, prevError_ang_, sumError_ang_;
    };
}
#endif