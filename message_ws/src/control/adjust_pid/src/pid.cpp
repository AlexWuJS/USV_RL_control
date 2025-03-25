#include <iostream>
using namespace std;
#include "ros/ros.h"
#include "adjust_pid/pid.h"

PID::PID(double kp, double ki, double kd, double min_out, double max_out)
        : kp(kp), ki(ki), kd(kd), min_output_(min_out), maxOutput(max_out),
          lastError_(0.0), error_(0.0), integral_(0.0), maxIntegral(1000.0) {}

double PID::update(float reference,float feedback){
    double output;

    lastError_=error_;

    error_ = reference - feedback;

    float dout=(error_-lastError_)*kd;
    
    float pout=error_*kp;
    
    integral_+=error_*ki;
    
    if(integral_ > maxIntegral) integral_=maxIntegral;
    else if(integral_ < -maxIntegral) integral_=-maxIntegral;
    
    output=pout+dout+integral_;
    
    if(output > maxOutput) output=maxOutput;
    else if(output < -maxOutput) output=-maxOutput;

    return output;        
}


AnglePID::AnglePID(double kp, double ki, double kd, double min_out, double max_out)
        : PID(kp, ki, kd, min_out, max_out) {}

/**
  * @brief  角度映射到-180~180度的范围。
  * @param  angle: 映射前的角度。
  * @retval map_angle：映射前的角度。
  */
double AnglePID::map(double angle_error){
    if (angle_error > 180 && angle_error < 360)
        angle_error -= 360;
    else if (angle_error < -180 && angle_error > -360)
        angle_error += 360;
    
    return angle_error;
}

double AnglePID::update(float reference,float feedback){
    double output;

    lastError_=error_;
    error_ = reference - feedback;

    error_ = AnglePID::map(error_);

    ROS_INFO("reference:%f feedback:%f error: %f",reference,feedback, error_);

    float dout=(error_-lastError_)*kd;
    
    float pout=error_*kp;
    
    integral_+=error_*ki;
    
    if(integral_ > maxIntegral) integral_=maxIntegral;
    else if(integral_ < -maxIntegral) integral_=-maxIntegral;
    
    output=pout+dout+integral_;
    
    if(output > maxOutput) output=maxOutput;
    else if(output < -maxOutput) output=-maxOutput;

    return output;    
}

