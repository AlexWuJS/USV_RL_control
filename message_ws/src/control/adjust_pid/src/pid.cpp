#include <iostream>
using namespace std;
#include "ros/ros.h"
#include "adjust_pid/pid.h"

double PID::PID_Calc(float reference,float feedback)
{
    double output;
    lastError_=error_;

    /* 角度误差映射 */
    error_ = reference - feedback;
    if (error_ > 180 && error_ < 360)
        error_ -= 360;
    else if (error_ < -180 && error_ > -360)
        error_ += 360;
        
    float dout=(error_-lastError_)*kd_u;
    
    ROS_WARN("lastError_:%f,error_:%f,kd:%f,kd_u:%f",lastError_,error_,dout,kd_u);
    
    float pout=error_*kp_u;
    
    integral_+=error_*ki_u;
    
    if(integral_ > maxIntegral) integral_=maxIntegral;
    else if(integral_ < -maxIntegral) integral_=-maxIntegral;
    
    output=pout+dout+integral_;
    
    if(output > maxOutput) output=maxOutput;
    else if(output < -maxOutput) output=-maxOutput;

    return output;
}
