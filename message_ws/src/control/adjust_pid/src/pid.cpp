#include <iostream>
using namespace std;
#include "ros/ros.h"
#include "adjust_pid/pid.h"

double PID::PID_AngleCalc(float reference,float feedback)
{
    double output;
    lastError_=error_;

    /* 角度误差映射 */
    error_ = reference - feedback;
    if (error_ > 180 && error_ < 360)
        error_ -= 360;
    else if (error_ < -180 && error_ > -360)
        error_ += 360;
        
    float dout=(error_-lastError_)*angle_kd;
    
    // ROS_WARN("lastError_:%f,error_:%f,kd:%f,angle_kd:%f",lastError_,error_,dout,angle_kd);
    
    float pout=error_*angle_kp;
    
    integral_+=error_*angle_ki;
    
    if(integral_ > maxIntegral) integral_=maxIntegral;
    else if(integral_ < -maxIntegral) integral_=-maxIntegral;
    
    output=pout+dout+integral_;
    
    if(output > maxOutput) output=maxOutput;
    else if(output < -maxOutput) output=-maxOutput;

    return output;
}

double PID::PID_SpeedCalc(float reference,float feedback)
{
    double output, error, integral;
    static double lastError;

    lastError=error;
    
    error = reference - feedback;

    float dout=(error-lastError)*speed_kd;
    
    float pout=error*speed_kp;
    
    integral+=error*speed_ki;
    
    if(integral > maxIntegral) integral=maxIntegral;
    else if(integral < -maxIntegral) integral=-maxIntegral;
    
    output=pout+dout+integral;
    
    if(output > maxOutput) output=maxOutput;
    else if(output < -maxOutput) output=-maxOutput;

    return output;
}
