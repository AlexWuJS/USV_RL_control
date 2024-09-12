#include <iostream>
using namespace std;
#include "ros/ros.h"
#include "adjust_pid/pid.h"

double PID::PID_Calc(float reference,float feedback)
{
    double output;
    lastError_=error_;
    error_=reference-feedback;
    
        ROS_INFO("reference:%f, feedback:%f", reference, feedback);
        
    float dout=(error_-lastError_)*kd_u;
    
    float pout=error_*kp_u;
    
    integral_+=error_*ki_u;
    
    if(integral_ > maxIntegral) integral_=maxIntegral;
    else if(integral_ < -maxIntegral) integral_=-maxIntegral;
    
    output=pout+dout+integral_;
    
    if(output > maxOutput) output=maxOutput;
    else if(output < -maxOutput) output=-maxOutput;

    return output;
}
