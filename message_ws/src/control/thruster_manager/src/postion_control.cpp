#include <ros/ros.h>
#include <vector>
#include <tf2/utils.h>
#include <control/Motor.h>
#include <nav_msgs/Odometry.h>
#include "common.h"


/**
  * @brief  角度映射到-180~180度的范围。
  * @param  angle: 映射前的角度。
  * @retval map_angle：映射前的角度。
  */
double angleMap(double angle)
{
    double map_angle;
    if (angle > 180 && angle < 360)
        map_angle = angle - 360;
    else if (angle < -180 && angle > -360)
        map_angle = angle + 360;
    else
        map_angle = angle;

    return map_angle;
}

class PositionPID{
public:
    double angle_kp;
    double angle_ki;
    double angle_kd;
    double speed_kp;
    double speed_ki;
    double speed_kd;
    double position_kp;
    double position_ki;
    double position_kd;
    
    double maxIntegral;
    double maxOutput;

    double AngleCalc(float reference,float feedback){
        double output;
        _lastError=_error;

        /* 角度误差映射 */
        _error = reference - feedback;
        if (_error > 180 && _error < 360)
            _error -= 360;
        else if (_error < -180 && _error > -360)
            _error += 360;
            
        float dout=(_error-_lastError)*angle_kd;
        
        // ROS_WARN("_lastError:%f,_error:%f,kd:%f,angle_kd:%f",_lastError,_error,dout,angle_kd);
        
        float pout=_error*angle_kp;
        
        _integral+=_error*angle_ki;
        
        if(_integral > maxIntegral) _integral=maxIntegral;
        else if(_integral < -maxIntegral) _integral=-maxIntegral;
        
        output=pout+dout+_integral;
        
        if(output > maxOutput) output=maxOutput;
        else if(output < -maxOutput) output=-maxOutput;

        return output;
    }

    double PositionCalc(float reference,float feedback)
    {
        double output, error, integral;
        static double lastError;

        lastError=error;
        
        error = reference - feedback;

        float dout=(error-lastError)*position_kd;
        
        float pout=error*position_kp;
        
        integral+=error*position_ki;
        
        if(integral > maxIntegral) integral=maxIntegral;
        else if(integral < -maxIntegral) integral=-maxIntegral;
        
        output=pout+dout+integral;
        
        if(output > maxOutput) output=maxOutput;
        else if(output < -maxOutput) output=-maxOutput;

        return output;
    }
private:
    double _lastError;
    double _error;
    double _integral;
};

class PositionControl{
public:
    PositionPID position_pid;

    PositionControl(){
        ros::NodeHandle nh("~");

        nh.param("angle_kp", position_pid.angle_kp, 20.0);
        nh.param("angle_ki", position_pid.angle_ki, 0.0);
        nh.param("angle_kd", position_pid.angle_kd, 125.0);
        nh.param("position_kp", position_pid.position_kp, -50.0);
        nh.param("position_ki", position_pid.position_ki, 0.0);
        nh.param("position_kd", position_pid.position_kd, 0.0);
        nh.param("maxIntegral", position_pid.maxIntegral, 800.0);
        nh.param("maxOutput", position_pid.maxOutput, 1800.0);

    }
    
    std::vector<int> getThrusterPwm(double desired_angle, double heading_angle, double desired_distance, double boat_distance){
        double position_pid_output, angle_pid_output;

        if (boat_distance - desired_distance < 1.5)
        {
            /* 如果目标点在船身后,那么后退 */
            if ((desired_angle < angleMap(heading_angle - 90)) && (desired_angle > angleMap(heading_angle + 90)))
            {
                if (desired_angle != 0)
                {
                    desired_angle = angleMap(desired_angle - 180 * desired_angle/abs(desired_angle));
                }
                boat_distance = -boat_distance;
            }
        }
        /* 输出差速值 */
        position_pid_output = position_pid.PositionCalc(desired_distance, boat_distance); 
        angle_pid_output = position_pid.AngleCalc(desired_angle, heading_angle); 

        // if (boat_distance < 0.2 && boat_distance > -0.2)
        // {
        //     _left_thruster_pwm = THRUSTER_MID_PWM;
        //     _right_thruster_pwm = THRUSTER_MID_PWM;
        // }
        // else
        // {
            _left_thruster_pwm = THRUSTER_MID_PWM - angle_pid_output * 0.5 + position_pid_output * 0.5;
            _right_thruster_pwm = THRUSTER_MID_PWM + angle_pid_output * 0.5 + position_pid_output * 0.5;
        // }


        /* 越界判断 */
        if (_left_thruster_pwm > THRUSTER_MAX_PWM)
            _left_thruster_pwm = THRUSTER_MAX_PWM;
        else if (_left_thruster_pwm < THRUSTER_MIN_PWM)
            _left_thruster_pwm = THRUSTER_MIN_PWM;

        if (_right_thruster_pwm > THRUSTER_MAX_PWM)
            _right_thruster_pwm = THRUSTER_MAX_PWM;
        else if (_right_thruster_pwm < THRUSTER_MIN_PWM)
            _right_thruster_pwm = THRUSTER_MIN_PWM;
      
        return {_left_thruster_pwm, _right_thruster_pwm};
    }
private:
    geometry_msgs::Point _boat_position;  
    int _left_thruster_pwm;
    int _right_thruster_pwm;
};
