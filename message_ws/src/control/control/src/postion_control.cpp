class PostionControl{
public:
    PostionControl(){
        ros::NodeHandle nh("~");

        nh.param("angle_kp", myPid.angle_kp, 20.0);
        nh.param("angle_ki", myPid.angle_ki, 0.0);
        nh.param("angle_kd", myPid.angle_kd, 125.0);
        nh.param("location_kp", myPid.location_kp, -50.0);
        nh.param("location_ki", myPid.location_ki, 0.0);
        nh.param("location_kd", myPid.location_kd, 0.0);
        nh.param("maxIntegral", maxIntegral, 800);
        nh.param("maxOutput", maxOutput, 1800);
    }
    
    vector<int> getThrusterPwm(double desired_angle, double heading_angle, double desired_distance, double boat_distance){
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
        location_pid_output = myPid.PID_LocationCalc(0, boat_distance); 
        angle_pid_output = myPid.PID_AngleCalc(desired_angle, heading_angle); 

        // if (boat_distance < 0.2 && boat_distance > -0.2)
        // {
        //     thruster_msg.left_thruster_pwm = THRUSTER_MID_PWM;
        //     thruster_msg.right_thruster_pwm = THRUSTER_MID_PWM;
        // }
        // else
        // {
            thruster_msg.left_thruster_pwm = THRUSTER_MID_PWM - angle_pid_output * 0.5 + location_pid_output * 0.5;
            thruster_msg.right_thruster_pwm = THRUSTER_MID_PWM + angle_pid_output * 0.5 + location_pid_output * 0.5;
        // }


        /* 越界判断 */
        if (thruster_msg.left_thruster_pwm > THRUSTER_MAX_PWM)
            thruster_msg.left_thruster_pwm = THRUSTER_MAX_PWM;
        else if (thruster_msg.left_thruster_pwm < THRUSTER_MIN_PWM)
            thruster_msg.left_thruster_pwm = THRUSTER_MIN_PWM;

        if (thruster_msg.right_thruster_pwm > THRUSTER_MAX_PWM)
            thruster_msg.right_thruster_pwm = THRUSTER_MAX_PWM;
        else if (thruster_msg.right_thruster_pwm < THRUSTER_MIN_PWM)
            thruster_msg.right_thruster_pwm = THRUSTER_MIN_PWM;
      
        return {thruster_msg.left_thruster_pwm, thruster_msg.right_thruster_pwm};
    }
private:

}