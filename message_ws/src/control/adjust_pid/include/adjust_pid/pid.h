#ifndef __PID_H
#define __PID_H

class PID{
public:
    double PID_AngleCalc(float reference,float feedback);
    double PID_SpeedCalc(float reference,float feedback);

    double angle_kp;
    double angle_ki;
    double angle_kd;
    double speed_kp;
    double speed_ki;
    double speed_kd;

    double maxIntegral;
    double maxOutput;
private:
    double lastError_;
    double error_;
    double integral_;

};
#endif
