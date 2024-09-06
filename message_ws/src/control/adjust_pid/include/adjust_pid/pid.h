#ifndef __PID_H
#define __PID_H

class PID{
public:
    double PID_Calc(float reference,float feedback);

    double kp_u;
    double ki_u;
    double kd_u;
    double maxIntegral;
    double maxOutput;
private:
    double lastError_;
    double error_;
    double integral_;

};
#endif
