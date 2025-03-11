#ifndef __PID_H
#define __PID_H


class PID {
public:
    double kp;
    double ki;
    double kd;
    double maxIntegral;
    double maxOutput;

    PID() = default;
    PID(double kp, double ki, double kd, double min_out, double max_out);
    virtual double update(float reference, float feedback);
    virtual ~PID() = default; // 虚析构函数

protected:
    double min_output_;
    double lastError_;
    double error_;
    double integral_;
};

class AnglePID : public PID {
public:
    AnglePID() = default;
    AnglePID(double kp, double ki, double kd, double min_out, double max_out);
    static double map(double angle_error);
    double update(float reference, float feedback) override;

};

#endif

