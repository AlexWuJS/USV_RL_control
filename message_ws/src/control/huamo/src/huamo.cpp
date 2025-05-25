#include "huamo/huamo.h"

#include <iostream>
#include <cmath>

float A = 0.793023;
float B = 0.0236265;
float k1 = 1.7;
float k2 = 0.1;
float a1 = -0.66;
float a2 = 0.5;
float a3 = 3;
float b1 = 0.4;
float b2 = 0.1;

float yK[6] = {-0.046518829,   -0.022200586,  -0.016743531, 
               -0.019396446,   -0.021851526,       1};
float yT[6] = {1.102738,       0.516880736,     0.299512315, 
               0.32402933,     0.322889884,        1};
float yn3[6] = {-0.914695041,  -2.461098986,   -0.849048809,  
                -0.195679512,  0.27797399,    1};
float yc[6] = {6,     6,      6,      6,      6,       1};
float yk1[6] = {0.65,   0.65,    0.65,      0.65,      0.65,     1};
float yk2[6] = {1.3,   1.8,    1.5,    1.8,    1.1,     1};
float ya1[6] = {-0.58,  -0.5,   -0.5,   -0.55,   -0.55,    1};
float ya2[6] = {0.5,  0.6,    0.5,    0.5,    0.5,     1};
float ya3[6] = {3,     3,      3,      3,      3,       1};
float yb1[6] = {0.8,   0.7,    0.7,      0.8,      0.7,     1};
float yb2[6] = {0.15,   0.35,    0.35,   0.3,    0.35,    1};



float speed_huamo(const float now_v,const float expect_v)
{
    static float last_u = 0;
    static float last_x1_expect = 0;
    float u = 0;
    float x1_expect = expect_v;
    float x1_d_expect = 0;
    float x1 = now_v;

    float s = 10*(now_v - x1_expect);
    double reaching_law = k1*pow(std::abs(s),a1)*tanh(b1*s) + k2*pow(std::abs(s),a2)*asinh(b2*pow(s,a3));
    u = 1/B*x1 + A/B*x1_d_expect - A/B * (reaching_law);    

    u = u > 100? 100 : (u < 0? 0 : u);

    last_x1_expect = x1_expect;

    return u;
}

float yaw_huamo(const float forward,const float now_yaw,const float expect_yaw,const float w,const float ye)
{
    static bool get = false;
    static float last_ye = 0;
    static float last_u = 0;
    static float last_x1_expect = 0;
    static float last_x1_d_expect = 0;
    float u = 0;
    float x1_expect = expect_yaw > M_PI? (expect_yaw-2*M_PI) : (expect_yaw < -M_PI? (expect_yaw+2*M_PI) : expect_yaw);
   
    float x1 = now_yaw, x2 = w;
    x1 = x1 > M_PI? (x1-2*M_PI) : (x1 < -M_PI? (x1+2*M_PI) : x1);

    float x1_d_expect = 0;

    float x1_dd_expect = 0;

    float e = (x1 - x1_expect);

    e = e > M_PI?(e-2*M_PI) : (e < -M_PI? (e+2*M_PI) : e);
    last_ye = ye;

    int i = 0;
    float persent = 0;
    //权重
    float a1 = 1, a2 = 0;

    if(forward <= 5.1)
    {
        i = 0;
        a1 = 1;
        a2 = 0;
    }
    else if(5.1 < forward && forward < 13.9)
    {
        i = 0;
        persent = (forward - 5.1)/8.8;
        a1 = (1-persent);
        a2 = persent;
    }
    else if(13.9 <= forward && forward <= 16.5)
    {
        i = 1;
        a1 = 1;
        a2 = 0;
    }
    else if(16.5 < forward && forward < 28.5)
    {
        i = 1;
        persent = (forward - 16.5)/12.f;
        a1 = (1-persent);
        a2 = persent;
    }
    else if(28.5 <= forward && forward <= 32)
    {
        i = 2;
        a1 = 1;
        a2 = 0;
    }
    else if(32 < forward && forward < 47)
    {
        i = 2;
        persent = (forward - 32)/15;
        a1 = (1-persent);
        a2 = persent;
    }
    else if(47 <= forward && forward <= 53)
    {
        i = 3;
        a1 = 1;
        a2 = 0;
    }
    else if(53 < forward && forward < 77)
    {
        i = 3;
        persent = (forward - 53)/24;
        a1 = (1-persent);
        a2 = persent;
    }
    else if(77 <= forward && forward <= 100)
    {
        i = 4;
        a1 = 1;
        a2 = 0;
    }
    else
    {
        i = 4;
        a1 = 0;
        a2 = 0;
    }

    //第一个控制器
    float s1 = yc[i] * e*1.5 +(x2 - x1_d_expect);
    double reaching_law1 = yk1[i]*pow(std::abs(s1),ya1[i])*tanh(yb1[i]*s1) + yk2[i]*pow(std::abs(s1),ya2[i])*asinh(yb2[i]*pow(s1,ya3[i]));
    float u1 = yn3[i]/yK[i]*pow(x2,3) + (1/yK[i]-yc[i]*yT[i]/yK[i])*x2 + yc[i]*yT[i]/yK[i]*x1_d_expect+yT[i]/yK[i]*x1_dd_expect - yT[i]/yK[i]*(reaching_law1);
    u1 = u1 > 100? 100 : (u1 < -100? -100 : u1);

    //第二个控制器
    float s2 = yc[i+1]*e + (x2 - x1_d_expect);
    double reaching_law2 = yk1[i+1]*pow(std::abs(s2),ya1[i+1])*tanh(yb1[i+1]*s2) + yk2[i+1]*pow(std::abs(s2),ya2[i+1])*asinh(yb2[i+1]*pow(s2,ya3[i+1]));
    float u2 = yn3[i+1]/yK[i+1]*pow(x2,3) + (1/yK[i+1]-yc[i+1]*yT[i+1]/yK[i+1])*x2 + yc[i+1]*yT[i+1]/yK[i+1]*x1_d_expect+yT[i+1]/yK[i+1]*x1_dd_expect - yT[i+1]/yK[i+1]*(reaching_law2);
    u2 = u2 > 100 ? 100 : (u2 < -100? -100 : u2);

    u = a1 * u1 + a2 * u2;
    // std::cout << "expect_yaw:" << expect_yaw*180/M_PI << "\tyaw:" << x1*180/M_PI << "\te:" << e*180/M_PI << "\tu1:" << u1 << "\tu2:" << u2 << "\tu:" << u << std::endl;

    if(std::abs(u) <= 1.5)
        u = 0;

    last_x1_expect = x1_expect;
    last_x1_d_expect = x1_d_expect;

    return u;
}
