#ifndef _PWM_DATA_H_
#define _PWM_DATA_H_

#include <vector>

std::vector<float> remote_data_deal(const float forward, const float turn);
int lim(const int min,int data,const int max);
float float_lim(const float minimum, const float data, const float maximum);

#endif