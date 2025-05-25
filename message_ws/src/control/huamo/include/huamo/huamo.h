#ifndef _HUAMO_H_
#define _HUAMO_H_

#include <vector>

float speed_huamo(const float now_v,const float expect_v);
float yaw_huamo(const float forward,const float now_yaw,const float expect_yaw,const float w,const float ye);
std::vector<float> remote_data_deal(const float forward, const float turn);

#endif