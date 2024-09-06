
#include "sharedMemory.h"
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
// #include "devel/include/location/speed2D.h"

int main(){

      char *buf;
      std::string head_buf = "$XO";
      std::string end_buf = "\r\n";
      std::string res;
      std::string rtk_data[4];
      printf("rtk sharedmemory is ok !!");
      sharedMemory m_shm(0, "bin", 1, "bin1", 5555, 1024);
      m_shm.sharedMemoryInit(nullptr,0);
      int start_pos = 1 , end_pos = 0;
      std::string str;
      float f;

      while(1)
      {
            start_pos = 0;
            buf=m_shm.readData();
            res=buf;
            // std::cout << res << std::endl;
            while(res.find(head_buf,start_pos) != std::string::npos)
            {
                  res = res.substr(res.find(head_buf,start_pos),res.find(end_buf,start_pos) - res.find(head_buf,start_pos));
                  // std::cout << res << std::endl;
                  start_pos = 4;
                  // 找到逗号，以此为依据分割裁剪出一段string，并转为float输出
                  while(res.find(',',start_pos) != std::string::npos)
                  {
                        end_pos = res.find(',',start_pos);
                        str = res.substr(start_pos,end_pos-start_pos);
                        f = std::stof(str);
                        start_pos = end_pos + 1;
                        std::cout<< f << std::endl;   
                        
                  }
                  sleep(1);
            }
            
      }
      return 0;  
}
