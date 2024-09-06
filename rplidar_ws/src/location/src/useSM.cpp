#include "sharedMemory.h"
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <location/speed2D.h> // 自定义消息头文件位置

int main(int argc, char **argv){

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

    ros::init(argc, argv, "RTKtopic_publisher");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<location::speed2D>("speed2D_topic", 10);
    ros::Rate loop_rate(20);

    while(ros::ok())
    {
        start_pos = 0;
        buf=m_shm.readData();
        res=buf;
        // std::cout << res << std::endl;
        while(res.find(head_buf,start_pos) != std::string::npos)
        {
            res = res.substr(res.find(head_buf,start_pos),res.find(end_buf,start_pos) - res.find(head_buf,start_pos));
            std::cout << res << std::endl;
            start_pos = 4;

            // 创建自定义消息对象并设置数据
            location::speed2D speed2D_msg;
            float f1,f2,f3;
            int count = 0;
            // 找到逗号，以此为依据分割裁剪出一段string，并转为float输出
            while(res.find(',',start_pos) != std::string::npos)
            {
                end_pos = res.find(',',start_pos);
                

                str = res.substr(start_pos,end_pos-start_pos);
                f = std::stof(str);
                start_pos = end_pos + 1;
                std::cout<< f << std::endl;

                //根据count值，将本段float赋给对应的速度
                count++;
                
                // std::cout<< count << std::endl;
                switch(count)
                {
                    case 1:
                        f1=f;
                        std::cout<<"f1 loaded"<<std::endl;
                        break;
                    case 2:
                        f2=f;
                        std::cout<<"f2 loaded"<<std::endl;
                        break;
                    case 3:
                        f3=f;
                        std::cout<<"f3 loaded"<<std::endl;
                        break;
                    default:
                        std::cout<<"---f counter error---"<<std::endl;
                }

            }
            count = 0;
            // 发布自定义消息
            speed2D_msg.header.stamp = ros::Time::now();
            speed2D_msg.header.frame_id = "RTK";
            speed2D_msg.linearX = f1;
            speed2D_msg.linearY = f2;
            speed2D_msg.angularZ = f3;
            
            pub.publish(speed2D_msg);
            
            loop_rate.sleep();

        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;  
}
// 生成的可执行SM文件默认在build/location下，要copy到devel/lib/location下，才能被rosrun找到