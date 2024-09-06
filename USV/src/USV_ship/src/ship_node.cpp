#include "ship_node.h"
#include <iostream>

using namespace std;

unsigned char* double_to_bytes(const float data)
{
      unsigned char* p = (unsigned char*)malloc(4);
      if (p == NULL)
      {
            printf("malloc failure\r\n");
      }
      memset(p, 0, 4);
      memcpy(p,&data,4);
      return p;
}


//主函数
//功能：调用构造函数并初始化
int main(int argc,char** argv)
{   
    ros::init(argc,argv,"ship_node");
    ROS_INFO("Ship is Starting");

    Ship ship;

    return 0;
}


//构造函数-船的初始化
Ship::Ship()
{
    ros::NodeHandle nh_;
    point_sub_ = nh_.subscribe<nav_msgs::Path>("move_base/TebLocalPlannerROS/local_plan",10,&Ship::cmdVelCallback,this);
    //机器人启动完成提示信息
    ROS_INFO("Ship initialization completed, is Running!");
    ros::spin();
    
}

Ship::~Ship()
{
    
    //提示信息
    ROS_INFO("Ship shutting down."); 
}


void Ship::cmdVelCallback(const nav_msgs::Path::ConstPtr& msg)
{
    float data[2]={0};
    bool sendtodata=false;
    for(const auto& poseStamped : msg->poses)
    {
      geometry_msgs::Pose pose = poseStamped.pose;
      geometry_msgs::Point position = pose.position;
    
      data[0] = position.x;
      data[1] = position.y;
      sendtodata = true;
      cout<<"data[0]="<<data[0]<<endl;
      cout<<"data[1]="<<data[1]<<endl;
      struct termios usart_cfg;
      int sent_temp_data_int[4];//数组大小
      float sent_temp_data_double[2];
      int ret , fd=0 , i;
      char sent_data[50];
      unsigned char* temp_data;


      fd = open("/dev/ttyUSB2",O_RDWR|O_NOCTTY);
      if(fd < 0)
      {
            printf("open error\r\n");
      }
            /*设置串口*/
      cfmakeraw(&usart_cfg);
      usart_cfg.c_lflag &= ~CSIZE;
      usart_cfg.c_cflag |= (CS8 |CREAD|CLOCAL);
      usart_cfg.c_cflag &= ~PARENB;

      usart_cfg.c_iflag &= ~(INLCR | ICRNL); //不要回车和换行转换
      usart_cfg.c_iflag &= ~(IXON | IXOFF | IXANY); //不要软件流控制
      usart_cfg.c_oflag &= ~OPOST;
      usart_cfg.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); //原始模式

      cfsetspeed(&usart_cfg,B115200);

      /*设置即时生效*/
      ret = tcsetattr(fd,TCSADRAIN, &usart_cfg);
      cout << "set done" << endl;

      while(sendtodata)
      {
            sent_temp_data_double[0] = data[0];
            sent_temp_data_double[1] = data[1];
            // sent_temp_data_double[0] = 1.0;
            // sent_temp_data_double[1] = 0.0;

            sent_data[0] = '$';
            sent_data[1] = '$';
            for(i = 0 ; i < 2 ; i++) 
            {
                  temp_data = double_to_bytes(sent_temp_data_double[i]);
                  memcpy(&sent_data[2]+4*i,temp_data,4);
            } 
                  
            sent_data[47] = '0';
            sent_data[48] = '\r';
            sent_data[49] = '\n';
            ret = write(fd , sent_data,sizeof(sent_data)*sizeof(char));
            sendtodata = false;
            if(ret == -1)
            {
            cout << " sent data to 32 error : " << strerror(errno) << endl;
            }
            sleep(1);
                  // memset(&sent_data,0,sizeof(sent_data)*sizeof(char));
                  // memset(&sent_temp_data_int,0,sizeof(sent_temp_data_int)*sizeof(int));
                  // memset(&sent_temp_data_double,0,sizeof(sent_temp_data_double)*sizeof(float));
      }

      close(fd);
    }
      

}
