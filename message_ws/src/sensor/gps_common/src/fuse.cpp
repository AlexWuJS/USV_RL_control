#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Imu.h>

double origin_x = 0.0;
double origin_y = 0.0;
double origin_z = 0.0;
double last_x = 0.0;
double last_y = 0.0;
double last_z = 0.0;

ros::Publisher odom_pub;
nav_msgs::Odometry odom_local;
ros::CallbackQueue myQueue;

// void callback(const nav_msgs::Odometry::ConstPtr& gps_odom, const sensor_msgs::Imu::ConstPtr& imu_data){
//     odom_local.pose.pose.position = gps_odom->pose.pose.position;
//     odom_local.pose.pose.orientation = imu_data->orientation;

//     odom_pub.publish(odom_local);
// }

void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr& gps_odom) {
    // 处理gpsOdom
    // 如果是第一次执行，以当前UTM坐标为odom的原点
    if (origin_x == 0.0 && origin_y == 0.0 && origin_z == 0.0) {
        origin_x = gps_odom->pose.pose.position.x;
        origin_y = gps_odom->pose.pose.position.y;
        origin_z = gps_odom->pose.pose.position.z;

        double dx = 0.0; 
        double dy = 0.0; 
        double dz = 0.0; 
    }
    else{ 
    // 根据UTM坐标更新，计算局部位置变化--
    double dx = gps_odom->pose.pose.position.x - origin_x; // Change in easting
    double dy = gps_odom->pose.pose.position.y - origin_y; // Change in northing
    double dz = gps_odom->pose.pose.position.z - origin_z; // Change in altitude

    // 更新局部里程计--
    odom_local.pose.pose.position.x = dx;
    odom_local.pose.pose.position.y = dy;
    odom_local.pose.pose.position.z = dz;

    odom_local.header = gps_odom->header;
    }
    
    // 保留本次UTM坐标位置，以便下次循环时计算
    last_x = gps_odom->pose.pose.position.x;
    last_y = gps_odom->pose.pose.position.y;
    last_z = gps_odom->pose.pose.position.z;

}
void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_data) {
    // 处理IMU数据:直接读取四元数
    odom_local.pose.pose.orientation = imu_data->orientation;
}
void combinedCallback() {
    myQueue.callAvailable();
    odom_pub.publish(odom_local);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "odom_processor");
    ros::NodeHandle nh;

    void callback(const nav_msgs::Odometry::ConstPtr& gps_odom, const sensor_msgs::Imu::ConstPtr& imu_data);
    ros::Subscriber gps_odom_sub = nh.subscribe("/gpsOdom", 10, gpsOdomCallback);
    ros::Subscriber imu_sub = nh.subscribe("/imu/data", 10, imuCallback);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom_local", 1);

    // 循环调用自定义回调函数
    ros::Rate loop_rate(10);        // 每秒10次
    while (ros::ok()) {
        combinedCallback();
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}
