#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "tf_broadcaster");
    ros::NodeHandle nh;
    ros::Rate rate(100);
    
    tf::TransformBroadcaster br;
    tf::Transform transform;
    
    while (ros::ok()) {
        // 发布 map->odom 变换
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
        
        // 发布 odom->base_link 变换
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
        
        rate.sleep();
    }
    
    return 0;
} 