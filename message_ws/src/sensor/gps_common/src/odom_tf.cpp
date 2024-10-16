#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>

class OdomTfBroadcaster {
public:
    OdomTfBroadcaster() {
        sub_ = nh_.subscribe("odom_local", 1, &OdomTfBroadcaster::odomCallback, this);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 发布/odom_local到/base_footprint的tf变换
        geometry_msgs::TransformStamped odom_tf;
        odom_tf.header.stamp = msg->header.stamp;
        odom_tf.header.frame_id = "odom_local";
        odom_tf.child_frame_id = "base_footprint";
        odom_tf.transform.translation.x = msg->pose.pose.position.x;
        odom_tf.transform.translation.y = msg->pose.pose.position.y;
        odom_tf.transform.translation.z = msg->pose.pose.position.z;
        odom_tf.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_.sendTransform(odom_tf);

        // 发布/map到/odom_local的tf变换
        geometry_msgs::TransformStamped map_tf;
        map_tf.header.stamp = msg->header.stamp;
        map_tf.header.frame_id = "map";
        map_tf.child_frame_id = "odom_local";
        map_tf.transform.translation.x = 0.0;
        map_tf.transform.translation.y = 0.0;
        map_tf.transform.translation.z = 0.0;
        map_tf.transform.rotation.x = 0.0;
        map_tf.transform.rotation.y = 0.0;
        map_tf.transform.rotation.z = 0.0;
        map_tf.transform.rotation.w = 1.0;
        tf_broadcaster_.sendTransform(map_tf);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_tf_broadcaster");
    OdomTfBroadcaster odom_tf_broadcaster;
    ROS_INFO("----------odom_tf publishing----------");
    ros::spin();
    return 0;
}
