import time
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Empty as EmptyMsg
import tf2_ros

rospy.init_node("hover_only", anonymous=True)
od = rospy.Publisher("/Odometry", Odometry, queue_size=1)
tf_br = tf2_ros.TransformBroadcaster()
hov = rospy.Publisher("/guidance/hover_set", EmptyMsg, queue_size=1)

odom = Odometry(); odom.header.frame_id = "camera_init"
odom.pose.pose.orientation.w = 1.0
boat = [0.0, 0.0]
def pub(dur):
    t_end = time.time() + dur
    while time.time() < t_end:
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position.x = boat[0]
        odom.pose.pose.position.y = boat[1]
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "camera_init"; t.child_frame_id = "body"
        t.transform.translation.x = boat[0]; t.transform.translation.y = boat[1]
        t.transform.rotation.w = 1.0
        tf_br.sendTransform(t)
        od.publish(odom)
        time.sleep(0.02)

pub(1.0)
hov.publish(EmptyMsg())
pub(1.0)
boat[0], boat[1] = 0.4, 0.2   # 漂 0.45 m
pub(3.0)
boat[0], boat[1] = 0.0, 0.0   # 回锚点
pub(3.0)
boat[0], boat[1] = -0.5, 0.3  # 反向漂
pub(3.0)
print("hover_only 场景执行完毕")
