
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import quaternion_from_euler

class OdomPublisher():
    def __init__(self):
        rospy.init_node("odometry_publisher")
        rospy.Subscriber("/wl", Float32, self.wl_cb)
        rospy.Subscriber("/wr", Float32, self.wr_cb)

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=1)

        self.r = 0.05
        self.L = 0.19
        self.dt = 0.02
        self.w = 0.0
        self.v = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wl = 0.0
        self.wr = 0.0

        self.odom = Odometry()

        rate = rospy.Rate(int(1.0 / self.dt))

        while not rospy.is_shutdown():
            [v, w] = self.get_robot_vel(self.wl, self.wr)
            self.update_robot_pose(v, w)
            self.get_odometry()
            rate.sleep()

    def wl_cb(self, msg):
        self.wl = msg.data

    def wr_cb(self, msg):
        self.wr = msg.data

    def get_robot_vel(self, wl, wr):
        v = self.r * (wr + wl) / 2.0
        w = self.r * (wr - wl) / self.L

        return [v, w]

    def get_odometry(self):
        odom_quat = quaternion_from_euler(0, 0, self.theta)
        self.update_robot_pose(self.v, self.w) 
        self.odom.header.frame_id = "origin"
        self.odom.child_frame_id = "base_link"
        self.odom.header.stamp = rospy.Time.now()

        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.orientation.x = odom_quat[0]
        self.odom.pose.pose.orientation.y = odom_quat[1]
        self.odom.pose.pose.orientation.z = odom_quat[2]
        self.odom.pose.pose.orientation.w = odom_quat[3]

        self.odom.twist.twist.linear.x = self.v
        self.odom.twist.twist.angular.z = self.w 

        self.odom_pub.publish(self.odom)
        self.wl = 0
        self.wr = 0

    def update_robot_pose(self, v, w):
        self.x = self.x + v * np.cos(self.theta) * self.dt
        self.y = self.y + v * np.sin(self.theta) * self.dt
        self.theta = self.theta + w * self.dt

if __name__ == "__main__":
    OdomPublisher()
