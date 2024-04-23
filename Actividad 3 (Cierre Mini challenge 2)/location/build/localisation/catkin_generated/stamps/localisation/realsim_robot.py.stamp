
#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import numpy as np
from tf.transformations import quaternion_from_euler

class RealsimRobot(): 
    def __init__(self):
        rospy.init_node("puzzlebot_realsim")
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_cb)
        self.pose_sim_pub = rospy.Publisher("/pose_sim", PoseStamped, queue_size=1)
        self.wr_pub = rospy.Publisher("/wr", Float32, queue_size=1)
        self.wl_pub = rospy.Publisher("/wl", Float32, queue_size=1)

        self.r = 0.05
        self.L = 0.19
        self.dt = 0.02

        self.w = 0.0
        self.v = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose_stamped = PoseStamped()

        rate = rospy.Rate(int(1.0 / self.dt))

        while not rospy.is_shutdown():
            self.update_robot_pose(self.v, self.w)
            pose_stamped = self.get_pose_stamped(self.x, self.y, self.theta)
            [wl, wr] = self.get_wheel_speed()

            self.pose_sim_pub.publish(pose_stamped)
            self.wr_pub.publish(wr)
            self.wl_pub.publish(wl)  
            rate.sleep()

    def cmd_vel_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def get_wheel_speed(self):
        wl = ((2 * self.v - self.w * self.L) / (2 * self.r))
        wr = ((2 * self.v + self.w * self.L) / (2 * self.r))

        return [wl, wr]

    def get_pose_stamped(self, x, y, yaw):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "origin"
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y

        quat = quaternion_from_euler(0, 0, yaw)
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]

        return pose_stamped

    def update_robot_pose(self, v, w):
        self.x = self.x + v * np.cos(self.theta) * self.dt
        self.y = self.y + v * np.sin(self.theta) * self.dt
        self.theta = self.theta + w * self.dt

if __name__ == "__main__":
    RealsimRobot()  