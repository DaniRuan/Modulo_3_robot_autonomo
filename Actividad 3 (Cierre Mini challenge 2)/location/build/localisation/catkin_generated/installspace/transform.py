#!/usr/bin/env python3
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

class Cordinate_transform():

    def __init__(self):
        rospy.init_node('cordinate_transform')

        self.x = 0
        self.y = 0
        self.orientation = [0.0, 0.0, 0.0, 0.0]

        rospy.Subscriber('/odom', Odometry, self.odom_cb)

        tf_br = tf2_ros.TransformBroadcaster()

        r = rospy.Rate(50)

        while not rospy.is_shutdown():
            t = TransformStamped()  
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "origin"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0

            #orientracion como cuaternion
            t.transform.rotation.x = self.orientation[0]
            t.transform.rotation.y = self.orientation[1]
            t.transform.rotation.z = self.orientation[2]
            t.transform.rotation.w = self.orientation[3]

            tf_br.sendTransform(t)

            r.sleep()


    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.orientation = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]


if __name__ == '__main__':
    Cordinate_transform()