
#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

class JointsToPublish():
    def __init__(self):
        rospy.init_node("Joint_State_Publisher")
        self.wl = 0
        self.wr = 0
        self.r = 0.05
        self.L = 0.19
        self.dt = 0.02

        self.contJoints = JointState()
        self.init_joints()

        rospy.Subscriber("/wl", Float32, self.wl_cb)
        rospy.Subscriber("/wr", Float32, self.wr_cb)

        self.joint_pub_tf = rospy.Publisher("/joint_states", JointState, queue_size=1)

        self.loop_rate = rospy.Rate(int(1.0 / self.dt))
        rospy.on_shutdown(self.stop)
        self.run()

    def init_joints(self):
        self.contJoints.header.frame_id = "base_link"
        self.contJoints.header.stamp = rospy.Time.now()
        self.contJoints.name.extend(["left_wheel_joint", "right_wheel_joint"])
        self.contJoints.position.extend([0.0, 0.0])
        self.contJoints.velocity.extend([0.0, 0.0])
        self.contJoints.effort.extend([0.0, 0.0])

    def wl_cb(self, msg):
        self.wl = msg.data

    def wr_cb(self, msg):
        self.wr = msg.data

    def calculate_wheels_speed(self, v, w):
        theta_l = self.wl * self.dt  
        theta_r = self.wr * self.dt  
        return theta_l, theta_r

    def run(self):
        try:
            while not rospy.is_shutdown():
                theta_l, theta_r = self.calculate_wheels_speed(self.wl, self.wr)
                self.contJoints.header.stamp = rospy.Time.now()
                self.contJoints.position[0] += theta_r
                self.contJoints.position[1] += theta_l

                self.joint_pub_tf.publish(self.contJoints)

                self.loop_rate.sleep()

        except rospy.ROSInterruptException:
            pass

    def stop(self):
        rospy.loginfo("Stop the node")

if __name__ == "__main__":
    JointsToPublish()




