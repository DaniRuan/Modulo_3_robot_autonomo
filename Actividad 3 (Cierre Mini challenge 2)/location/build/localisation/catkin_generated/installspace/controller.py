
#!/usr/bin/env python
#open loop
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import numpy as np

class Trayectory():
    def __init__(self):
        rospy.init_node("controller_path", anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.on_shutdown(self.cleanup)

        # Definir la velocidad lineal y angular para moverse en forma de cuadrado
        self.linear_speed = 0.2  # Velocidad lineal [m/s]
        self.angular_speed = 0.5  # Velocidad angular [rad/s]

        # Definir la duración de cada movimiento
        self.linear_duration = 5.0  # Duración de movimiento lineal [s]
        self.angular_duration = 3.14  # Duración de giro [s]

        # Inicializar el mensaje Twist
        self.vel_msg = Twist()

        # Ejecutar el movimiento del cuadrado
        self.run_square()

    def run_square(self):
        # Definir el bucle para moverse en forma de cuadrado
        while not rospy.is_shutdown():

            # Mover hacia adelante
            self.vel_msg.linear.x = self.linear_speed
            self.vel_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(self.vel_msg)
            rospy.sleep(self.linear_duration)

            # Girar
            self.vel_msg.linear.x = 0.0
            self.vel_msg.angular.z = self.angular_speed
            self.cmd_vel_pub.publish(self.vel_msg)
            rospy.sleep(self.angular_duration)

    def cleanup(self):
        self.vel_msg.linear.x = 0.0
        self.vel_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.vel_msg)
        rospy.loginfo("stop before shutdown")

if __name__ == '__main__':
    try:
        Trayectory()
    except rospy.ROSInterruptException:
        pass


'''
#!/usr/bin/env python
#close loop
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
import numpy as np
from nav_msgs.msg import Odometry

class Controller:
    def __init__(self):
        self.current_time = 0.0
        self.previous_time = 0.0
        self.first = True
        self.paso = 0

        # Variables para el almacenamiento de las posiciones del robot
        self.pose = Pose2D()
        self.pose.x = 0.0
        self.pose.y = 0.0
        self.pose.theta = 0.0

        # Variables para almacenar los errores de posicion y ángulo
        self.error_distancia = 0.0
        self.error_angular = 0.0

        # Initialize Twist message for robot speed
        self.speed = Twist()
        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0

        # Initialize PID controller gains
        self.kp_angular = 0.5
        self.ki_angular = 0.1
        self.kd_angular = 0.1

        self.kp_linear = 0.35
        self.ki_linear = 0.1
        self.kd_linear = 0.1

        #Variables para cada constante del PID
        self.P_linear = 0.0
        self.I_linear = 0.0
        self.D_linear = 0.0

        self.P_angular = 0.0
        self.I_angular = 0.0
        self.D_angular = 0.0

        # Previous errors for PID control
        self.prev_error_angular = 0.0
        self.prev_error_linear = 0.0

        # Initialize ROS node and publishers/subscribers
        rospy.init_node("controller")
        rospy.Subscriber("/odom", Odometry, self.callback_odom)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.input_path_x = rospy.get_param("/trayectoria/path_x")
        self.input_path_y = rospy.get_param("/trayectoria/path_y")
        self.rate = rospy.Rate(rospy.get_param("~node_rate", 100))

    def callback_odom(self, msg):
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        # Note: Consider using quaternion to obtain accurate orientation
        self.pose.theta = msg.pose.pose.orientation.z

    def wrap_theta(self, theta):
        result = np.fmod((theta + np.pi), (2 * np.pi))
        if result < 0:
            result += 2 * np.pi
        return result - np.pi

    def calculate_errors(self):
        self.error_angular = self.wrap_theta(np.arctan2(self.input_path_y[self.paso] - self.pose.y,
                                                         self.input_path_x[self.paso] - self.pose.x) - self.pose.theta)
        self.error_distancia = np.sqrt(np.square(self.input_path_x[self.paso] - self.pose.x) +
                                       np.square(self.input_path_y[self.paso] - self.pose.y))

    def run(self):
        while not rospy.is_shutdown():
            if self.first:
                self.current_time = rospy.get_time()
                self.previous_time = rospy.get_time()
                self.first = False
            else:
                self.current_time = rospy.get_time()
                dt = (self.current_time - self.previous_time)
                self.previous_time = self.current_time

                self.calculate_errors()

                # Controlador PID para la velocidad angular
                self.P_angular = self.kp_angular * self.error_angular
                self.I_angular += self.error_angular * dt
                self.D_angular = (self.error_angular - self.prev_error_angular) / dt
                self.prev_error_angular = self.error_angular

                self.respuesta_angular = self.P_angular + self.ki_angular * self.I_angular + self.kd_angular * self.D_angular

                # Controlador PID para la velocidad lineal
                self.P_linear = self.kp_linear * self.error_distancia
                self.I_linear += self.error_distancia * dt
                self.D_linear = (self.error_distancia - self.prev_error_linear) / dt
                self.prev_error_linear = self.error_distancia

                self.respuesta_linear = self.P_linear + self.ki_linear * self.I_linear + self.kd_linear * self.D_linear

                if self.error_distancia < 0.1:
                    self.error_distancia = 0
                    self.paso += 1
                    if self.paso >= len(self.input_path_x):
                        rospy.loginfo("Se completó la trayectoria.")
                        break

                # Publicar las velocidades controladas
                self.speed.linear.x = self.respuesta_linear
                self.speed.angular.z = self.respuesta_angular
                self.vel_pub.publish(self.speed)
                self.rate.sleep()

if __name__ == "__main__":
    Controller = Controller()
    try:
        Controller.run()
    except rospy.ROSInterruptException:
        pass
'''