#!/usr/bin/env python3

# Adapted from move_square from lab class
# rosrun assignment src/task1/move_figure_eight.py

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import sqrt, pow, pi

class Eight:
    def callback_function(self, odom_data):
        # obtain the orientation and position co-ords:
        or_x = odom_data.pose.pose.orientation.x
        or_y = odom_data.pose.pose.orientation.y
        or_z = odom_data.pose.pose.orientation.z
        or_w = odom_data.pose.pose.orientation.w
        pos_x = odom_data.pose.pose.position.x
        pos_y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch & yaw (theta_x, theta_y, theta_z):
        (roll, pitch, yaw) = euler_from_quaternion([or_x, or_y, or_z, or_w], 'sxyz')
        
        self.x = pos_x
        self.y = pos_y
        self.theta_z = yaw 

        if self.startup:
            self.startup = False
            self.x0 = self.x
            self.y0 = self.y
            self.theta_z0 = self.theta_z

    def __init__(self):
        node_name = "move_figure_eight"
        
        self.startup = True

        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('odom', Odometry, self.callback_function)

        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10) # hz

        self.x = 0.0
        self.y = 0.0
        self.theta_z = 0.0
        self.x0 = 0.0
        self.y0 = 0.0
        self.theta_z0 = 0.0
        
        self.vel = Twist()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)

        rospy.loginfo(f"the {node_name} node has been initialised...")

    def shutdownhook(self):
        self.pub.publish(Twist())
        self.ctrl_c = True
    
    def print_stuff(self, a_message):
        print(a_message)
        print(f"current odometry: x={self.x:.3f} [m] , y={self.y:.3f} [m] , yaw={(self.theta_z * (180 / pi)):.3f} [degrees]")
        print("z = " + str(self.theta_z))
        print("z0 = " + str(self.theta_z0))

    def main_loop(self):
        status = ""
        direction = "anticlockwise"
        self.vel.angular.z = -0.2
        self.vel.linear.x = 0.1
        wait = 0
        stop = 0

        while not self.ctrl_c:
            print(direction)
            if self.startup:
                self.vel = Twist()
                status = "init"
            
            if self.theta_z0 == 0.0:
                self.theta_z0 = self.theta_z
            
            # check whether direction needs changing
            elif self.theta_z0 + -0.01 > self.theta_z >= self.theta_z0 + (-0.05) and wait > 50 and stop == 0:
                # If the robot has turned 360 degrees (in radians), change direction to complete the figure 0
                # stop
                print("Changing direction!!!")
                self.vel = Twist()
                # reset direction
                self.theta_z0 = self.theta_z
                status = ""

                if direction == "anticlockwise":
                    # switch to clockwise
                    print("Changing direction to clockwise")
                    direction = "clockwise"
                    self.vel.angular.z = 0.2
                    self.vel.linear.x = 0.1
                    wait = 0
                elif direction == "clockwise":
                    # stop moving after figure 8
                    print("Changing direction to anticlockwise")
                    direction = "anticlockwise"
                    self.vel = Twist()  
                    wait = 0
                    stop = 1

            wait+= 1

            self.pub.publish(self.vel)
            self.print_stuff(status)
            self.rate.sleep()

if __name__ == '__main__':
    moveeight_instance = Eight()
    try:
        moveeight_instance.main_loop()
    except rospy.ROSInterruptException:
        pass