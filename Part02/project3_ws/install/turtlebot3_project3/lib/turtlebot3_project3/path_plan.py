#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import time

class move(Node):

    def __init__(self,lin_vel,ang_vel,t):
        super().__init__('keyboard_control_node')
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        self.t = t
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.settings = termios.tcgetattr(sys.stdin)


    def publish_moves(self):
        velocity_message = Twist()
        linear_vel=0.0
        angular_vel=0.0
        velocity_message.linear.x = linear_vel
        velocity_message.angular.z = angular_vel
        self.cmd_vel_pub.publish(velocity_message)

        for i in range(len(self.lin_vel)-1):
            # Publish the twist message
            lin_vel = (float(self.lin_vel[i]))
            ang_vel = (float(self.ang_vel[i]))*-1
            print('lin vel: ', lin_vel)
            velocity_message.linear.x = lin_vel
            velocity_message.angular.z = ang_vel
            self.cmd_vel_pub.publish(velocity_message)
            time.sleep(self.t)
            # velocity_message.linear.x = linear_vel
            # velocity_message.angular.z = angular_vel
            # self.cmd_vel_pub.publish(velocity_message)
            # input('Next')
            
            
        velocity_message.linear.x = linear_vel
        velocity_message.angular.z = angular_vel
        self.cmd_vel_pub.publish(velocity_message)

def main(args=None):
    rclpy.init(args=args)
    lin_vel = [0.08639379797371934, 0.08639379797371934, 0.08639379797371932, 0.08639379797371934, 0.08639379797371934, 0.08639379797371934, 0.08639379797371934, 0.08639379797371934, 0.08639379797371934, 0.08639379797371934, 0.08639379797371934, 0]
    ang_vel = [0.00, -0.0, 0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 0]
    t = .145
    turtlebot = move(lin_vel,ang_vel,t)
    turtlebot.publish_moves()
    turtlebot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
