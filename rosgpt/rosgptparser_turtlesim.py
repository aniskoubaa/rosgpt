#!/usr/bin/env python3
# This file is part of rosgpt package.
#
# Copyright (c) 2023 Anis Koubaa.
# All rights reserved.
#
# This work is licensed under the terms of the Creative Commons Attribution-NonCommercial-ShareAlike 4.0
# International Public License. See https://creativecommons.org/licenses/by-nc-sa/4.0/ for details.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import copy
import math
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose
import time


class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')
        self.create_subscription(String,'/voice_cmd',self.voice_cmd_callback,10)
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.x = 0.0
        self.y  = 0.0
        self.theta  = 0.0
        self.pose = Pose()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self)
        #rclpy.spin_once(self)
        
    
    def pose_callback(self, msg):
        self.x = msg.x
        self.y  = msg.y
        self.theta  = msg.theta
        self.pose = msg
        print('-\n')
        #print (self.pose)
    

    def voice_cmd_callback(self, msg):
        #print(msg.data)
        cmd = json.loads(msg.data)
        cmd = json.loads(cmd['json']) #we only consider the pure json message. cmd['text'] contains a mix of text and json
        print('JSON command received: \n',cmd,'\n')
        if cmd['action'] == 'go_to_goal':
            location = cmd['params']['location']['value']
            self.go_to_goal(location)
        elif cmd['action'] == 'move':
            linear_speed = cmd['params'].get('linear_speed', 0.2)
            distance = cmd['params'].get('distance', 1.0)
            is_forward = cmd['params'].get('is_forward', True)
            self.move(linear_speed, distance, is_forward)
        elif cmd['action'] == 'rotate':
            angular_velocity = cmd['params'].get('angular_velocity', 1.0)
            angle = cmd['params'].get('angle', 90.0)
            is_clockwise = cmd['params'].get('is_clockwise', True)
            self.rotate(angular_velocity, angle, is_clockwise)

    def go_to_goal(self, location):
        # TODO: Implement go_to_goal method
        pass


    def get_distance(self,start, destination):
        return math.sqrt(((destination.x-start.x)**2 + (destination.y-start.y)**2))
    
    def move (self, linear_speed, distance, is_forward):
        if is_forward: 
             direction = 'forward'
        else:
             direction = 'backward'
        print('Start moving the robot ',direction,' at ', linear_speed, 'm/s and for a distance ',distance,'meter')
        #rclpy.spin_once(self)
        twist_msg=Twist()
        
        if (linear_speed>1.0) :
            print('[ERROR]: The speed must be lower than 1.0!')
            return -1
                
        twist_msg.linear.x = float(abs(linear_speed) if is_forward else -abs(linear_speed))
        twist_msg.linear.x = float(abs(linear_speed) * (1 if is_forward else -1))

        start_pose = copy.copy(self.pose)
        
        #rclpy.spin_once(self)

        while self.get_distance (start_pose, self.pose)<distance:
            print ('start_pose', start_pose, 'self.pose', self.pose, 'moved_distance: ', self.get_distance (start_pose, self.pose))
            print('self.get_distance (start_pose, self.pose)<distance: ', self.get_distance (start_pose, self.pose)<distance)
            
            #rclpy.spin_once(self)
            self.velocity_publisher.publish(twist_msg)
            self.executor.spin_once()
            time.sleep(1.0)

        twist_msg.linear.x = 0.0
        self.velocity_publisher.publish(twist_msg)
        print ('start_pose', start_pose, 'self.pose', self.pose, 'distance: ', self.get_distance (start_pose, self.pose))
        print('The Robot has stopped...')

        return 0

    def rotate (self, angular_speed_degree, desired_relative_angle_degree, clockwise):
        print('Start Rotating the Robot ...')
        #rclpy.spin_once(self)
        twist_msg=Twist()
        angular_speed_degree=abs(angular_speed_degree) #make sure it is a positive relative angle
        if (angular_speed_degree>30) :
            print (angular_speed_degree)
            print('[ERROR]: The rotation speed must be lower than 0.5!')
            return -1
        
        angular_speed_radians = math.radians(angular_speed_degree)
        twist_msg.angular.z = -abs(angular_speed_radians) if clockwise else abs(angular_speed_radians)
        twist_msg.angular.z = abs(angular_speed_radians) * (-1 if clockwise else 1)

        start_pose = copy.copy(self.pose)
        
        #rclpy.spin_once(self)

        rotated_related_angle_degree=0.0

        while rotated_related_angle_degree<desired_relative_angle_degree:
            #rclpy.spin_once(self)
            self.velocity_publisher.publish(twist_msg)
            #print ('rotated_related_angle_degree', rotated_related_angle_degree, 'desired_relative_angle_degree', desired_relative_angle_degree)
            rotated_related_angle_degree = math.degrees(abs(start_pose.theta - self.pose.theta))
            #rclpy.spin_once(self)
            
            #rclpy.spin_once(self)
            time.sleep(0.01)
        print ('rotated_related_angle_degree', rotated_related_angle_degree, 'desired_relative_angle_degree', desired_relative_angle_degree)
        twist_msg.angular.z = 0.0
        self.velocity_publisher.publish(twist_msg)
        print('The Robot has stopped...')

        return 0

    def go_to_goal(self, target_x:float, target_y:float, distance_error_tolerance=0.5):
        twist_msg = Twist()

        while (True):
            
            p_gain_linear = 1.0 
            distance = abs(math.sqrt(((float(target_x)-self.pose.get_x()) ** 2) + ((float(target_y)-self.pose.get_y()) ** 2)))
            linear_speed = distance * p_gain_linear

            p_gain_angular = 2.0
            desired_angle_goal = math.atan2(target_y-self.pose.get_y(), target_x-self.pose.get_x())
            angular_speed = (desired_angle_goal-self.pose.get_theta())*p_gain_angular

            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = angular_speed
            
            #-----____PUBLISHING----------------------
            #rclpy.spin_once(self)
            self.velocity_publisher.publish(twist_msg)
            time.sleep(0.1)
            #---------------------------------------------

            


            if (distance <distance_error_tolerance):
                print('Goal reached. Mission completed.')
                #rclpy.spin_once(self)
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                #-----____PUBLISHING TO STOP----------------------
                
                self.velocity_publisher.publish(twist_msg)
                time.sleep(0.1)
                #---------------------------------------------
                break
        print('distance = ', distance)
        print ('angle error = ', (desired_angle_goal-self.pose.get_theta()))
        return 0

def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
