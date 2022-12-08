#!/usr/bin/python3
# -*- coding: utf-8 -*-
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
class ObstacleAvoidingBot(Node):
    def __init__(self):
        super().__init__('Go_to_position_node') ## name of the node
        # publisher
        self.publisher = self.create_publisher(Twist, 'my_bot0/cmd_vel', 40)
        #subscriber
        self.odom_sub = self.create_subscription(Odometry,'my_bot0/odom',self.get_position,40)
        #
        timer_period = 0.2;self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        self.linear_vel = 0.2
        self.x=0
        self.y=0
        self.velocity=Twist()
        self.yaw_error=0
        self.yaw_precision = 4.0 * (math.pi / 180) 
        self.desired_yaw=0
        self.dist_thresh_wf=1.2
        self.dist_too_close_to_wall = 1.0
        ################################
        self.subscription=self.create_subscription(LaserScan,'my_bot0/scan',self.get_scan_values,40)
        self.region_1=0;self.region_2=0;self.region_3=0 
        self.error=0
        self.status=""
        self.current_yaw=0
        self.wall_following_state=""
    def get_position(self,position):
        self.x=position.pose.pose.position.x
        self.y=position.pose.pose.position.y
        self.layer=5
        print(round(self.region_1,3) ,"/",round(self.region_2,3),"/",round(self.region_3,3))
    def get_scan_values(self,scan_data):
        ## We have 360 data points so we divide them in 3 region
        ## we say if there is something in the region get the smallest distance value of a point in the area
        
        self.region_1= min(min(scan_data.ranges[15:45])   , 100 )
        self.region_2= min(min(scan_data.ranges[80:100]) , 100 )
        self.region_3= min(min(scan_data.ranges[135:150]) , 100 )
        # self.region_4= min(scan_data.ranges[135] , 100 )
        # self.region_5= min(scan_data.ranges[179] , 100 )
        print(round(self.region_1,3) ,"/",round(self.region_2,3),"/",round(self.region_3,3),"/",round(self.x,2),"/",round(self.y,3),"/",round(self.yaw_error,3),"/",round(self.desired_yaw,3),"/",round(self.current_yaw,3),"/",self.status,"/", self.wall_following_state)

    def follow_wall(self):
        """
        This method causes the robot to follow the boundary of a wall.
        """
        d=1.7
        if self.region_1 > d and self.region_2 > d and self.region_3 > d:
            self.wall_following_state = "search for wall"
            self.velocity.angular.x = 0.2
            self.velocity.angular.z = 0.0 # turn right to find wall
             
        elif self.region_1 > d and self.region_2 < d and self.region_3 > d:
            self.wall_following_state = "turn left"
            self.velocity.angular.z = -1.572
            self.velocity.angular.x = 0.2
             
             
        elif (self.region_1 > d and self.region_2 > d and self.region_3 < d):
            if (self.region_3 < self.dist_too_close_to_wall):
                # Getting too close to the wall
                self.wall_following_state = "turn left_1"
                self.velocity.angular.x = 0.2
                self.velocity.angular.z = 0.572      
            else:           
                # Go straight ahead
                self.wall_following_state = "follow wall_1" 
                self.velocity.angular.x = 0.2   
                                     
        elif self.region_1 < d and self.region_2 > d and self.region_3 > d:
            self.wall_following_state = "search for wall_1"
            self.velocity.angular.x = 0.2
            self.velocity.angular.z = 0.572 # turn right to find wall
             
        elif self.region_1 > d and self.region_2 < d and self.region_3 < d:
            self.wall_following_state = "turn left_2"
            self.velocity.angular.z = 0.572
             
        elif self.region_1 < d and self.region_2 < d and self.region_3 > d:
            self.wall_following_state = "turn left_3"
            self.velocity.angular.z = 0.572
             
        elif self.region_1 < d and self.region_2 < d and self.region_3 < d:
            self.wall_following_state = "turn left_4"
            self.velocity.angular.z = 1.572
             
        elif self.region_1 < d and self.region_2 > d and self.region_3 < d:
            self.wall_following_state = "search for wall_2"
            self.velocity.angular.x = 0.2
            self.velocity.angular.z = -1.572 # turn right to find wall
             
        else:
            pass
        
        # if self.region_1 > d and self.region_2 > d and self.region_3 > d:
        #     self.wall_following_state = "search for wall_5"
        #     self.velocity.angular.x= 0.2
        #     self.velocity.angular.z= -1.625 # turn right to find wall
                
        # elif self.region_1 > d and self.region_2 < d and self.region_3 > d:
        #     self.wall_following_state = "turn left_6"
        #     self.velocity.angular.z = -1.625
        # elif self.region_1 < d and self.region_2 > d and self.region_3 > d:
        #     self.wall_following_state = "turn left_7"
        #     self.velocity.angular.z = 0.0     
        #     self.velocity.angular.x= 0.2   
        
    def send_cmd_vel(self):
        if self.region_1>2.5 and self.region_2>2.5 and self.region_3>2.5:  # Flocking
            self.desired_yaw = math.atan2(15-self.y, 0-self.x)
            self.current_yaw= math.atan2(self.y, self.x)
            self.yaw_error = self.desired_yaw- self.current_yaw
            self.wall_following_state="Flocking"
            if abs(self.yaw_error)>self.yaw_precision:
                if self.yaw_error>0:
                    self.velocity.angular.z=0.2
                    self.velocity.linear.x=-0.5
                    self.velocity.linear.y=0.0
                    self.status='positive'
                elif self.yaw_error<0 :
                    # if abs(self.yaw_error)<1.572:
                        self.velocity.angular.z=-0.2
                        self.velocity.linear.x=-0.5
                        self.velocity.linear.y=0.0
                        self.status='negative'
                    # else:
                    #     self.velocity.angular.z=1.625
                    #     self.velocity.linear.x=0.0
                    #     self.velocity.linear.y=0.0
                    #     self.status='negative22'
                    
            else:
                self.velocity.angular.z=0.0
                self.velocity.linear.x=-0.5
                # self.velocity.linear.y=2.2
                self.status='None'

        ######Wall_Following##################################
        else:
             self.follow_wall()

        self.publisher.publish(self.velocity)  
    
def main(args=None):
    rclpy.init(args=args)
    oab=ObstacleAvoidingBot()
    rclpy.spin(oab)
    rclpy.shutdown()

if __name__ == '__main__':
    main()