#! /usr/bin/env python
import sys
import numpy as np
import math
import time
import random

#ROS Imports
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

roll = pitch = yaw = 0.0

class MazeSolve:

    path = [[0,0,0,0]]
    
    gb = 0
    global d 
    global fd
    global regions 
    global backdist
    global st
    regions = {
            'right':  0,
            'front':  10,
            'left':   0,
        }
    d=2 #side distance
    fd=2 #front distance
    backdist= 1.2
    st=1 #straight flag


## ----- Topics and initialization ----- ##

    def __init__(self):
        #Topics & Subs, Pubs
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        lidarscan_topic = '/kobuki/laser/scan'
        twist_topic = '/cmd_vel'
        self.lidar_sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.lidar_callback) # Subscribe to LIDAR
        self.drive_pub = rospy.Publisher(twist_topic, Twist, queue_size=10) # Publish to Twist
        self.move=Twist()
        self.thetacurrent= 0
    
## ----- Odometry  ----- ##

    def odom_callback(self, odom_msg):
        global st
        orientation_q = odom_msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        x, y = odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y
        self.thetacurrent= yaw

        if st==1:
            self.desired = self.thetacurrent
            self.angle_compensator() # adjusting direction to face desired angle accurately 
            error = self.desired - self.thetacurrent
            werrorpr = math.atan2(math.sin(error),math.cos(error))
            self.move.angular.z=5*werrorpr
            self.drive_pub.publish(self.move)

## ----- Lidar Readings ----- ##

    def lidar_callback(self, msg):
        global regions 
        regions = {
            'right':  msg.ranges[0],
            'front':  min(min(msg.ranges[320:400]), 10),
            'left':   msg.ranges[719],
        }


## -----Decision making and adjustments ----- ##

        ## making sure stops to make a decision at the center 
    def stop(self):
        rospy.sleep(0.8)
        self.move.linear.x=0
        self.move.angular.z=0
        self.drive_pub.publish(self.move)
        rospy.sleep(2.5)

        ## checking what kind of intersaction is ahead and making decisions in accordance
    def check_decisions(self):
        global d
        global fd
        global regions
        global backdist
        if regions['left'] > d and regions['front'] > fd and   regions['right'] > d :
            if self.gb == 0:
                self.path.append([0,0,0,3])
            self.stop()
            self.make_decision(self.path)
            rospy.sleep(3)          
        elif regions['left'] > d and regions['front'] > fd and   regions['right'] < d :
            if self.gb == 0:
                self.path.append([0,0,1,3])
            self.stop()
            self.make_decision(self.path) 
            rospy.sleep(3) 
        elif regions['left'] > d and regions['front'] < fd and   regions['right'] > d :
            if self.gb == 0:
                self.path.append([0,1,0,3])  
            time.sleep(1)
            self.stop()
            self.make_decision(self.path) 
            rospy.sleep(3) 
        elif regions['left'] < d and regions['front'] > fd and   regions['right'] > d :  
            if self.gb == 0:
                self.path.append([1,0,0,3])
            self.stop()
            self.make_decision(self.path)  
            rospy.sleep(3) 
        elif regions['left'] > d and regions['front'] < 0.8 and   regions['right'] < d : 
            self.go_left()
            rospy.sleep(3) 
        elif regions['left'] < d and regions['front'] > 1.4 and   regions['right'] < d :  
            self.go_straight()
        elif regions['left'] < d and regions['front'] < 0.8 and   regions['right'] > d :
            self.go_right()
            rospy.sleep(3) 
        elif regions['left'] < d and regions['front'] < backdist and   regions['right'] < d : 
            self.gb=1
            self.stop()
            self.go_back()
        
        
        ## adjusting 4 tuples to the new view angle depending on the turn
    def shift(self,rand):
       
        for x in range(0,rand+1):
            first=self.path[-1][0]
            second=self.path[-1][1]
            third=self.path[-1][2]
            fourth=self.path[-1][3]
            self.path[-1][0]=second
            self.path[-1][1]=third
            self.path[-1][2]=fourth
            self.path[-1][3]=first
        

        ## Randomly chossing 
    def make_decision(self, path):
        a=0
        for x in range(0,3):  ## check if all paths have been explored 
            if self.path[-1][x] == 0:
                a=1
            elif self.path[-1][x] == 3:
                r=x


        if a == 0:    ## all paths have been explored, tries to go back 
            self.gb=1
            if r == 0:
                self.go_left()
            elif r == 1:
                self.go_straight()
            elif r == 2:
                self.go_right()
            self.path.pop()

        else:  ## make a random choice between available directions
        
        
         ## if want to bias the randomness
       
        #     rand =random.randint(0,10)
        #     print 'random val1:', rand
        #     if rand>=0 and rand<6:
        #         rand=1
        #     elif rand>=6 and rand <8:
        #         rand=2
        #     elif rand>=8:
        #         rand=0
        # while (self.path[-1][rand]!=0):
        #         rand =random.randint(0,10)
        #         print 'random val2:', rand
        #         if rand>=0 and rand<6:
        #             rand=1
        #         elif rand>=6 and rand <8:
        #             rand=2
        #         elif rand>=8:
        #             rand=0

        ## No bias 
            rand =random.randint(0,3)
            while (self.path[-1][rand]!=0):
                rand =random.randint(0,3)
            if rand == 0:
                self.go_left()
                self.path[-1][rand] = 1
                self.shift(rand) 
            elif rand ==1:
                self.go_straight()
                self.path[-1][rand] = 1
                self.shift(rand) 
            elif rand == 2:
                self.go_right()
                self.path[-1][rand] = 1
                self.shift(rand)  
            self.gb=0
        
        
        ## adjusting the angles to be more precise 
    def angle_compensator(self):
        self.desired = math.atan2(math.sin(self.desired),math.cos(self.desired))
        #up
        if self.desired <= -1.1 and self.desired >=-1.8:
            self.desired= -1.57
        #left
        elif self.desired > -0.7 and self.desired < 0.7:
            self.desired= 0.05
        #right
        elif self.desired > 2.70 and self.desired < 3.14:
            self.desired= -3.12
        #right
        elif self.desired < -2.70 and self.desired > -3.14:
            self.desired= -3.12
        #down
        elif self.desired <1.9 and self.desired >1.1:
            self.desired = 1.57



    ## ----- Turning  ----- ##

    def go_left(self):
        global st
        st=0
        self.move.linear.x=0.2
        self.move.angular.z=1.2
        self.desired = self.thetacurrent+(1.60)
        self.angle_compensator() 
        error = self.desired - self.thetacurrent
        werrorpr = math.atan2(math.sin(error),math.cos(error))
        while abs(werrorpr) > 0.2:
            self.move.linear.x=0
            self.move.angular.z=0.5
            error = self.desired - self.thetacurrent
            werrorpr = math.atan2(math.sin(error),math.cos(error))
            self.drive_pub.publish(self.move)
            rospy.sleep(0.2)
        st=1
        self.move.linear.x=0.5
        self.move.angular.z=0
        duration = 3
        self.drive_pub.publish(self.move)
        
    def go_straight(self):
        global st
        st=1
        self.move.linear.x=0.5
        self.desired = self.thetacurrent
        self.angle_compensator()
        error = self.desired - self.thetacurrent
        werrorpr = math.atan2(math.sin(error),math.cos(error))
        self.move.angular.z=1*werrorpr
        self.drive_pub.publish(self.move)


        
    def go_right(self):
        global st
        st=0
        self.desired = self.thetacurrent-(1.32)
        self.angle_compensator()
        error = self.desired - self.thetacurrent
        werrorpr = math.atan2(math.sin(error),math.cos(error))
        while abs(werrorpr) > 0.25:
            self.move.linear.x=0
            self.move.angular.z=-0.5
            error = self.desired - self.thetacurrent
            werrorpr = math.atan2(math.sin(error),math.cos(error))
            self.drive_pub.publish(self.move)
            rospy.sleep(0.2)
        st=1
        self.move.linear.x=0.5
        self.move.angular.z=0
        duration = 3
        self.drive_pub.publish(self.move)
      

    def go_back(self):
        global st
        st=0
        self.gb=1
        self.desired = self.thetacurrent-(2.9)
        self.angle_compensator()
        error = self.desired - self.thetacurrent
        while abs(error) > 0.2:
            self.move.linear.x=0
            self.move.angular.z=-0.5
            error = self.desired - self.thetacurrent
            self.drive_pub.publish(self.move)
            rospy.sleep(0.2)
        st=1
        self.move.linear.x=0.5
        self.move.angular.z=0
        duration = 3
        self.drive_pub.publish(self.move)


    def loop(self):
        global regions
        while not rospy.is_shutdown():
            self.check_decisions() ## making sure it constantly checks for intersections


def main(args):

    rospy.init_node("Mazesolver_node", anonymous=True)
    ms =MazeSolve()
    ms.loop()
    rospy.sleep(0.1)
    rospy.spin()
    
    
if __name__=='__main__':
	main(sys.argv)