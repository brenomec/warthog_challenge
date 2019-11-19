#!/usr/bin/env python
from __future__ import print_function
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from move_base_msgs.msg import MoveBaseActionResult
from cv2 import aruco
import rospy
import math
import roslib
import sys
import cv2 as cv
import numpy as np

class Warthog():

    def __init__(self):
        # Creates a node with name 'warthog_controller' and make sure it is a unique node (using anonymous=True).
        rospy.init_node('warthog_controller', anonymous=True)

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        # Subscriber to the Odom data:
        self._odomSubscriber = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.callback_result)

        # Internal Variables iniciation
        self._pose = [0, 0, 0]
        self.goal_pose = [0, 0]

    def callback_result(self,data):
        self.result = True

    # Function to move the robot
    def move2goal_init(self):
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.pose.orientation.w = 0.7
        goal.pose.position.x = 5
        goal.pose.position.y = -4
        rospy.sleep(1)
        self.velocity_publisher.publish(goal)

    def move2goal_tag_0(self):
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.pose.orientation.w = 0.5
        goal.pose.position.x = 5
        goal.pose.position.y = 5
        rospy.sleep(1)
        self.velocity_publisher.publish(goal)

    def move2goal_zero(self):
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.pose.orientation.w = 0.2
        goal.pose.position.x = 0
        goal.pose.position.y = 0
        rospy.sleep(1)
        self.velocity_publisher.publish(goal)

class image_detection():

    def __init__(self):
        
        # Creates a node with name 'warthog_controller' and make sure it is a unique node (using anonymous=True).
        rospy.init_node('warthog_controller', anonymous=True)
        
        # Create the ROS publisher and Subscriber
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/warthog/camera1/image_raw",Image,self.callback)
        self.id = []
        # Initial Variables
        self.bridge = CvBridge()

    def callback(self,data):
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)

        # Load the image
        image = cv_image
    
        # Converting the image to a grayscale
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)

        # Loading the aruco original dictionary for comparison
        aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)

        # Specific Parameters generation
        parameters =  aruco.DetectorParameters_create()

        # Lists of ids and the corners beloning to each id
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        # Function that displays the id and the corners to the image
        gray = aruco.drawDetectedMarkers(image, corners, ids)

        # show the images
        cv.imshow('id_detection',gray)
        cv.waitKey(3)

        #
        self.id = ids

        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
          print(e)

    def getIds(self):
        if (not self.id):
            return list()
        return self.id

class every():

    def __init__(self):
        self.id = image_detection()
        self.warthog = Warthog()
        self.encontrado = False

    def move(self):
        lista = self.id.getIds()
        self.warthog.move2goal_init()
        if not self.encontrado:
            if 4 in lista:
                print ("Tag recognized")
                self.encontrado = True
                self.warthog.move2goal_tag_0()
                self.encontrado = False
                if not self.encontrado:
                    if 0 in lista:
                        print ("Tag recognized")
                        self.encontrado = True
                        self.warthog.move2goal_zero()
                        # self.encontrado = False

if __name__ == "__main__":
    E = every()
    while(not rospy.is_shutdown()):
        E.move()
        whileRate = rospy.Rate(50)
        whileRate.sleep()