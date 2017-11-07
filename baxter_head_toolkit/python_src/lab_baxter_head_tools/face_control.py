#!/usr/bin/python

import baxter_interface
import os
import cv2
import rospkg 
import cv_bridge
import rospy
import actionlib
from sensor_msgs.msg import (
    Image,
)
import time

from baxter_head_toolkit.msg import(
    faceEmotionAction,
    faceEmotionGoal,
    faceEmotionResult,
)

#PACKAGE_NAME = "baxter_head_toolkit"


class FaceController(object):

    def __init__(self):
	    self._face_client = actionlib.SimpleActionClient('FaceEmotion',faceEmotionAction)
	    print("waiting for server")	
	    self._face_client.wait_for_server()

    def set_emotion(self, name, block = False):
        goal = faceEmotionGoal()
        goal.type = name
        self._face_client.send_goal(goal)
        if block:
	        self._face_client.wait_for_result()
    
    def wait(self):
        self._face_client.wait_for_result()


# def main():


# 	time.sleep(2)

# 	goal.type = "anime"
# 	face_client.send_goal(goal)
# 	face_client.wait_for_result()	

# 	face_client.get_result()


# if __name__ == '__main__':
# 	rospy.init_node('face_emotion_test', anonymous=True)
# 	main()