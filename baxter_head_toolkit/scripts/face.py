#!/usr/bin/python2

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

PACKAGE_NAME = "baxter_head_toolkit"


def main():
	face_client = actionlib.SimpleActionClient('FaceEmotion',faceEmotionAction)
	print("waiting for server")	
	face_client.wait_for_server()
	print("server ready")
	goal = faceEmotionGoal()
	goal.type = "happy"
	face_client.send_goal(goal)
	face_client.wait_for_result()

	time.sleep(2)

	goal.type = "anime"
	face_client.send_goal(goal)
	face_client.wait_for_result()	

	face_client.get_result()


if __name__ == '__main__':
	rospy.init_node('face_emotion_test', anonymous=True)
	main()