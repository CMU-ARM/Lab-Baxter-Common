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

from baxter_head_toolkit.msg import(
    faceEmotionAction,
    faceEmotionGoal,
    faceEmotionResult,
)

PACKAGE_NAME = "baxter_head_toolkit"

class FaceServer:

    def __init__(self):

        self._img_pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
        self._as = actionlib.SimpleActionServer("FaceEmotion", faceEmotionAction, execute_cb=self._receive_action, auto_start=False)
        self._as.start()

        rospack = rospkg.RosPack()
        self._pack_path = rospack.get_path(PACKAGE_NAME)
        self._base_path = os.path.join(self._pack_path,'res') #Which folder the images are located
        
        self._send_image(os.path.join(self._base_path,"baxter_standard.png"))
        rospy.loginfo("action server: faceEmotion started")
    


    def _send_image(self, path):
        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self._img_pub.publish(msg)

    def _receive_action(self, goal):
        emotion_type = goal

        rospy.loginfo("face changed to type:" + goal.type.lower())
        ##Should do a check, but we are not going to do it
        path = 'baxter_' + goal.type.lower() + '.png'
        final_path = os.path.join(self._base_path, path)
        self._send_image(final_path)

        result = faceEmotionResult()
        result.complete = True

        self._as.set_succeeded(result)



def main():
    #initialize node
    rospy.init_node('FaceEmotionActionServer')
    server = FaceServer()
    #loop forever
    rospy.spin()


if __name__ == '__main__':
    main()
