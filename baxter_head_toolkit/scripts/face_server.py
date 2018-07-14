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

        self._file_list = [f for f in os.listdir(self._base_path) if os.path.isfile(os.path.join(self._base_path, f))]
        self._process_files()
        rospy.loginfo("action server: faceEmotion started")

        self._cur_emotions = 'happy'
        self._cur_index = 0
        self._max_index = self._emotions[self._cur_emotions]+1
        
        self.rater = rospy.Rate(2)

    def _process_files(self):
        self._emotions = {}
        for file in self._file_list:
            file_name = file.split('_')
            if len(file_name) != 3:
                continue
            key = file_name[1]
            num = int(file_name[2].split('.')[0])
            if key not in self._emotions:
                self._emotions[key] = num
            else:
                if self._emotions[key] < num:
                    self._emotions[key] = num

    def _send_image(self, path):
        img = cv2.imread(path)
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self._img_pub.publish(msg)

    def spin(self):
        while not rospy.is_shutdown():

            #get num
            if self._cur_emotions not in self._emotions:
                self.rater.sleep()
                continue

            path = 'baxter_{}_{}.png'.format(self._cur_emotions,self._cur_index)
            self._cur_index = (self._cur_index + 1)%self._max_index
            self._send_image(os.path.join(self._base_path, path))
            self.rater.sleep()

    def _receive_action(self, goal):
        result = faceEmotionResult()
        if goal.type.lower() in self._emotions:

            rospy.loginfo("face changed to type:" + goal.type.lower())
            
            self._cur_emotions = goal.type.lower()
            self._cur_index = 0
            self._max_index = self._emotions[self._cur_emotions]+1
            
            # ##Should do a check, but we are not going to do it
            # path = 'baxter_' + goal.type.lower() + '.png'
            # final_path = os.path.join(self._base_path, path)
            # self._send_image(final_path)
            result.complete = True
        else:
            result.complete = False

        self._as.set_succeeded(result)



def main():
    #initialize node
    rospy.init_node('FaceEmotionActionServer')
    server = FaceServer()
    #loop forever
    server.spin()


if __name__ == '__main__':
    main()
