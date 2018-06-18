#!/usr/bin/python

from lab_baxter_common.face_id.detection import CameraNode
from lab_polly_speech.polly_speech import PollySpeech
import rospy

if __name__ == '__main__':
    rospy.init_node("hello")
    cn = CameraNode()
    ps = PollySpeech()
    cn.detect()
    ps.speak("Hello, I am taking a break!")
    cn.detect()

