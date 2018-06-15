#!/usr/bin/python

from lab_baxter_common.face_id.detection import CameraNode
import rospy

if __name__ == '__main__':
    rospy.init_node("hello")
    cn = CameraNode()
    cn.detect()
