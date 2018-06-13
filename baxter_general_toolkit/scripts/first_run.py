#!/usr/bin/python

from lab_baxter_common.face_id.face_id import CameraNode
import rospy

if __name__ == '__main__':
    rospy.init_node("Face")
    cn = CameraNode()
    cn.detect()
    
