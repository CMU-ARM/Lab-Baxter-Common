#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

       
class Pause:
    def __init__(self):
        self.pub = rospy.Publisher('pause', Bool, queue_size=2)
        self.msg = None

    def pause(self, time):
        self.msg.data = True
        pub.publish(self.msg)
        rospy.sleep(time)
        self.resume()
    
    def resume(self):
        self.msg.data = False
        while not rospy.is_shutdown():
            pub.publish(self.msg)
                
