#!/usr/bin/env python

import rospy

from baxter_core_msgs.msg import(
    HeadPanCommand
)

       
class Publish:
    def __init__(self):
        self.pub = rospy.Publisher('/robot/head/command_head_pan', HeadPanCommand, queue_size=2)
        self.sub = rospy.Subscriber('/temporary', HeadPanCommand, self._cb, queue_size=1)
        self._last_msg = None
        
    def _cb(self, msg):
        self._last_msg = msg
        
    def execute(self):
        while not rospy.is_shutdown():
            if self._last_msg != None:
                # get message from 'temporary' topic
                msg = HeadPanCommand()
                msg.enable_pan_request = self._last_msg.enable_pan_request
                msg.target = self._last_msg.target
                
                # smooth velocity curve through quintic splining
                x = 0
                while msg.target == self._last_msg.target and x < 216:
                    msg.speed_ratio = (-3.4055831149*(10**-12))*(x**5)+(1.2685088515*(10**-9))*(x**4)+(2.1635717468*(10**-7))*(x**3)+(-0.00018699145565)*(x**2)+(0.0248928770417)*x+(0.000107467879942)
                    self.pub.publish(msg) 
                    x += 1
                
if __name__ == '__main__':
    rospy.init_node('mover')
    pb = Publish()
    pb.execute()
    

