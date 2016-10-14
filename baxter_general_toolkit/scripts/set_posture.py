#!/usr/bin/env python


"""
Tool that use to set Baxter into different modes
"""

import argparse
# import baxter_interface


def move_to_posture(posture_name):

    ##TODO actually move it to different posture
    #1) read the name of the posture
    #2) move both arms to that posture
    #3) Read it from a file instead of hardcoding it

    claw_pose_left = {
        left_w0: 0.2446699356677235,
        left_w1: 0.7531845668517382,
        left_w2: 0.034514567727421806,
        left_e0: -0.3236699462438223,
        left_e1: 1.8795099603566032,
        left_s0: -0.2899223689103432,
        left_s1: -1.026616642292313 
    }

    rospy.init_node("bax_set_posture")
    left_limb = baxter_interface.Limb('left')
    right_limb = baxter_interface.Limb('right')

    left_limb.move_to_joint_positions(claw_pose_left)    


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Move Baxter to different starting postures')
    parser.add_argument('-p','--posture', type=str, default="claw_pose")
    args = parser.parse_args()
    print args.posture

    move_to_posture(args.posture)