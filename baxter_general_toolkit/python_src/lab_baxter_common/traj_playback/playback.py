#!/usr/bin/env python

from recorder import JointRecorder
from trajectory import Trajectory
import baxter_interface
import rospkg
import rospy
import os
import time
import actionlib
from dynamic_reconfigure.server import Server
from baxter_interface import CHECK_VERSION

def _search_path(file_path):
    """Completes the file path if given a filename and not a directory. If only given a file name and 
    not a directory, the system will search for it in the library folder. The library folder is defined
    either in rosparam:`/baxter/playback_library_dir` or if doesn't exist. Defaults to 
    'baxter_general_toolkit/trajectory_playback_library'
    """

    if os.path.split(file_path)[0] == "":
        #This means that this isn't a directory
        playback_directory = rospy.get_param('/baxter/playback_library_dir', 
            os.path.join(rospkg.RosPack().get_path('baxter_general_toolkit'), 'trajectory_playback_library'))
        file_path = os.path.join(playback_directory, file_path)
    return file_path


def record(file_path, rate=100.0): 

    rospy.logdebug("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    rospy.logdebug("Enabling robot... ")
    rs.enable()

    file_path = _search_path(file_path)
    file_dir, _ = os.path.split(file_path)
    # make library if non-existent
    if not os.path.isdir(file_dir):
 	    os.mkdir(file_dir)
 	
    if os.path.isfile(file_path):
        raise RuntimeError("Unable record! File already exists.")
        return
        
    recorder = JointRecorder(file_path, rate)
    recorder.record()


def playback(file_path, loops=1):

    file_path = _search_path(file_path)
    if not os.path.isfile(file_path):
        raise RuntimeError("Playback file doesn't exist")

    traj = Trajectory()
    traj.parse_file(file_path)

    
    # for safe interrupt handling
    rospy.on_shutdown(traj.stop)
    result = True
    loop_cnt = 1
    loopstr = str(loops)
    if loops == 0:
        loops = float('inf')
        loopstr = "forever"
    while (result == True and loop_cnt <= loops
           and not rospy.is_shutdown()):
        rospy.logdebug("Playback loop {} of {}".format(loop_cnt, loopstr))
        traj.start()
        result = traj.wait()
        loop_cnt = loop_cnt + 1
    rospy.logdebug("Exiting - File Playback Complete")


if __name__ == '__main__':
    filename = raw_input("Enter desired full name of .txt file: ")
    while filename == '':
        filename = raw_input("Enter desired full name of .txt file: ")

	# get directory path
    rospack = rospkg.RosPack()
    path = os.path.join(rospack.get_path("baxter_general_toolkit"), "playback_library/" + filename)
    
    # initialize node
    print("Initializing node... ")
    rospy.init_node("joint_playback")
    
    if Path(path).is_file():
        print("File already exists. Can only playback.")
        loop_count = int(raw_input("How many times would you like to loop playback? "))
        while loop_count < 0:
            loop_count = int(raw_input("How many times would you like to loop playback? "))
        playback(path, loops=loop_count)
    else:
    	print("File does not exist. Must at least record.")

        # record/playback
        R = ['r', 'R']
        B = ['b', 'B']
    
        command = raw_input("Record(R)/Both(B)? ")
        if command in B:
            loop_count = int(raw_input("How many times would you like to loop playback? "))
            while loop_count < 0:
                loop_count = int(raw_input("How many times would you like to loop playback? "))
                print loop_count
        if command in R or command in B:
            record(path, 100.0)
        if command in B:
            rospy.sleep(2)
            playback(path, loops=loop_count)

    # cleanup and shutdown
    os.system("rosnode cleanup")
    rospy.signal_shutdown("All done!")
    

    
    
