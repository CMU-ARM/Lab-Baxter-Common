# Lab Baxter Common

This repository contains generic helper functions and rospackages related to baxter.  These include:
- **head_toolkit**: this package contains scripts to send a face-image to baxter via an actionlib.
- **general_toolkit**: this package contains scripts that does general purpose stuff like moving to some particular posture, etc.
- **camera_toolkit**: this package contains the *camera_control_helpers.py* file with the *CameraController* class, which allows users to safely open cameras without worrying about the 2 cameras on at a time limitation.
