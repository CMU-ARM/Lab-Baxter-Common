# baxter_general_toolkit
A collection of baxter code that can be shared between projects

### lab_baxter_common.general_toolkit
- `set_posture()`
    - Given a name of the posture, move baxter to that posture 
- `solve_ik(arm_name, pose)`
    - Given the arm and pose (stamped or not), converts that pose into joint_states message that move to that pose

### lab_baxter_common.camera_toolkit
Wrapper to open baxter camera
```
    settings = CameraController.createCameraSettings(width=640, height=400, exposure=-1) # settings for the first camera
    settings2 = CameraController.createCameraSettings(width=1280, height=800, exposure=-1) # settings for the second camera
    CameraController.openCameras("head_camera", "right_hand_camera", settings=settings, settings2=settings2)
```