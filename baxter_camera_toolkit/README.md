# Baxter Camera Toolkit

This package contains the **camera_control_helpers.py** file with the **CameraController** class, which allows users to safely open cameras without worrying about the 2 cameras on at a time limitation.

### Usage
This script defines three static methods that are available to the user:
- **CameraController.createCameraSettings(kwargs)**: this function returns an argument to be passed as the 'settings' parameter to CameraController.openCamera. Note that this function does not check whether the set settings are valid, that check happens in openCamera. Valid parameters for this function include:
    - **width**: the width-aspect of the resolution
    - **height**: the height aspect of the resolution.  Note there are only 6 valid resolutions: http://sdk.rethinkrobotics.com/wiki/Cameras
    - **fps**: the frames per second
    - **exposure**: the camera exposure.  Valid values are 0-100, or -1 for auto exposure.
    - **gain**: the camera gain.  Valid values are 0-79, or -1 for auto.
    - **whiteBalanceR**: the white balance red.  Valid values are 0-4095, or -1 for auto
    - **whiteBalanceG**: the white balance green.  Valid values are 0-4095, or -1 for auto
    - **whiteBalanceB**: the white balance blue.  Valid values are 0-4095, or -1 for auto
    - **windowX**: the width of the window.  Valid values depends on the current camera resolution and whether resolutionHalf is enabled or not
    - **windowY**: the height of the window.  Valid values depends on the current camera resolution and whether resolutionHalf is enabled or not
    - **flip**: indicates whether to flip the camera image (non-zero) or not (zero)
    - **mirror**: indicates whether to mirror the camera image (non-zero) or not (zero)
    - **resolutionHalf**: indicates whether to enable half-resolution (non-zero) or not (zero).


- **CameraController.openCameras(camera, camera2=None, settings=None, settings2=None)**: this function turns on the specified camera(s) with the specified settings (a CameraSettings msg).  If settings are not specified, it uses Baxter's default settings.  If two cameras are specified, it is guarenteed to close the third camera (if it is on).  If one camera is specified, it will randomly turn off one of the other two camera (if they both are on).


- **CameraController.closeCamera(camera)**: this function turns off the specified camera.


##### Sample Usage

```python
from lab_baxter_common.camera_control_helpers import CameraController

if __name__ == '__main__':
    settings=CameraController.createCameraSettings(width=640, height=400, exposure=-1)
    CameraController.openCameras("head_camera", "right_hand_camera", settings=settings)
```

### Notes
- Cameras on the Baxter seem to have three states (this information is pieced together from Baxter docs and forums): open, powered on, and off.  Cameras that are open have power turned on to them and are streaming.  Cameras that are powered on have power going to them, but are not streaming.  And cameras that are off have no power going to them and are not streaming.  The limitation about cameras being on applies to cameras being POWERED on -- in Baxter, if two cameras are powered on but neither are open, you still won't be able to open the third without first closing one of the others.  
Generally, calling the "/cameras/list" rosservice will list the cameras that are POWERED ON, whereas looking through rostopics will list cameras that are currently OPEN.  The only exception to this rule is when Baxter is first powered on, when "/cameras/list" returns all three cameras, even though according to Baxter docs only two can be powered on at once.  At any time, calling the "/cameras/open" rosservice will move the specified camera to open (if the other two cameras are not powered on).  Calling the "/cameras/close" rosservice will turn off the camera completely, and possibly (I'm not completely sure of the mechanics of this) turn another off camera to the POWERED ON (not open) state.  See the Baxter forums and Camera docs for more information: http://sdk.rethinkrobotics.com/wiki/Cameras
- One improvement that can be made to these helpers is to incorporate a global variable that keeps track of calls to openCamera.  Then, when a user calls openCamera with only one camera, as oppsoed to closing a random camera as it currently does, it will close the last other camera to be opened.
