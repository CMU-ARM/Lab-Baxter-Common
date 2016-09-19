import rospy
import rosgraph
import socket
import time

from baxter_core_msgs.srv import OpenCamera
from baxter_core_msgs.srv import CloseCamera
from baxter_interface.msg import CameraControl
from baxter_interface.msg import CameraSettings

class CameraControls(object):
    _validCameras = ['head_camera', 'left_hand_camera', 'right_hand_camera']
    _validRes = [(1280, 800),
                 (960, 600),
                 (640, 400),
                 (480, 300),
                 (384, 240),
                 (320, 200)]
    # Used to represent when the camera is using automatic controls.
    # Valid for exposure, gain and white balance.
    _autoControl = -1

    @staticmethod
    def _getOpenCameras():

    @staticmethod
    def _isValidCameraSettings(settings):

    @staticmethod
    def _setCameraControls(controls, name, value):
        control = CameraControl() # exposure
        control.id = name
        control.value = value
        controls.append(control)

    @staticmethod
    def _getDefaultSettings():
        defaultSettings = CameraSettings()
        # Default camera setting on startup: http://sdk.rethinkrobotics.com/wiki/Cameras
        defaultSettings.width = 320
        defaultSettings.height = 200
        defaultSettings.fps = 25
        # # Auto Camera CameraControls
        # controls = []
        # _setCameraControls(controls, CameraControl.CAMERA_CONTROL_EXPOSURE, _autoControl)
        # _setCameraControls(controls, CameraControl.CAMERA_CONTROL_GAIN, _autoControl)
        # _setCameraControls(controls, CameraControl.CAMERA_CONTROL_WHITE_BALANCE_R, _autoControl)
        # _setCameraControls(controls, CameraControl.CAMERA_CONTROL_WHITE_BALANCE_G, _autoControl)
        # _setCameraControls(controls, CameraControl.CAMERA_CONTROL_WHITE_BALANCE_B, _autoControl)
        return defaultSettings

    @staticmethod
    def openCamera(camera, camera2=None, settings=None, settings2=None):
        """
        Opens the specified camera, with settings, and camera2/settings2 if
        specified.  Cameras should be strings, settings should be instances of
        CameraSettings
        """
        # Check that cameras are valid
        if camera not in _validCameras:
            raise ValueError('invalid camera %s, valid cameras are %s' % (camera, str(validCameras)))
        if camera2 is not None and camera2 not in _validCameras:
            raise ValueError('invalid camera2 %s, valid cameras are %s' % (camera2, str(validCameras)))

        # Check that the settings are valid
        if settings is not None:
            ok, err =  _isValidCameraSettings(settings)
            if not ok: raise ValueError("invalid settings: %s", err)
        else:
            settings = _getDefaultSettings()
        if settings2 is not None:
            ok, err =  _isValidCameraSettings(settings2)
            if not ok: raise ValueError("invalid settings2: %s", err)
        else:
            settings2 = _getDefaultSettings()

        # Get the currently open cameras
        openCameras = _getOpenCameras()
        numOpenCameras = openCameras.values().count(True)
        numDesiredCameraOpen = 0 # between camera and camera2, how many are open
        if openCameras[camera]:
            numDesiredCameraOpen += 1
        if camera2 is not None and openCameras[camera2]:
            numDesiredCameraOpen += 1

        # Close the necessary camera
        cameraToClose = None
        if camera2 is None:
            if not openCameras[camera] and numOpenCameras == 2:
                for cam in _validCameras:
                    if cam != camera:
                        cameraToClose = cam
                        break
        else:
            if numDesiredCameraOpen < numOpenCameras:
                for cam in _validCameras:
                    if cam != camera and cam != camera2:
                        cameraToClose = cam
                        break
        if cameraToClose is not None:
            closeCamera(cameraToClose)

        # Open the specified cameras
        rospy.wait_for_service('cameras/open')
        openService = rospy.ServiceProxy('cameras/open', OpenCamera)
        try:
            resp = openService(camera, settings)
            if resp.err != 0:
                raise OSError("error turning on %s: %d", camera, resp)
        except rospy.ServiceException as err:
            raise OSError("error turning on %s: %d", camera, err)
        if camera2 is not None and camera2 != camera:
            try:
                resp = openService(camera2, settings2)
                if resp.err != 0:
                    raise OSError("error turning on %s: %d", camera2, resp)
            except rospy.ServiceException as err:
                raise OSError("error turning on %s: %d", camera2, err)


    @staticmethod
    def closeCamera(camera):
