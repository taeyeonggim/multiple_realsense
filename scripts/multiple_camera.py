#!/usr/bin/env python3.7
import argparse
import rospy
import subprocess

from sensor_msgs.msg import Image
from ros_logger import ROSLogger
from multiple_realsense.srv import LaunchState, LaunchStateResponse


class CameraManager:
    def __init__(self, args, logger):
        self.args = args
        self.logger = logger
        self.response = LaunchStateResponse()
        self.avoid_camera_launch = None
        self.camera01_launch = None
        self.camera02_launch = None

    def avoid_camera_enable(self, req):
        if req.enable:
            avoid_camera_launch = subprocess.Popen(
                ["roslaunch", "realsense2_camera", "rs_rgbd_d435i.launch"]
            )
            color_check = rospy.wait_for_message(
                "/camera/color/image_raw", Image, timeout=None
            )
            if color_check:
                self.response.success = True
                self.logger.info("avoid_camera launched.")
            else:
                self.response.success = False
                self.logger.info("avoid_camera launch failed.")
        else:
            avoid_camera_launch.terminate()
            self.logger.info("avoid_camera stopped.")
            self.response.success = True
        return self.response

    def camera01_enable(self, req):
        if req.enable:
            self.camera01_launch = subprocess.Popen(
                ["roslaunch", "realsense2_camera", "rs_rgbd_d415.launch"]
            )
            color_check = rospy.wait_for_message(
                "/camera01/color/image_raw", Image, timeout=None
            )
            if color_check:
                self.response.success = True
                self.logger.info("camera01 launched.")
            else:
                self.response.success = False
                self.logger.info("camera01 launch failed.")
        else:
            self.camera01_launch.terminate()
            self.logger.info("camera01 stopped.")
            self.response.success = True
        return self.response

    def camera02_enable(self, req):
        if req.enable:
            color_check = rospy.wait_for_message(
                "/camera02/color/image_raw", Image, timeout=None
            )
            self.camera02_launch = subprocess.Popen(
                ["roslaunch", "realsense2_camera", "rs_rgbd_l515.launch"]
            )
            if color_check:
                self.response.success = True
                self.logger.info("camera01 launched.")
            else:
                self.response.success = False
                self.logger.info("camera01 launch failed.")
        else:
            self.camera02_launch.terminate()
            self.logger.info("camera01 stopped.")
            self.response.success = True
        return self.response


def get_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--avoid_cam", type=str, help="Path to Camera Avoid (exclude HOME directory)"
    )
    parser.add_argument(
        "--cam1", type=str, help="Path to Camera 1 (exclude HOME directory)"
    )
    parser.add_argument(
        "--cam2", type=str, help="Path to Camera 2 (exclude HOME directory)"
    )
    args = parser.parse_args()

    return args


def main():
    args = get_arguments()
    logger = ROSLogger()

    camera_manager = CameraManager(args, logger)

    rospy.init_node("vs_camera_manager")
    camera_manager.logger.info("vs_camera_manager node created.")

    rospy.Service(
        "/camera/launch/enable", LaunchState, camera_manager.avoid_camera_enable
    )
    rospy.Service(
        "/camera01/launch/enable", LaunchState, camera_manager.camera01_enable
    )
    rospy.Service(
        "/camera02/launch/enable", LaunchState, camera_manager.camera02_enable
    )
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.signal_shutdown("vs_camera_manager shutdown.")
