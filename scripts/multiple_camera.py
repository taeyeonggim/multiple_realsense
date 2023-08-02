#!/usr/bin/env python3.7
import rospy
import subprocess
import argparse


from plaif_msgs.srv import CameraCapture, CameraCaptureResponse
from plaif_msgs.msg import CameraData
from ros_logger import ROSLogger
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs.msg import CameraInfo
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest,SetBool, SetBoolResponse, SetBoolRequest
import time


class Camera:
    def __init__(
        self,
        logger:ROSLogger,
        lauch_file_name:str = 'rs_rgbd_l515.launch',
        cam_name:str = 'camera01',
        cam_switch_service_name:str = ''

        ) -> None:
        self.logger = logger

        self.cam_name = cam_name
        self.launch_file_name = lauch_file_name

        self.camera_info_topic_name = f'/{cam_name}/color/camera_info'
        self.camera_info:CameraInfo = None

        self.color_topic_name = f'/{cam_name}/color/image_raw'
        self.points_topic_name = f'/{cam_name}/depth_registered/points'

        self.enable_service_name = f'/{cam_name}/enable'
        self.reset_service_name = f'/{cam_name}/realsense2_camera/reset'

        self.realsense_enable_service_call:rospy.ServiceProxy = None
        self.realsense_reset_service_call:rospy.ServiceProxy = None

        self.state = False
        self.launch_process:subprocess.Popen = None

        self.color_data:Image = None
        self.point_data:PointCloud2 = None

        rospy.Service(
            f"/camera_manager/{cam_name}/capture", CameraCapture, self.get_cam_data
        )

        self.camera_manager_switch = rospy.ServiceProxy(cam_switch_service_name, Empty)

    def launch_run(self):
        """Run lauch file

        Initialize the enable and reset service calls
        """
        self.logger.info(f'{self.cam_name} turning on... Please reboot if more than 5 minutes')
        self.launch_process = subprocess.Popen(
            ["roslaunch", "realsense2_camera", self.launch_file_name],
            stdout=subprocess.DEVNULL
        )

        try:
            self.camera_info = rospy.wait_for_message(self.camera_info_topic_name, CameraInfo, timeout=30)
        except:
            self.reset()
    
                
        self.realsense_enable_service_call = rospy.ServiceProxy(self.enable_service_name, SetBool)
        self.realsense_reset_service_call = rospy.ServiceProxy(self.reset_service_name, Empty)

        self.state = True
        self.logger.info(f'{self.cam_name} turned on')

    def enable(self, req):
        """
        """
        success = False
        
        if req:
            if self.state ==False:
                response = self.realsense_enable_service_call(True)
                success = response.success
                self.state = True

        else:
            if self.state == True:
                response = self.realsense_enable_service_call(False)
                success = response.success
                self.state = False
        
        if not success:
            success = self.reset()

        return success
    
    def get_cam_data(self, req):
        """ get the color and points data from camera
        
        This is under the condition that self.state is True
        """
        response = CameraCaptureResponse()
        cam_data = CameraData()
        
        success = False        
        
        try:
            if not self.state:
                self.camera_manager_switch(EmptyRequest())
            
            self.color_data = rospy.wait_for_message(self.color_topic_name, Image, timeout=3.0)
            self.point_data = rospy.wait_for_message(self.points_topic_name, PointCloud2, timeout=3.0)
            success = True
            
        except:
            success = self.reset(True)
            
        if success:
            cam_data.color = self.color_data
            cam_data.pointcloud = self.point_data
            cam_data.camera_info = self.camera_info  
                     
            response.data = cam_data
            
        response.success = success
    
        return response

    def reset(self, check_rgbd=False):
        """reset the camera
        
        This resets the camera and try to get the needed data again
        If the required data acquisition fails under reset, the camera process is stopped
        """
        self.logger.info(f'{self.cam_name} resetting!!!')
        
        success = True

        try:
            self.realsense_reset_service_call(EmptyRequest())
            time.sleep(5)

            rospy.wait_for_service(self.reset_service_name, timeout=60.0)            

            self.camera_info = rospy.wait_for_message(self.camera_info_topic_name, CameraInfo, timeout=30.0)

            if check_rgbd:
                self.color_data = rospy.wait_for_message(self.color_topic_name, Image, timeout=3.0)
                self.point_data = rospy.wait_for_message(self.points_topic_name, PointCloud2, timeout=3.0)\
                    
            self.state = True

        except:
            self.logger.err(f'{self.cam_name} not working. REBOOT PC!!!')
            self.launch_process.terminate()
            success = False
            
            for i in range(10):
                self.logger.err(f'{self.cam_name} not working. REBOOT PC!!!')

                time.sleep(0.5)

        return success


class CameraManager:

    def __init__(self, args, logger):


        self.args = args
        self.logger = logger
        
        cam_switch_service_name = "/camera_manager/switch"
        
        rospy.Service(
            '/camera_manager/kill_all', Empty, self.kill_all
        )
        
        rospy.Service(
            cam_switch_service_name, Empty, self.switch_camera
        )

        self.camera01 = Camera(self.logger,
                               args.cam1_launch,
                               'camera01',
                               cam_switch_service_name
                               )

        self.camera02 = Camera(self.logger,
                               args.cam2_launch,
                               'camera02',
                               cam_switch_service_name)

        self.avoid_cam = Camera(self.logger,
                               args.avoid_cam_launch,
                               'camera')       
        
        self.init_launch()

    def init_launch(self):
        ## 카메라 A와 B를 각각 순차적으로 실행하는 코드 작성 ##

        self.camera01.launch_run()
        time.sleep(4)
        self.camera02.launch_run()
        time.sleep(4)
        self.camera02.enable(False)
        time.sleep(0.5)
        self.avoid_cam.launch_run()

    def switch_camera(self, req):
        """
        """
        if self.camera01.state:
            try:
                self.camera01.enable(False)
                time.sleep(0.5)
                self.camera02.enable(True)
            except:
                self.camera02.reset()
        else:
            try:
                self.camera02.enable(False)
                time.sleep(0.5)
                self.camera01.enable(True)
            except:
                self.camera01.reset()

        return EmptyResponse()
    
    def kill_all(self, req):
        """
        """
        self.camera01.launch_process.terminate()
        self.camera02.launch_process.terminate()
        self.avoid_cam.launch_process.terminate()
        
        return EmptyResponse()
        

def get_arguments():
    parser = argparse.ArgumentParser(
        description="recive camera, model and process configurations"
    )

    parser.add_argument(
        "--avoid_cam_launch", type=str, help="avoid cam launch file name", default='rs_rgbd_d435i.launch'
    )
    parser.add_argument(
        "--cam1_launch", type=str, help="cam1 launch file name", default="rs_rgbd_d415_cam01.launch"
    )
    parser.add_argument(
        "--cam2_launch", type=str, help="cam2 launch file name", default="rs_rgbd_d415_cam02.launch"
    )

    args, _ = parser.parse_known_args()

    return args


def main():
    rospy.init_node("vs_camera_manager")
    args = get_arguments()
    logger = ROSLogger()

    camera_manager = CameraManager(args,logger)
    camera_manager.logger.info("vs_camera_manager node created.")

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.signal_shutdown("vs_camera_manager shutdown.")
