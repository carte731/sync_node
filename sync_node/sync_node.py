import rclpy
from rclpy.node import Node

import os
import cv2
import json
from std_msgs.msg import String
from sensor_msgs.msg import Image
from yolo_msgs.msg import DetectionArray
from cv_bridge import CvBridge, CvBridgeError
from rosidl_runtime_py.convert import message_to_ordereddict
from message_filters import Subscriber, ApproximateTimeSynchronizer 

class sync_node(Node):
    
    def __init__(self):
        super().__init__("sync_node")

        self.imgCounter_ = 0
        self.bridge_ = CvBridge()

        # Used for RealSense Camera D435
        self.rgbSubcription_ = Subscriber(self, Image, "/camera/camera/color/image_raw")
        #self.rgbdSubcription_ = Subscriber(self, Image, "/camera/camera/depth/image_rect_raw")
        self.rgbaSubcription_ = Subscriber(self, Image, "/camera/camera/aligned_depth_to_color/image_raw")

        # Used for simulating RGB and Depth cameras in Gazebo - 
        # will functionalize with config/param file
        #self.rgbSubcription_ = Subscriber(self, Image, "/robot_0/rgb_camera")
        #self.rgbdSubcription_ = Subscriber(self, Image, "/robot_0/depth_camera")

        # Used for YOLO object detection
        #self.yoloTrackSub_ = Subscriber(self, DetectionArray, "/yolo/detections")
        #self.yoloTrackSub_ = Subscriber(self, DetectionArray, "/yolo/tracking")
        self.yoloTrackSub_ = Subscriber(self, DetectionArray, "/yolo/detections_3d")

        # Used if the user would like to save the images with the YOLO bounding box - 
        # will functionalize with config/param file
        #self.yoloDebugImgSub_ = Subscriber(self, Image, "/yolo/dbg_image")

        # Syncs the subscriptions
        #self.img_sync = ApproximateTimeSynchronizer([self.rgbSubcription_, self.rgbdSubcription_, self.rgbaSubcription_, self.yoloTrackSub_], 100, 0.1)
        #self.img_sync = ApproximateTimeSynchronizer([self.rgbSubcription_, self.rgbdSubcription_, self.yoloTrackSub_], 100, 0.1)
        self.img_sync = ApproximateTimeSynchronizer([self.rgbSubcription_, self.rgbaSubcription_, self.yoloTrackSub_], 100, 0.1)
        self.img_sync.registerCallback(self.imgSyncCallback)

        self.get_logger().info("Sync-Node has started.")

    #def imgSyncCallback(self, rgbMsg, rgbDepthMsg, rgb_depth_AlignedMsg, yoloTrack):
    #def imgSyncCallback(self, rgbMsg, rgbDepthMsg, yoloTrack):
    def imgSyncCallback(self, rgbMsg, rgb_depth_AlignedMsg, yoloTrack):

        # Used for filing naming
        self.currentImg_ = self.imgCounter_

        # Don't save and discard empty detections
        if(len(yoloTrack.detections) == 0):
            return(None)

        try:
            # Convert your ROS Image message to OpenCV2
            rgbcv2_img = self.bridge_.imgmsg_to_cv2(rgbMsg, "bgr8")
            #rgbDcv2_image = self.bridge_.imgmsg_to_cv2(rgbDepthMsg, desired_encoding='passthrough')
            alignedcv2_image = self.bridge_.imgmsg_to_cv2(rgb_depth_AlignedMsg, desired_encoding='passthrough')
        except CvBridgeError as e:
            self.get_logger().error(e)
        else:

            # Tries to create a new directory to save the files to
            try:
                os.mkdir("/root/img_data/Output_" + str(self.imgCounter_))
            except FileExistsError:
                # directory already exists
                pass

            # Save your OpenCV2 image as a jpeg and pngs
            cv2.imwrite('/root/img_data/Output_' + str(self.imgCounter_) + '/camera_image.jpeg', rgbcv2_img)
            #cv2.imwrite('/root/img_data/Output_' + str(self.imgCounter_) + '/depth_image.png', rgbDcv2_image)
            cv2.imwrite('/root/img_data/Output_' + str(self.imgCounter_) + '/aligned_image.png', alignedcv2_image)

            # Saves YOLO message as a JSON and save it to a .json text file
            with open('/root/img_data/Output_' + str(self.imgCounter_) + '/trackOutput.json', 'a') as fileIO:
                trackDict = message_to_ordereddict(yoloTrack)
                json.dump(trackDict, fileIO, indent=5)
            
            # Creates an empty hidden text file, marking the directory and files safe to delete from cFS
            with open('/root/img_data/Output_' + str(self.imgCounter_) + '/.safeToDelete.safe', 'w') as fileIO:
                pass

            self.imgCounter_ += 1

        finally:
            self.get_logger().info("sync-image Image-CallBack completed...")

def main(args=None):
  rclpy.init(args = args)  
  
  node = sync_node()
  
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == "__main__":
    main()