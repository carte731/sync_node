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

# For sending messages to JAXA-RACS2 ROS-cFS bridge
from racs2_msg.msg import RACS2UserMsg

config = "BRIDGE"

class sync_node(Node):
    
    def __init__(self):
        super().__init__("sync_node")

        if(config == "LOCAL"):
            self.localPass()
        elif(config == "BRIDGE"):
            self.bridgePass()
        else:
            self.localPass()

        self.get_logger().info("Sync-Node has started.")

    # JAXA-Bridge 
    def bridgePass(self):
        # Subscribing to YOLO-ROS detections
        self.sub_JaxaBridge_ = self.create_subscription(DetectionArray, "/yolo/detections_3d", self.bridgeCallBack, 20)
        self.sub_JaxaBridge_

        # Creating publisher for RACS2 outbound messages
        self.pub_JaxaBridge_ = self.create_publisher(RACS2UserMsg, '/RACS2Bridge', 10)

    # Local File-IO saving
    def localPass(self):
        self.imgCounter_ = 0
        self.bridge_ = CvBridge()

        # Used for RealSense Camera D435
        self.rgbSubcription_ = Subscriber(self, Image, "/camera/camera/color/image_raw")
        self.rgbdSubcription_ = Subscriber(self, Image, "/camera/camera/depth/image_rect_raw")
        
        # Used for YOLO detection
        self.yoloTrackSub_ = Subscriber(self, DetectionArray, "/yolo/detections_3d")

        # Used for simulating RGB and Depth cameras in Gazebo
        #self.rgbSubcription_ = Subscriber(self, Image, "/robot_0/rgb_camera")
        #self.rgbdSubcription_ = Subscriber(self, Image, "/robot_0/depth_camera")

        # Syncs the subscriptions
        self.img_sync = ApproximateTimeSynchronizer([self.rgbSubcription_, self.rgbdSubcription_, self.yoloTrackSub_], 100, 0.1)
        self.img_sync.registerCallback(self.imgSyncCallback)

    def bridgeCallBack(self, msg):
        # Creating a RACS2 message
        outboungMsg = RACS2UserMsg()
        
        # Converting ROS2 message to JSON string
        #detectionObject = message_to_ordereddict(msg)
        #outboungMsg.body_data = json.dumps(detectionObject)

        outboungMsg.body_data = json.dumps(msg)

        # Sending the JSON string of a ROS2 message to RACS2 bridge
        self.pub_JaxaBridge_.publish(outboungMsg)
        self.get_logger().info("Sync-Node publishing YOLO completed...")

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