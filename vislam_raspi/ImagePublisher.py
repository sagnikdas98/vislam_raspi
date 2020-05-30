#python3 -m pip install picamera
import time

import numpy as np

import io
from picamera.array import PiRGBArray
from picamera import PiCamera

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage


class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')

        #load parameter
        self.framerate = self.parameters["BasicParam"]["framerate"]
        self.res_width = self.parameters["BasicParam"]["res_width"]
        self.res_height = self.parameters["BasicParam"]["res_height"]
        self.vertical_flip = self.parameters["BasicParam"]["vertical_flip"]
        self.horizontal_flip = self.parameters["BasicParam"]["horizontal_flip"]



        #create publishers
        self.info_publisher_ = self.create_publisher(String, 'pi_info', 10)
        self.image_publisher = self.create_publisher("image/compressed", CompressedImage, 5)

        self.get_logger().info('Initializing Camera ')
        #create Picamera object
        self.picamera = PiCamera()
        self.picamera.resolution = (self.res_width,self.res_height)
        self.picamera.vflip = self.vertical_flip 
        self.picamera.hflip = self.horizontal_flip 
        self.picamera.framerate = self.framerate
        self.compress_stream = io.BytesIO()
        self.get_logger().info('Initialized Camera ')


        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_image_msg)
        self.i = 0



    def publish_image_msg(self):

        # self.i += 1

        # Clear stream
        self.compress_stream.seek(0)
        self.compress_stream.truncate()
        
        #TODO: grab_time time current time 


        self.picamera.capture(self.compress_stream, format="jpeg",use_video_port=True,thumbnail=None)
        # dur = rospy.Time.now() - grab_time
        # img_ts = grab_time + (dur * 0.5)

        stream = self.compress_stream
        stream.seek(0)
        stream_data = stream.getvalue()
        
        # Generate compressed image
        image_msg = CompressedImage()
        image_msg.format = "jpeg"
        image_msg.data = stream_data
        
        image_msg.header.stamp = 0#TODO img_ts
        image_msg.header.frame_id = "camera"

        self.image_publisher.publish(image_msg)
        # self.get_logger().info('Publishing: ')


def main(args=None):
    rclpy.init(args=args)

    image_publisher = ImagePublisher()
    rclpy.spin(image_publisher)

    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


# def startCapturing(self):
#         rospy.loginfo("[%s] Start capturing." %(self.node_name))
#         while not self.is_shutdown and not rospy.is_shutdown():
#             gen =  self.grabAndPublish(self.stream,self.pub_img)
#             try:
#                 self.camera.capture_sequence(gen,'jpeg',use_video_port=True,splitter_port=0)
#             except StopIteration:
#                 pass
#             print "updating framerate"
#             self.camera.framerate = self.framerate
#             self.update_framerate=False

#         self.camera.close()
#         rospy.loginfo("[%s] Capture Ended." %(self.node_name))

# def grabAndPublish(self,stream,publisher):
#     while not self.update_framerate and not self.is_shutdown and not rospy.is_shutdown(): 
#         yield stream
#         # Construct image_msg
#         # Grab image from stream
#         stamp = rospy.Time.now()
#         stream.seek(0)
#         stream_data = stream.getvalue()
#         # Generate compressed image
#         image_msg = CompressedImage()
#         image_msg.format = "jpeg"
#         image_msg.data = stream_data

#         image_msg.header.stamp = stamp
#         image_msg.header.frame_id = self.frame_id
#         publisher.publish(image_msg)
                    
#         # Clear stream
#         stream.seek(0)
#         stream.truncate()
        
#         if not self.has_published:
#             rospy.loginfo("[%s] Published the first image." %(self.node_name))
#             self.has_published = True

#         rospy.sleep(rospy.Duration.from_sec(0.001))