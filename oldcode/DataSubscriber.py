import numpy 

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import CompressedImage

class DataSubscriber(Node):

    def __init__(self):
        super().__init__('data_subscriber')
        self.imu_subscription = self.create_subscription(TwistStamped,'imu/data',self.imu_subscribe,10)
        self.image_subscription = self.create_subscription(CompressedImage,'image/compressed',self.image_subscribe,10)
        self.prev_imu_time

        #TODO set previous time here by waiting foe 2sec and accessing pi nodes


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)



    def imu_subscribe(self, imumsg):
        #we receive 
        
        accel = imumsg.Twist.linear
        gyro = imumsg.Twist.gyro
        delta_time = imumsg.Header.stamp - self.prev_imu_time
        self.prev_imu_time = imumsg.Header.stamp


    def image_subscribe(self, imagemsg):
        image = self.bridge.compressed_imgmsg_to_cv2(imagemsg, 'bgr8')
        pass



    


def main(args=None):
    rclpy.init(args=args)

    data_subscriber = DataSubscriber()

    rclpy.spin(data_subscriber)
    data_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
