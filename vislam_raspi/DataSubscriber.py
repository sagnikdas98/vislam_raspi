import numpy 

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class DataSubscriber(Node):

    def __init__(self):
        super().__init__('data_subscriber')
        self.imu_subscription = self.create_subscription(TwistStamped,'imu/data',self.imu_subscribe,10)
        self.image_subscription = self.create_subscription(CompressedImage,'image/compressed',self.image_subscribe,10)
        self.prev_imu_time


    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)



    def imu_subscribe(self, imumsg):
        #we receive 
        
        accel = imumsg.Twist.linear
        gyro = imumsg.Twist.gyro
        delta_time = imumsg.Header.stamp - self.prev_imu_time
        self.prev_imu_time = imumsg.Header.stamp


    def image_subscribe(self, imagemsg):
        pass



    


def main(args=None):
    rclpy.init(args=args)

    data_subscriber = DataSubscriber()

    rclpy.spin(data_subscriber)
    data_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
