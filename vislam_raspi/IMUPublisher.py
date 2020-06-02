#sudo pip3 install adafruit-circuitpython-mpu6050

#TODO: caliberation of z axis, time calc


import board
import busio
import adafruit_mpu6050		
from time    
from scipy.spatial.transform import Rotation as R   

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped



class IMUPublisher(Node):


    def __init__(self):
        super().__init__('imu__mpu6050_publisher')



        self.imu_twist_publisher_ = self.create_publisher(TwistStamped, 'imu/data', 10)
        self.info_publisher_ = self.create_publisher(String, 'pi_info', 10)

        self.msg = TwistStamped()
        
        self.x_accel_offset = 0
        self.y_accel_offset = 0
        self.z_accel_offset = 0
        self.x_gyro_offset = 0
        self.y_gyro_offset = 0
        self.z_gyro_offset = 0


        i2c = busio.I2C(board.SCL, board.SDA)
        self.mpu = adafruit_mpu6050.MPU6050(i2c)
    
        

        # self.pinSDA = self.parameters["pin"]["SDA"]
        # self.pinSCL = self.parameters["pin"]["SCL"]

        self.bool_caliberate_imu = self.parameters["offsets"]["caliberate_imu"]

        if self.bool_caliberate_imu:
            self.get_logger().info('Caliberating IMU')
            self.mpu6050_caliberation()
            self.get_logger().info('IMU Caliberation Complete')
        else:
            self.x_accel_offset = self.parameters["offsets"]["x_accel_offset"]
            self.y_accel_offset = self.parameters["offsets"]["y_accel_offset"]
            self.z_accel_offset = self.parameters["offsets"]["z_accel_offset"]
            self.x_gyro_offset = self.parameters["offsets"]["x_gyro_offset"]
            self.y_gyro_offset = self.parameters["offsets"]["y_gyro_offset"]
            self.z_gyro_offset = self.parameters["offsets"]["z_gyro_offset"]
            self.enable_debug_output = self.parameters["offsets"]["enable_debug_output"]
            self.get_logger().info('Loaded Caliberationfrom Parameter Server')

        

        
        

    
        timer_period = 0.1  # seconds
        self.get_logger().info('IMU Publisher Started')
        self.timer = self.create_timer(timer_period, self.read_mpu6050)
        


    def mpu6050_caliberation(self):

        meanax = 0
        meanay = 0
        meanaz = 0
        meangx = 0
        meangy = 0
        meangz = 0

        no_sample = 200

        for i in range(no_sample):
            accel = self.mpu.acceleration
            gyro = self.mpu.gyro
            meanax += accel[0]
            meanay += accel[1]
            meanaz += accel[2]
            meangx += gyro[0]
            meangy += gyro[1]
            meangz += gyro[2]

            time.sleep(0.1)
              
  
        self.x_accel_offset = meanax/no_sample
        self.y_accel_offset = meanay/no_sample
        self.z_accel_offset = meanaz/no_sample
        self.x_gyro_offset = meangx/no_sample
        self.y_gyro_offset = meangy/no_sample
        self.z_gyro_offset = meangz/no_sample

    
        
    def read_mpu6050(self):


        accel = self.mpu.acceleration
        gyro = self.mpu.gyro

        #substract offset
        accel[0] -= self.x_accel_offset
        accel[1] -= self.y_accel_offset
        accel[2] -= self.z_accel_offset

        gyro[0] -= self.x_gyro_offset
        gyro[1] -= self.y_gyro_offset
        gyro[2] -= self.z_gyro_offset

        #convet degrees to radians
        quatenion = IMUPublisher.degreeToRadians(gyro)
        #TODO


        #assign individual x,y,z if vector assignment doesnt work
        self.msg.Twist.linear = accel
        self.msg.Twist.angular = gyro
        self.msg.Header.stamp  = self.get_clock().now()
        self.msg.Header.frame_id = 'imu'

        self.imu_twist_publisher_.publish(self.msg)
        

    def degreeToRadians(gyro):
        

        r = R.from_euler('XYZ',gyro,degrees=True)
        return r.as_quat()
        
        


    def timer_callback(self):
        pass
        
        

    



def main(args=None):
    rclpy.init(args=args)

    imu_publisher = IMUPublisher()

    rclpy.spin(imu_publisher)
    self.get_logger().info('IMU Publisher Destroyed')
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()