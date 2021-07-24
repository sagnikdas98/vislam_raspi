#python3 -m pip install picamera
#sudo pip3 install adafruit-circuitpython-mpu6050
#TODO: Time info


import time

#imu imports
import board
import busio
import adafruit_mpu6050	

#camera imports
import io
from picamera.array import PiRGBArray
from picamera import PiCamera

#ros2 imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped
#

class DataPublisherPi(Node):

	def __init__(self):
		super().__init__('data_publisher')


		#create publishers
		self.info_publisher_ = self.create_publisher(String, 'pi_info', 10)
		self.imu_twist_publisher_ = self.create_publisher(TwistStamped, 'imu/data', 10)
		self.image_publisher = self.create_publisher( CompressedImage, "image/compressed", 5)

		self.initCamera()
		self.initIMU()


		timer_period_imu = 0.1 
		timer_period_camera = 0.5  # seconds
		
		self.timer_imu = self.create_timer(timer_period_imu, self.publishIMU)
		self.timer_camera = self.create_timer(timer_period_camera, self.publishImage)
		self.i = 0


	def initCamera(self):

		self.get_logger().info('Initializing Camera ')

		self.image_msg = CompressedImage()

		#load parameter
		self.framerate = self.parameters["BasicParam"]["framerate"]
		self.res_width = self.parameters["BasicParam"]["res_width"]
		self.res_height = self.parameters["BasicParam"]["res_height"]
		self.vertical_flip = self.parameters["BasicParam"]["vertical_flip"]
		self.horizontal_flip = self.parameters["BasicParam"]["horizontal_flip"]

		#create Picamera object
		self.picamera = PiCamera()
		self.picamera.resolution = (self.res_width,self.res_height)
		self.picamera.vflip = self.vertical_flip 
		self.picamera.hflip = self.horizontal_flip 
		self.picamera.framerate = self.framerate
		self.compress_stream = io.BytesIO()
		self.get_logger().info('Initialized Camera ')


	def initIMU(self):

		self.get_logger().info('Initializing IMU ')

		self.imu_msg = TwistStamped()
		
		self.x_accel_offset = 0
		self.y_accel_offset = 0
		self.z_accel_offset = 0
		self.x_gyro_offset = 0
		self.y_gyro_offset = 0
		self.z_gyro_offset = 0


		i2c = busio.I2C(board.SCL, board.SDA)
		self.mpu = adafruit_mpu6050.MPU6050(i2c)
	

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

		self.get_logger().info('Initialized IMU ')


	def mpu6050_caliberation(self):

		meanax = 0
		meanay = 0
		meanaz = 0
		meangx = 0
		meangy = 0
		meangz = 0

		no_sample = 200

		for _ in range(no_sample):
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



	def publishImage(self):

		# self.i += 1

		# Clear stream
		self.compress_stream.seek(0)
		self.compress_stream.truncate()
		
		#TODO: grab_time time current time 


		self.picamera.capture(self.compress_stream, format="jpeg",use_video_port=True,thumbnail=None)

		stream = self.compress_stream
		stream.seek(0)
		stream_data = stream.getvalue()
		
		# Generate compressed image
		
		self.image_msg.format = "jpeg"
		self.image_msg.data = stream_data
		
		# self.image_msg.header.stamp = 0#TODO img_ts
		self.image_msg.header.frame_id = "camera"

		self.image_publisher.publish(self.image_msg)
		self.get_logger().info('Publishing: ')

	def publishIMU(self):

		accel = self.mpu.acceleration
		gyro = self.mpu.gyro

		#substract offset
		accel[0] -= self.x_accel_offset
		accel[1] -= self.y_accel_offset
		accel[2] -= self.z_accel_offset

		gyro[0] -= self.x_gyro_offset
		gyro[1] -= self.y_gyro_offset
		gyro[2] -= self.z_gyro_offset

		#assign individual x,y,z if vector assignment doesnt work
		self.imu_msg.Twist.linear = accel
		self.imu_msg.Twist.angular = gyro
		# self.msg.Header.stamp  = self.get_clock().now()
		self.imu_msg.Header.frame_id = 'imu'

		self.imu_twist_publisher_.publish(self.msg)
		
		


def main(args=None):
	rclpy.init(args=args)

	data_publisher = DataPublisherPi()
	rclpy.spin(data_publisher)

	data_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()


