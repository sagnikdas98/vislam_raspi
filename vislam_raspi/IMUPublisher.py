#sudo pip3 install adafruit-circuitpython-mpu6050
from MPU6050 import MPU6050
		
from time import sleep         

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion





class IMUPublisher(Node):


    def __init__(self):
        super().__init__('imu__mpu6050_publisher')


        self.imu_twist_publisher_ = self.create_publisher(Twist, 'imu/data', 10)
        self.info_publisher_ = self.create_publisher(String, 'pi_info', 10)

        
        self.x_accel_offset = 0
        self.y_accel_offset = 0
        self.z_accel_offset = 0
        self.x_gyro_offset = 0
        self.y_gyro_offset = 0
        self.z_gyro_offset = 0
        self.enable_debug_output = False
        
        self.mpu = None
        self.packet_size = None

        self.pinSDA = self.parameters["pin"]["SDA"]
        self.pinSCL = self.parameters["pin"]["SCL"]

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

        self.mpu6050_init()

        
        

    
        timer_period = 0.5  # seconds
        self.get_logger().info('IMU Publisher Started')
        self.timer = self.create_timer(timer_period, self.read_mpu6050)
        


    def mpu6050_caliberation(self):
        from MPU6050_cal import Calib
        cal = Calib()
        xaloff, yaloff, zaloff, xgyoff, ygyoff, zgyoff = cal.imu_calib()
        self.x_accel_offset = xaloff
        self.y_accel_offset = yaloff
        self.z_accel_offset = zaloff
        self.x_gyro_offset = xgyoff
        self.y_gyro_offset = ygyoff
        self.z_gyro_offset = zgyoff

    def mpu6050_init(self):
        i2c_bus = 1
        device_address = 0x68
        self.mpu = MPU6050(i2c_bus, device_address, self.x_accel_offset, self.y_accel_offset,self.z_accel_offset, self.x_gyro_offset, self.y_gyro_offset, self.z_gyro_offset,self.enable_debug_output)
 
        self.mpu.dmp_initialize()
        self.mpu.set_DMP_enabled(True)
        mpu_int_status = self.mpu.get_int_status()
        self.get_logger().info('MPU6050 init status'+mpu_int_status)

        self.packet_size = self.mpu.DMP_get_FIFO_packet_size()
        
        
    def read_mpu6050(self):
        
        FIFO_buffer = [0]*64

        FIFO_count_list = list()
    
        FIFO_count = self.mpu.get_FIFO_count()
        mpu_int_status = self.mpu.get_int_status()

        # If overflow is detected by status or fifo count we want to reset
        if (FIFO_count == 1024) or (mpu_int_status & 0x10):
            self.mpu.reset_FIFO()
            self.get_logger().info('MPU6050 FIFO Buffer Overflow')
        # Check if fifo data is ready
        elif (mpu_int_status & 0x02):
            # Wait until packet_size number of bytes are ready for reading, default
            # is 42 bytes
            while FIFO_count < self.packet_size:
                FIFO_count = self.mpu.get_FIFO_count()
            FIFO_buffer = self.mpu.get_FIFO_bytes(self.packet_size)
            accel = self.mpu.DMP_get_acceleration_int16(FIFO_buffer)
            quat = self.mpu.DMP_get_quaternion_int16(FIFO_buffer)
            grav = self.mpu.DMP_get_gravity(quat)
            roll_pitch_yaw = self.mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
            if count % 100 == 0:
                print('roll: ' + str(roll_pitch_yaw.x))
                print('pitch: ' + str(roll_pitch_yaw.y))
                print('yaw: ' + str(roll_pitch_yaw.z))
            count += 1



    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        
        

    



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