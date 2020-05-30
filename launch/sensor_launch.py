from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vislam_raspi',
            node_namespace='vislam',
            node_executable='IMUPublisher',
            node_name='imu__mpu6050_publisher',
            parameters=["params/raspi_params.yaml"]
        ),
        Node(
            package='vislam_raspi',
            node_namespace='vislam',
            node_executable='ImagePublisher',
            node_name='image_publisher'
            parameters=["params/camera_params.yml"]
        ),
        Node(
            package='tf2_ros',
            node_namespace='static_transform_publisher',
            node_executable='static_transform_publisher',
            node_name='imu_camera_transform_publisher',
            arguments=['0.012', '0.00', '0.00','0.0','0.0','0.0','imu','camera']
        )
        
    ])