from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ppor_arduino_bridge",
            executable="arduino_bridge_node",
            name="arduino_bridge",
            parameters=[{"port": "/dev/ttyUSB0", "baudrate": 115200}],
        ),
        Node(
            package="ppor_app_bridge",
            executable="app_bridge_node",
            name="app_bridge",
        ),
        Node(
            package="ppor_tof_reader",
            executable="tof_reader",
            name="tof_reader",
            output="screen",
        ),
        Node(
            package="ppor_imu_reader",
            executable="imu_yaw_node",
            name="imu_yaw_node",
        ),

        Node(
            package="ppor_rotate_mapper",
            executable="rotate_mapper",
            name="rotate_mapper",
        ),
        Node(
            package="ppor_supervisor",
            executable="supervisor",
            name="supervisor",
        ),
        # 나중에 slam_toolbox, robot_state_publisher 등 추가
    ])
