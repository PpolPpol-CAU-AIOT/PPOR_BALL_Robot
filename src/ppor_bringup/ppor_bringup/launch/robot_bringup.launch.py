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
            package="ppor_wheel_odom",
            executable="wheel_odom_node",
            name="wheel_odom",
            parameters=[{
                "wheel_radius": 0.03,
                "wheel_base": 0.15,
                "ticks_per_rev": 1024,
            }],
        ),
        Node(
            package="ppor_tof_scan",
            executable="tof_scan_node",
            name="tof_scan",
        ),
        Node(
            package="ppor_app_bridge",
            executable="app_bridge_node",
            name="app_bridge",
        ),
        Node(
            package="slam_toolbox",
            executable="sync_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[{
                "use_sim_time": False
            }],
        ),
        Node(
            package="ppor_tof_reader",
            executable="tof_reader",
            name="tof_reader",
            output="screen",
        ),
        # 나중에 slam_toolbox, robot_state_publisher 등 추가
    ])
