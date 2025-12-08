from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os

def generate_launch_description():

    # camera_stream.py 절대경로 찾기
    package_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    script_path = os.path.join(package_dir, "ppor_camera", "camera_stream.py")

    return LaunchDescription([
        ExecuteProcess(
            cmd=["python3", script_path],
            output="screen",
        )
    ])
