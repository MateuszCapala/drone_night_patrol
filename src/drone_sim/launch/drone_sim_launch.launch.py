from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
import os

def generate_launch_description():
    # Ścieżki bazowe
    px4_path = os.path.expanduser("~/Desktop/sem2_magisterka/drony/PX4/PX4-Autopilot")
    drone_sim_path = os.path.expanduser("~/Desktop/sem2_magisterka/drony/drone_night_patrol")

    # Model YOLO
    yolo_model_path = os.path.join(
        drone_sim_path,
        "yolo_thermal/person_detection/weights/best.pt"
    )

    return LaunchDescription([

        ExecuteProcess(
            cmd=[
                "bash", "-c",
                (
                    "PX4_HOME_LAT=51.507912 "
                    "PX4_HOME_LON=23.616133 "
                    "PX4_HOME_ALT=10 "
                    "make px4_sitl_default gazebo-classic"
                    #"make px4_sitl gazebo-classic_typhoon_h480 iris "
                )
            ],
            cwd=px4_path,
            output="screen"
        ),

        ExecuteProcess(
            cmd=[
                "ros2", "run", "mavros", "mavros_node",
                "--ros-args",
                "-p", "fcu_url:=udp://:14540@localhost:14557"
            ],
            output="screen"
        ),

        ExecuteProcess(
            cmd=[
                "ros2", "run", "drone_sim", "pose_to_tf"
            ],
            cwd=drone_sim_path,
            output="screen"
        ),

        ExecuteProcess(
            cmd=[
                "ros2", "run", "drone_sim", "human_position_publisher",
                "--ros-args",
                "-p", "target_frame:=map",
                "-p", "follow_height:=5.0"
            ],
            cwd=drone_sim_path,
            output="screen"
        ),

        ExecuteProcess(
            cmd=[
                "ros2", "run", "drone_sim", "human_detector",
                "--ros-args",
                "-p", f"model_path:={yolo_model_path}",
                "-p", "frame_skip:=2"
            ],
            cwd=drone_sim_path,
            output="screen"
        )
    ])
