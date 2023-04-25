from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    share_dir = get_package_share_directory('ohm_rrl_perception_utility')

    with_viewer = False
    input_image_topic = "/camera/tele/image_raw"
    img_haz_topic = "img_hazmats"

    return LaunchDescription([
        Node(
            package='find_object_2d',
            namespace='find_object_2d',
            executable='find_object_2d',
            name='hazmat_detection',
            output="screen",
            parameters=[
                {
                  "gui": with_viewer,
                  "subscribe_depth": False,
                  "objects_path": os.path.join(share_dir, "config","hazmats"),               # Path to folder containing objects to detect. 
                  "settings_path": os.path.join(share_dir, "config","hazmats","settings.ini")  # Path to settings file (*.ini).
                }
            ],
            remappings=[
                ('image', input_image_topic)
            ]
        ),
        Node(
            package='ohm_rrl_perception_utility',
            namespace='find_object_2d',
            executable='hazmat_viz_node',
            name='hazmat_viz',
            output="screen",
            parameters=[
                {
                    "input_topic": input_image_topic,
                    "viz_topic": img_haz_topic
                }
            ],
            remappings=[
            ]
        ),
    ])