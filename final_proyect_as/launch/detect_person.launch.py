# Copyright 2024 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('camera')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    pkg_yolo_dir = get_package_share_directory('yolo_bringup')
    yolo_launch_file = os.path.join(pkg_yolo_dir, 'launch', 'yolo.launch.py')

    yolo_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(yolo_launch_file),
        launch_arguments={
            "model": LaunchConfiguration("model", default="yolov8m.pt"),
            "tracker": LaunchConfiguration("tracker", default="bytetrack.yaml"),
            "device": LaunchConfiguration("device", default="cpu"),
            "enable": LaunchConfiguration("enable", default="True"),
            "threshold": LaunchConfiguration("threshold", default="0.5"),
            "input_image_topic": LaunchConfiguration("input_image_topic", default="/color/image_raw"),
            "image_reliability": LaunchConfiguration("image_reliability", default="1"),
            "namespace": LaunchConfiguration("namespace", default="yolo"),
        }.items()
    )

    detector_yolo = Node(package='camera',
                        executable='yolo_detection',
                        output='screen',
                        parameters=[param_file],
                        remappings=[
                          ('input_detection', '/yolo/detections'),
                          ('output_detection_2d', '/detection_2d')
                        ])

    final_proyect_as = Node(
                    package='final_proyect_as',
                    executable='final_proyect',
                    output='screen'
                    )

    ld = LaunchDescription()
    ld.add_action(final_proyect_as)
    ld.add_action(detector_yolo)
    ld.add_action(yolo_launcher)

    return ld
