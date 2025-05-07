import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Ruta al paquete 'camera' y su archivo de parámetros
    pkg_camera_dir = get_package_share_directory('camera')
    param_file = os.path.join(pkg_camera_dir, 'config', 'params.yaml')

    # Incluir el otro launcher desde el paquete 'yolo_bringup'
    yolo_bringup_dir = get_package_share_directory('yolo_bringup')
    yolo_launch_file = os.path.join(yolo_bringup_dir, 'launch', 'yolo.launch.py')

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

    # Nodo de detección de cámara (puedes quitar esto si yolo ya se encarga)
    detector_yolo = Node(
        package='camera',
        executable='yolo_detection',
        output='screen',
        parameters=[param_file],
        remappings=[
            ('input_detection', '/yolo/detections'),
            ('output_detection_2d', '/detection_2d')
        ]
    )

    # Descripción del lanzamiento
    ld = LaunchDescription()
    ld.add_action(yolo_launcher)
    ld.add_action(detector_yolo)

    return ld
