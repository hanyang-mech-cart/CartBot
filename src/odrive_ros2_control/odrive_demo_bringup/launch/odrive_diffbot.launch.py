from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from launch.actions import ExecuteProcess

def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("odrive_demo_description"),
                    "urdf",
                    "odrive_diffbot.urdf.xacro"
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("odrive_demo_bringup"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("my_bot_description"),
            "rviz",
            "diffbot.rviz"
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
        remappings=[('diffbot_base_controller/cmd_vel_unstamped', '/cmd_vel')]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "-c", "/controller_manager"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )
    
    # usb_cam 추가
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        parameters=[os.path.join(os.path.expanduser('~/my_ws/src/usb_cam/config/params_2.yaml'))],
        remappings=[('/usb_cam/image_raw', '/image_raw')]
    )
    
    # aruco marker 추가
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node_marker',
        parameters=[{'image_topic': '/image_raw'}, {'camera_info_topic': '/camera_info'}, {'marker_size': 0.2}],
    )
    
    # aruco marker 추가
    aruco_node_camera = Node(
        package='ros2_aruco',
        executable='aruco_node',
        name='aruco_node_camera',
        parameters=[{'image_topic': '/camera/camera/color/image_raw'}, {'camera_info_topic': '/camera/camera/color/camera_info'}, {'marker_size': 0.2}],
    )
    
    # my_tf2_package 추가
    my_static_cart_tf2_broadcaster_node = Node(
            package='my_tf2_package',
            executable='my_static_cart_tf2_broadcaster',
            name='my_static_cart_tf2_broadcaster_node'
    )

    # my_tf2_package 추가
    ydlidar_ros2_driver_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node'
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
        usb_cam_node,  # 추가된 USB 카메라 노드
        aruco_node,    # 추가된 aruco 노드
        aruco_node_camera, # 추가된 aruco camera
        # my_static_cart_tf2_broadcaster_node, # 추가된 tf2
        ydlidar_ros2_driver_node, # 추가된 ydlidar
    ])
