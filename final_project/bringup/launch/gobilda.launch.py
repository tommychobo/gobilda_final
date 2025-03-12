from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("gobilda_robot"), "urdf", "gobilda.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("gobilda_robot"),
            "config",
            "gobilda_controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/gobilda_base_controller/cmd_vel", "/cmd_vel"),
        ],
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
        arguments=["joint_state_broadcaster", 
                   "--controller-manager",
                   "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gobilda_base_controller", 
                   "--controller-manager",
                   "/controller_manager"],
    )

    lidar_node = Node(
        package="",
        executable="",
        arguments=[],
        remappings=[
            ("", "/scan")    
        ]
    )
# rf2o laser odometry
# https://github.com/MAPIRlab/rf2o_laser_odometry/blob/ros2/launch/rf2o_laser_odometry.launch.py
    lidar_odometry_node = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[{
            "laser_scan_topic" : "/scan",
            "odom_topic" : "odom_rf2o",
            "publish_tf" : True,
            "base_frame_id" : "base_link",
            "odom_frame_id" : "odom",
            "init_pose_from_topic" : "",
            "freq" : 20.0
        }]
    )

    camera_node = Node(
        package="",
        executable="",
        arguments=[],
        remappings=[
            ("", "/image")
        ]
    )

# https://github.com/luxonis/depthai-ros/blob/humble/depthai_ros_driver/launch/camera.launch.py
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directiory(
            )])

    instructions_node = Node(
        package="",
        executable="",
        arguments=[],
        remappings=[
            ("", "/instruction")
        ]
    )

# https://docs.nav2.org ??
    mapping_node = Node(
        package="",
        executable="",
        arguments=[],
        remappings=[
            ("", "/map")
        ]
    )

    navigation_node = Node(
        package="",
        executable="",
        arguments=[],
        remappings=[
            ("", "/cmd_vel")
        ]

    # Delay start of joint_state_broadcaster after `robot_controller`
    # TODO(anyone): This is a workaround for flaky tests. Remove when fixed.
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        lidar_odometry_node,
        lidar_node,
        camera_node,
        instructions_node,
        mapping_node,
        navigation_node
    ]

    return LaunchDescription(declared_arguments + nodes)
