from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Declare Arguments
        DeclareLaunchArgument("joy_config", default_value="xbox"),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        DeclareLaunchArgument("joy_topic", default_value="joy"),

        # Joystick Node
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[{
                "dev": LaunchConfiguration("joy_dev"),
                "deadzone": 0.3,
                "autorepeat_rate": 20,
            }],
            remappings=[("/joy", LaunchConfiguration("joy_topic"))]
        ),

        # Teleop Twist Joy Node with Separate Y and Z Control (Forced Scaling)
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy",
            output="screen",
            parameters=[{
                "axis_linear": {"x": 1, "y": 0},  # Ensure Y stays on axis 2
                "scale_linear": {"x": 1.0, "y": 1.0},  # Force scaling from 0 to 1

                "axis_angular": {"z": 2},  # Ensure Z is mapped separately to axis 6
                "scale_angular": {"z": 1.0},  # Force angular scaling to be between 0-1

                "enable_button": 0,

                "publish_joy": {
                    "axes": [1, 2, 3],  # Ensure Y and Z are separate axes
                    "buttons": [1]
                },

                "expect_cmd_vel": {
                    "linear": {"x": 0.4, "y": 1.0, "z": 0},  # Force Y max range to 1
                    "angular": {"x": 0, "y": 0, "z": 1.0}  # Force Z max range to 1
                }
            }]
        )
    ])
