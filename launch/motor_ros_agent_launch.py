from launch import LaunchDescription
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'device',
            default_value='/dev/motor',
            description='Serial device path'
        ),

        launch_ros.actions.Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', launch.substitutions.LaunchConfiguration('device')],
            name='micro_ros_agent',
            output='screen'
        )
    ])
