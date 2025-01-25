import launch
import launch_ros

def generate_launch_description():
    #产生launch描述

    action_node_judger_node = launch_ros.actions.Node(
        package='judger',
        executable='judger_node',
        output='screen'
    )

    action_node_transportation_node = launch_ros.actions.Node(
        package='transportation_hub',
        executable='transportation_hub',
        output='screen'
    )

    return launch.LaunchDescription([
        #action动作
        action_node_judger_node,
        action_node_transportation_node
    ])