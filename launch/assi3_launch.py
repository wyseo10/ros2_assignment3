from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.actions

def generate_launch_description():
    number_of_sim = 5

    ld = LaunchDescription()
    for sim_id in range(0, number_of_sim):
        turtlesim_node = Node(
            namespace = 'turtlesim' + str(sim_id),
            package='turtlesim',
            executable='turtlesim_node',
            output='screen'
        )
        ld.add_action(turtlesim_node)

        controller_node = Node(
            namespace= 'turtlesim' + str(sim_id),
            package= 'ros2_tutorial',
            executable= 'cmd_publisher_node',
            output='screen'
        )
        ld.add_action(controller_node)


    simulator_node = Node(
        namespace = 'simulator',
        package='ros2_tutorial',
        executable='simulator_node',
        output='screen'
    )
    ld.add_action(simulator_node)

    rviz_node = Node(
        package='rviz2',
        namespace='rviz2',
        executable='rviz2',
        name='rviz2',
    )
    ld.add_action(rviz_node)

    return ld
