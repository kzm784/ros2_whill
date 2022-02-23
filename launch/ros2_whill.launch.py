import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro

def generate_launch_description():
    ros2_whill_dir =get_package_share_directory('ros2_whill')
    xacro_path=os.path.join(ros2_whill_dir,'xacro','modelc.xacro')
    
    whill_controller_node = Node(
        package='ros2_whill',
        executable='whill_modelc_controller',
        name='whill_modelc_controller',
        output='screen')

    doc =xacro.process_file(xacro_path)
    robot_desc=doc.toprettyxml(indent='  ')
    f=open(xacro_path,'w')
    f.write(robot_desc)
    f.close()
    robot_model_node =Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        arguments=[xacro_path],
        remappings=[('joint_states', '/whill/states/jointState')])

    jointstate_node =Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        remappings=[('joint_states', '/whill/states/jointState')]
        )

    whill_pablisher_node = Node(
        package='ros2_whill',
        executable='whill_modelc_publisher',
        name='whill_modelc_publisher',
        output='screen')       
    
    decleare_param_file_cmd =DeclareLaunchArgument(
        'params',
        default_value=os.path.join(
            ros2_whill_dir,'params','sample_param.yaml'),
        description='Full path to the ros2 whill param file to use')

    # Create the launch description and populate
    ld = LaunchDescription()

    #add launch arguments to the launch description of node
    ld.add_action(whill_controller_node)
    ld.add_action(whill_pablisher_node)
    ld.add_action(robot_model_node)
    ld.add_action(jointstate_node)

    # Add launch arguments to the launch description
    ld.add_action(decleare_param_file_cmd)

    return ld