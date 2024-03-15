import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('final_challenge'),
            'config',
            'params.yaml'
        )

    generation_node = Node(
        package = 'final_challenge',
        executable = 'setpoint',
        output = 'screen',
        parameters = [config]
    )

    rqt_plot_node = Node(
        package = 'rqt_plot',
        executable = 'rqt_plot',
        parameters = [{'args': '/micro_ros_esp32/setpoint'}] 
     )

    # SE AGREGA EL RQT_PLOT DESPUES EN LA LISTA
    ld = LaunchDescription([generation_node,rqt_plot_node])
    return ld