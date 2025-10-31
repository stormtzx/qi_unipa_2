from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Definizione degli argomenti di lancio
    mock_mode_arg = DeclareLaunchArgument(
        'mock_mode',
        default_value='false',
        description='Enable MOCK mode (OFFLINE TESTING: no Pepper connection)'
    )
    
    ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='192.168.0.102',
        description='Robot IP address'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9559',
        description='Naoqi port number'
    )

    
    qi_unipa_2_sensor_node = Node(
        package='qi_unipa_2',
        executable='qi_unipa_2_sensor',  
        name='qi_unipa_2_sensor',
        output='screen',
        parameters=[{
            'mock_mode': LaunchConfiguration('mock_mode'),
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port')
        }]
    )
    
    qi_unipa_2_movement_node = Node(
        package='qi_unipa_2',
        executable='qi_unipa_2_movement',  
        name='qi_unipa_2_movement',
        output='screen',
        parameters=[{
            'mock_mode': LaunchConfiguration('mock_mode'),
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port')
        }]
    )

    qi_unipa_2_speech_node = Node(
        package='qi_unipa_2',
        executable='qi_unipa_2_speech',  
        name='qi_unipa_2_speech', #Talk+TTS
        output='screen',
        parameters=[{
            'mock_mode': LaunchConfiguration('mock_mode'),
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port')
        }]
    )

    qi_unipa_2_stt_node = Node(
        package='qi_unipa_2',
        executable='qi_unipa_2_stt',  
        name='qi_unipa_2_stt', #Solo STT
        output='screen',
        parameters=[{
            'mock_mode': LaunchConfiguration('mock_mode')
        }]
    )


    qi_unipa_2_tracking_node = Node(
        package='qi_unipa_2',
        executable='qi_unipa_2_tracking',  
        name='qi_unipa_2_tracking',
        output='screen',
        parameters=[{
            'mock_mode': LaunchConfiguration('mock_mode'),
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port')
        }]
    )
    
    qi_unipa_2_tablet_node = Node(
       package='qi_unipa_2',
       executable='qi_unipa_2_tablet',  
       name='qi_unipa_2_tablet',
       output='screen',
       parameters=[{
           'mock_mode': LaunchConfiguration('mock_mode'),
           'ip': LaunchConfiguration('ip'),
           'port': LaunchConfiguration('port')
       }]
    )
    
    qi_unipa_2_server_node = Node(
        package='qi_unipa_2',
        executable='qi_unipa_2_server',  
        name='qi_unipa_2_server',
        output='screen',
        parameters=[{
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port')
        }]
    )
    
    qi_unipa_2_vision_node = Node(
        package='qi_unipa_2',
        executable='qi_unipa_2_vision',  
        name='qi_unipa_2_vision',
        output='screen',
        parameters=[{
            'mock_mode': LaunchConfiguration('mock_mode'),
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port')
        }]
    )

    return LaunchDescription([
        mock_mode_arg,
        ip_arg,
        port_arg,
        qi_unipa_2_sensor_node,
        qi_unipa_2_movement_node,
        qi_unipa_2_speech_node,
        qi_unipa_2_stt_node,
        qi_unipa_2_tracking_node,
        qi_unipa_2_tablet_node,
        qi_unipa_2_server_node,
        qi_unipa_2_vision_node
    ])
