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

    mock_audio_transcription_arg = DeclareLaunchArgument(
        'mock_audio_transcription',
        default_value='false',
        description="Abilita l'audio transcription con OpenAI Whisper ()"
    )
    
    
    ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='192.168.0.101',
        description='Robot IP address'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9559',
        description='Naoqi port number'
    )

    reaction_mode_arg = DeclareLaunchArgument(
        'reaction_mode',
        default_value='Autonomous',
        description='Definisce se Pepper deve reagire agli input o meno. '
                    'Utile in caso di utilizzo con un LLM'
    )

    qi_unipa_2_sensor_node = Node(
        package='qi_unipa_2',
        executable='qi_unipa_2_sensor',
        name='qi_unipa_2_sensor',
        output='screen',
        parameters=[{
            'mock_mode': LaunchConfiguration('mock_mode'),
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port'),
            'reaction_mode': LaunchConfiguration('reaction_mode')
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
        name='qi_unipa_2_speech',  # Talk+TTS
        output='screen',
        parameters=[{
            'mock_mode': LaunchConfiguration('mock_mode'),
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port')
        }]
    )

    qi_unipa_2_audio_node = Node(
        package='qi_unipa_2',
        executable='qi_unipa_2_audio',
        name='qi_unipa_2_audio',
        output='screen',
        parameters=[{
            'mock_mode': LaunchConfiguration('mock_mode'),
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port'),
            'mock_audio_transcription': LaunchConfiguration('mock_audio_transcription')
        }]
    )

    qi_unipa_2_reference_node = Node(
        package='qi_unipa_2',
        executable='qi_unipa_2_reference',
        name='qi_unipa_2_reference',
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
            'mock_mode': LaunchConfiguration('mock_mode'),
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port'),
            'reaction_mode': LaunchConfiguration('reaction_mode')
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
        mock_audio_transcription_arg,
        ip_arg,
        port_arg,
        reaction_mode_arg,
        qi_unipa_2_sensor_node,
        qi_unipa_2_movement_node,
        qi_unipa_2_speech_node,
        qi_unipa_2_audio_node,
        qi_unipa_2_server_node,
        qi_unipa_2_reference_node,
        qi_unipa_2_vision_node
    ])
