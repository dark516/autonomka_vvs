from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Основной симулятор
    simulator = Node(
        package='simple_robot_sim',
        executable='simulator_node',
        name='simulator',
        output='screen',
        parameters=[{
            'field_size': 10.0,
            'window_size': 1000,
            'publish_rate': 30.0
        }]
    )
    
    # Публикатор данных с тестовыми значениями
    data_publisher = Node(
        package='simple_robot_sim',
        executable='data_publisher_node',
        name='data_publisher',
        output='screen',
        parameters=[{
            'publish_rate': 30.0,
            'enable_test_data': True  # Включаем тестовые данные
        }]
    )
    
    # Тестовый публикатор команд (с задержкой)
    cmd_publisher = Node(
        package='simple_robot_sim',
        executable='test_cmd_vel_publisher',
        name='test_cmd_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        simulator,
        data_publisher,
        TimerAction(
            period=1.0,
            actions=[cmd_publisher]
        )
    ])
