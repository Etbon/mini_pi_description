from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulated time from /clock'
    )

    # Paths to your existing launches
    pkg = FindPackageShare('mini_pi_description')
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'gz_spawn.launch.py'])  
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'mini_pi_model.launch.py']) 
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    # Start RViz a moment after sim so TF/robot_description exist
    delayed_rviz = TimerAction(
        period=2.0, 
        actions=[rviz_launch]
    )

    return LaunchDescription([
        use_sim_time,
        sim_launch,       # starts Gazebo, robot_state_publisher, controllers
        delayed_rviz      # then starts RViz
    ])
