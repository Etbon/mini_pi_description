from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
  # Data input 
  description_pkg = "mini_pi_description"
  xacro_path = "xacro/mini_pi.xacro"

  # Xacro file 
  xacro_file = PathJoinSubstitution([FindPackageShare(description_pkg), xacro_path])

  # Runtime toggle 
  use_sim_time = DeclareLaunchArgument(
    "use_sim_time",default_value="false",
    description="Set true only if a /clock is running (e.g Gazebo)."
  )  

  # Xacro -> Urdf
  robot_description = {
    "robot_description": ParameterValue(
      Command([FindExecutable(name="xacro"), " ", xacro_file]),
      value_type=str
    )
  }

  # === Only if launch isolated === 

  #  # Publish TF transform  
  #  robot_state_publisher = Node(
  #    package="robot_state_publisher",
  #    executable="robot_state_publisher",
  #    parameters=[robot_description, 
  #                {"use_sim_time": LaunchConfiguration("use_sim_time")}
  #    ],
  #    output="screen"
  #  )

  # Launch RViz
  rviz = Node(
    package="rviz2",
    executable="rviz2",
    arguments=["-d", PathJoinSubstitution([
      FindPackageShare(description_pkg), "rviz", "mini_pi_config.rviz"])
    ],
    output="screen"
  ) 

  return LaunchDescription([
    use_sim_time,
    # robot_state_publisher,
    rviz
  ])