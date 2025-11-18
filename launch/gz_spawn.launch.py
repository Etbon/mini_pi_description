from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, AppendEnvironmentVariable
from launch.substitutions import Command, FindExecutable
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  pkg_name   = "mini_pi_description"
  world_rel  = "worlds/empty_world.sdf"
  xacro_rel  = "xacro/mini_pi.xacro"
  entity_name = "mini_pi"
  x, y, z, yaw = "0.0", "0.0", "0.0", "0.0"

  pkg_path   = get_package_share_directory(pkg_name)
  world_path = os.path.join(pkg_path, world_rel)
  xacro_path = os.path.join(pkg_path, xacro_rel)

  # Gazebo meshes path 
  gz_resource_path = os.path.dirname(pkg_path)

  set_gz_resource = AppendEnvironmentVariable(
      name='GZ_SIM_RESOURCE_PATH',
      value=os.pathsep + gz_resource_path
  )

  # robot_description from xacro
  xacro_cmd = Command([FindExecutable(name="xacro"), " ", xacro_path])
  robot_description = ParameterValue(xacro_cmd, value_type=str)

  # Gazebo
  gz = ExecuteProcess(
      cmd=[FindExecutable(name="gz"), "sim", "-v", "3", "-r", world_path],
      output="screen"
  )

  # Ros Bridge 
  gz_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    name="gz_bridge",
    arguments=[
        # sensor
        "/ultra_front/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",

        # clock: BRIDGE the real GZ topic name
        "/world/empty/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
    ],
    # ROS remap so ROS sees canonical /clock
    remappings=[('/world/empty/clock', '/clock')],
    output="screen",
  )


  # robot_state_publisher (wall time)
  rsp = Node(
      package="robot_state_publisher",
      executable="robot_state_publisher",
      name="robot_state_publisher",
      parameters=[{
          "robot_description": robot_description,
          "use_sim_time": True,
      }],
      output="screen",
  )

  # spawn in Gazebo
  spawn = Node(
      package="ros_gz_sim",
      executable="create",
      arguments=[
          "-topic", "/robot_description",
          "-name", entity_name,
          "-x", x, "-y", y, "-z", z, "-Y", yaw,
      ],
      output="screen",
  )

  spawn_after_gz = RegisterEventHandler(
      OnProcessStart(
          target_action=gz,
          on_start=[spawn],
      )
  )

  # controller spawners
  joint_state_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=[
          "joint_state_broadcaster",
          "--controller-manager", "/controller_manager",
      ],
      output="screen",
  )

  diff_drive_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=[
          "diff_drive_controller",
          "--controller-manager", "/controller_manager",
      ],
      output="screen",
  )

  return LaunchDescription([
    set_gz_resource,
    gz,
    gz_bridge,
    rsp,
    spawn_after_gz,
    joint_state_spawner,
    diff_drive_spawner,
  ])
