from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, AppendEnvironmentVariable
from launch.substitutions import Command, FindExecutable
from launch.event_handlers import OnProcessStart, OnProcessExit
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
  probe_script_path = os.path.join(pkg_path, "scripts", "controller_manager_probe.py") # Full path to the controller manager probe script

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
        # Range sensors published by Gazebo
        "/ultra_front/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        "/ultra_right/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        "/ultra_back/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        "/ultra_left/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
        "/lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",

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

  # create a launch action that runs helper python script
  controller_manager_probe = ExecuteProcess(
    cmd=[FindExecutable(name="python3"), probe_script_path],
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
          "--controller-manager-timeout", "30.0",
      ],
      output="screen",
  )

  diff_drive_spawner = Node(
      package="controller_manager",
      executable="spawner",
      arguments=[
          "diff_drive_controller",
          "--controller-manager", "/controller_manager",
          "--controller-manager-timeout", "30.0",
      ],
      output="screen",
  )

  probe_after_spawn = RegisterEventHandler(
    OnProcessExit(
      target_action=spawn,
      on_exit=[controller_manager_probe],
    ),
  )

  joint_state_after_probe = RegisterEventHandler(
    OnProcessExit(
      target_action=controller_manager_probe,
      on_exit=[joint_state_spawner]
    )
  )

  diff_drive_after_joint_state = RegisterEventHandler(
    OnProcessExit(
      target_action=joint_state_spawner,
      on_exit=[diff_drive_spawner],
    ),
  )

  return LaunchDescription([
    set_gz_resource,
    gz,
    gz_bridge,
    rsp,
    spawn_after_gz,
    probe_after_spawn,
    joint_state_after_probe,
    diff_drive_after_joint_state
  ])
