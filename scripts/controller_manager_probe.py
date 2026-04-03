#!/usr/bin/env python3

import sys
import time
import rclpy 
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers

class ControllerManagerProbe(Node):
  def __init__(self):
    super().__init__("controller_manager_probe")

    # Initialize client 
    self.client = self.create_client(
      ListControllers,
      "/controller_manager/list_controllers"
    )

def main():
  rclpy.init()

  # Create the node object 
  node = ControllerManagerProbe()
  
  # Set time variables
  timeout_sec = 30.0
  start_time = time.time()

  # Loop for client is active 
  while time.time() - start_time < timeout_sec:
    if node.client.wait_for_service(timeout_sec=0.5):
      node.get_logger().info("controller_manager is ready")
      node.destroy_node()
      rclpy.shutdown()
      sys.exit(0)
    
    node.get_logger().info("Waiting for /controller_manager/list_controllers ...")
  
  node.get_logger().error("controller_manager did not become ready in time")
  node.destroy_node()
  rclpy.shutdown()
  sys.exit(1)

if __name__ == "__main__":
  main()