#!/usr/bin/env python3

import rclpy
from sensor_fusion_2.mpc_controller_node import MPCControllerNode

def main():
    rclpy.init()
    
    node = MPCControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 