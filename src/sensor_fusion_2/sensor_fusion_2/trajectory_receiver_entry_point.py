#!/usr/bin/env python3

import rclpy
from sensor_fusion_2.trajectory_receiver_node import TrajectoryReceiverNode

def main():
    rclpy.init()
    
    node = TrajectoryReceiverNode()
    
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