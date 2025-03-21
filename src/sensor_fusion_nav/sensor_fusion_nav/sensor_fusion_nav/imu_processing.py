import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from math import sin, cos, pi
import tf_transformations

class IMUProcessor(Node):
    def __init__(self):
        super().__init__('imu_processor')
        
        # Create a subscription to the IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Create a transform broadcaster to publish the orientation transform
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Store the latest orientation values
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        
        self.get_logger().info('IMU Processor Node initialized')

    def imu_callback(self, msg):
        """
        Process IMU data and extract the roll, pitch, and yaw values.
        Publishes a transform that can be used to correct sensor data.
        
        Args:
            msg (Imu): IMU message containing orientation data
        """
        try:
            # Extract orientation from quaternion
            quaternion = [
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            ]
            
            # Convert quaternion to Euler angles (roll, pitch, yaw)
            euler = tf_transformations.euler_from_quaternion(quaternion)
            self.roll, self.pitch, self.yaw = euler
            
            # Log the orientation values
            self.get_logger().debug(
                f'IMU Orientation - Roll: {self.roll:.3f}, Pitch: {self.pitch:.3f}, Yaw: {self.yaw:.3f}'
            )
            
            # Create and publish transform
            self.publish_imu_transform(msg.header.stamp)
            
        except Exception as e:
            self.get_logger().error(f'Error processing IMU data: {str(e)}')

    def publish_imu_transform(self, timestamp):
        """
        Publish a transform from base_link to imu_corrected frame.
        This transform can be used to correct the position of sensor data.
        
        Args:
            timestamp (Time): The timestamp to use for the transform
        """
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'imu_corrected'
        
        # Set the translation based on pitch and roll (represents "corrected" position)
        # This is a simplified approach - in a real application, you would use the full IMU data
        # to correct for vehicle tilt and movement
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        # Set the rotation based on the IMU orientation
        q = tf_transformations.quaternion_from_euler(self.roll, self.pitch, self.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    imu_processor = IMUProcessor()
    
    try:
        rclpy.spin(imu_processor)
    except KeyboardInterrupt:
        pass
    finally:
        imu_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 