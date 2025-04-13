#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, TransformStamped
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import tf2_ros
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from threading import Lock
import time


class IMUProcessor(Node):
    """
    Processes IMU data to provide vehicle orientation and motion information.
    
    This node:
    1. Subscribes to raw IMU data from sensor
    2. Filters and processes the data
    3. Provides orientation information for sensor fusion
    4. Estimates velocity through integration
    5. Publishes processed IMU data and vehicle state
    """
    def __init__(self):
        super().__init__('imu_processor')
        
        # Declare parameters
        self.declare_parameter('imu_topic', '/imu/data_raw')  # Raw IMU topic
        self.declare_parameter('use_madgwick_filter', True)   # Use Madgwick filter for orientation
        self.declare_parameter('madgwick_beta', 0.1)          # Madgwick filter parameter
        self.declare_parameter('gravity_compensation', True)  # Compensate for gravity in acceleration
        self.declare_parameter('publish_rate', 50.0)          # Rate to publish processed data (Hz)
        self.declare_parameter('gyro_bias_correction', True)  # Apply gyro bias correction
        self.declare_parameter('accel_lpf_cutoff', 5.0)       # Low-pass filter cutoff for accel (Hz)
        self.declare_parameter('velocity_reset_threshold', 0.1) # Velocity reset threshold (m/s)
        self.declare_parameter('velocity_decay_factor', 0.99)  # Velocity decay factor for stationary detection
        
        # Get parameters
        self.imu_topic = self.get_parameter('imu_topic').value
        self.use_madgwick_filter = self.get_parameter('use_madgwick_filter').value
        self.madgwick_beta = self.get_parameter('madgwick_beta').value
        self.gravity_compensation = self.get_parameter('gravity_compensation').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.gyro_bias_correction = self.get_parameter('gyro_bias_correction').value
        self.accel_lpf_cutoff = self.get_parameter('accel_lpf_cutoff').value
        self.velocity_reset_threshold = self.get_parameter('velocity_reset_threshold').value
        self.velocity_decay_factor = self.get_parameter('velocity_decay_factor').value
        
        # IMU data storage
        self.raw_imu_data = None
        self.processed_imu_data = None
        self.imu_timestamp = None
        
        # Vehicle state
        self.orientation_quaternion = [0.0, 0.0, 0.0, 1.0]  # w is last
        self.orientation_euler = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.linear_acceleration = [0.0, 0.0, 0.0]
        self.linear_velocity = [0.0, 0.0, 0.0]
        self.position = [0.0, 0.0, 0.0]
        
        # For gyro bias correction
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.bias_samples = 0
        self.calibration_samples = 100  # Number of samples to use for calibration
        self.is_calibrating = True
        
        # For low-pass filter
        self.accel_lpf_alpha = 0.0
        self.accel_filtered = [0.0, 0.0, 0.0]
        
        # For stationary detection
        self.stationary_counter = 0
        self.stationary_threshold = 10  # Number of cycles to consider stationary
        
        # Calculate low-pass filter coefficient
        dt = 1.0 / self.publish_rate
        rc = 1.0 / (2.0 * math.pi * self.accel_lpf_cutoff)
        self.accel_lpf_alpha = dt / (dt + rc)
        
        # TF buffer and broadcaster
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Locks for thread safety
        self.imu_lock = Lock()
        self.state_lock = Lock()
        
        # Publishers
        self.processed_imu_publisher = self.create_publisher(
            Imu, 
            '/imu/data',
            10
        )
        
        self.odometry_publisher = self.create_publisher(
            Odometry,
            '/odometry/imu',
            10
        )
        
        self.orientation_publisher = self.create_publisher(
            Vector3Stamped,
            '/imu/orientation',
            10
        )
        
        self.velocity_publisher = self.create_publisher(
            TwistStamped,
            '/imu/velocity',
            10
        )
        
        # Debug publishers
        self.debug_publisher = self.create_publisher(
            Float64MultiArray,
            '/imu/debug',
            10
        )
        
        # Subscribe to raw IMU data
        self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10
        )
        
        # Timer for periodic processing and publishing
        self.create_timer(1.0/self.publish_rate, self.process_and_publish)
        
        # Performance monitoring
        self.process_count = 0
        self.last_performance_time = time.time()
        self.create_timer(10.0, self.report_performance)
        
        self.get_logger().info('IMU processor initialized')
        self.get_logger().info(f'Using Madgwick filter: {self.use_madgwick_filter}')
        self.get_logger().info(f'Gravity compensation: {self.gravity_compensation}')
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'Starting calibration for {self.calibration_samples} samples...')
    
    def imu_callback(self, msg):
        """Process incoming raw IMU data"""
        with self.imu_lock:
            self.raw_imu_data = msg
            self.imu_timestamp = rclpy.time.Time.from_msg(msg.header.stamp)
            
            # If in calibration phase, accumulate gyro readings for bias estimation
            if self.is_calibrating and self.bias_samples < self.calibration_samples:
                self.gyro_bias[0] += msg.angular_velocity.x
                self.gyro_bias[1] += msg.angular_velocity.y
                self.gyro_bias[2] += msg.angular_velocity.z
                self.bias_samples += 1
                
                if self.bias_samples == self.calibration_samples:
                    # Calculate average bias
                    self.gyro_bias[0] /= self.calibration_samples
                    self.gyro_bias[1] /= self.calibration_samples
                    self.gyro_bias[2] /= self.calibration_samples
                    self.is_calibrating = False
                    self.get_logger().info(f'Calibration complete. Gyro bias: ({self.gyro_bias[0]:.5f}, {self.gyro_bias[1]:.5f}, {self.gyro_bias[2]:.5f})')
    
    def madgwick_update(self, gx, gy, gz, ax, ay, az, dt):
        """
        Update orientation using Madgwick filter algorithm
        Simplified implementation for this example
        """
        # Current orientation quaternion
        q = self.orientation_quaternion.copy()
        
        # Normalize acceleration
        norm = math.sqrt(ax*ax + ay*ay + az*az)
        if norm > 0:
            ax /= norm
            ay /= norm
            az /= norm
        
        # Estimated direction of gravity
        vx = 2 * (q[1] * q[3] - q[0] * q[2])
        vy = 2 * (q[0] * q[1] + q[2] * q[3])
        vz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
        
        # Error is cross product between estimated and measured direction of gravity
        ex = (ay * vz - az * vy)
        ey = (az * vx - ax * vz)
        ez = (ax * vy - ay * vx)
        
        # Apply feedback
        gx += self.madgwick_beta * ex
        gy += self.madgwick_beta * ey
        gz += self.madgwick_beta * ez
        
        # Integrate rate of change of quaternion
        gx *= 0.5 * dt
        gy *= 0.5 * dt
        gz *= 0.5 * dt
        
        # Apply angular velocity to quaternion
        qa = q[0]
        qb = q[1]
        qc = q[2]
        qd = q[3]
        
        q[0] += -qb * gx - qc * gy - qd * gz
        q[1] += qa * gx + qc * gz - qd * gy
        q[2] += qa * gy - qb * gz + qd * gx
        q[3] += qa * gz + qb * gy - qc * gx
        
        # Normalize quaternion
        norm = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
        if norm > 0:
            q[0] /= norm
            q[1] /= norm
            q[2] /= norm
            q[3] /= norm
            
        return q
    
    def detect_stationary(self, ax, ay, az):
        """Detect if vehicle is stationary based on acceleration variance"""
        # Simple stationary detection - look at acceleration magnitude
        accel_mag = math.sqrt(ax*ax + ay*ay + az*az)
        grav = 9.81  # m/s^2
        
        # If acceleration is close to gravity (with some tolerance)
        if abs(accel_mag - grav) < 0.5:
            self.stationary_counter += 1
        else:
            self.stationary_counter = 0
            
        return self.stationary_counter >= self.stationary_threshold
    
    def process_and_publish(self):
        """Process IMU data and publish processed data and vehicle state"""
        if self.raw_imu_data is None or self.is_calibrating:
            return
            
        with self.imu_lock, self.state_lock:
            # Extract raw IMU data
            ax = self.raw_imu_data.linear_acceleration.x
            ay = self.raw_imu_data.linear_acceleration.y
            az = self.raw_imu_data.linear_acceleration.z
            
            gx = self.raw_imu_data.angular_velocity.x
            gy = self.raw_imu_data.angular_velocity.y
            gz = self.raw_imu_data.angular_velocity.z
            
            # Apply gyro bias correction if enabled
            if self.gyro_bias_correction:
                gx -= self.gyro_bias[0]
                gy -= self.gyro_bias[1]
                gz -= self.gyro_bias[2]
            
            # Apply low-pass filter to acceleration
            self.accel_filtered[0] = self.accel_lpf_alpha * ax + (1.0 - self.accel_lpf_alpha) * self.accel_filtered[0]
            self.accel_filtered[1] = self.accel_lpf_alpha * ay + (1.0 - self.accel_lpf_alpha) * self.accel_filtered[1]
            self.accel_filtered[2] = self.accel_lpf_alpha * az + (1.0 - self.accel_lpf_alpha) * self.accel_filtered[2]
            
            # Use filtered values for processing
            ax = self.accel_filtered[0]
            ay = self.accel_filtered[1]
            az = self.accel_filtered[2]
            
            # Get time delta
            current_time = rclpy.time.Time.from_msg(self.raw_imu_data.header.stamp)
            dt = 1.0 / self.publish_rate  # Default to timer rate
            
            # Update orientation using Madgwick filter if enabled
            if self.use_madgwick_filter:
                self.orientation_quaternion = self.madgwick_update(gx, gy, gz, ax, ay, az, dt)
            else:
                # Simple quaternion integration (not recommended for real use)
                # Convert gyro to quaternion rate
                qDot = [0.0, 0.0, 0.0, 0.0]
                q = self.orientation_quaternion
                
                qDot[0] = 0.5 * (-q[1] * gx - q[2] * gy - q[3] * gz)
                qDot[1] = 0.5 * (q[0] * gx + q[2] * gz - q[3] * gy)
                qDot[2] = 0.5 * (q[0] * gy - q[1] * gz + q[3] * gx)
                qDot[3] = 0.5 * (q[0] * gz + q[1] * gy - q[2] * gx)
                
                # Integrate
                q[0] += qDot[0] * dt
                q[1] += qDot[1] * dt
                q[2] += qDot[2] * dt
                q[3] += qDot[3] * dt
                
                # Normalize
                norm = math.sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3])
                q[0] /= norm
                q[1] /= norm
                q[2] /= norm
                q[3] /= norm
                
                self.orientation_quaternion = q
            
            # Convert quaternion to Euler angles
            self.orientation_euler = euler_from_quaternion([
                self.orientation_quaternion[1],  # x
                self.orientation_quaternion[2],  # y
                self.orientation_quaternion[3],  # z
                self.orientation_quaternion[0]   # w
            ])
            
            # Store angular velocity
            self.angular_velocity = [gx, gy, gz]
            
            # Compensate for gravity if enabled
            if self.gravity_compensation:
                # Rotate gravity vector by inverse of orientation
                q = self.orientation_quaternion
                grav_x = 2 * (q[1] * q[3] - q[0] * q[2])
                grav_y = 2 * (q[0] * q[1] + q[2] * q[3])
                grav_z = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]
                
                # Compensate by removing gravity component
                ax -= grav_x * 9.81
                ay -= grav_y * 9.81
                az -= grav_z * 9.81
            
            # Store linear acceleration
            self.linear_acceleration = [ax, ay, az]
            
            # Detect if stationary
            is_stationary = self.detect_stationary(
                self.raw_imu_data.linear_acceleration.x,
                self.raw_imu_data.linear_acceleration.y,
                self.raw_imu_data.linear_acceleration.z
            )
            
            # Update velocity by integrating acceleration
            if is_stationary:
                # Vehicle is stationary, reset velocity
                self.linear_velocity = [0.0, 0.0, 0.0]
            else:
                # Integrate acceleration to get velocity
                self.linear_velocity[0] += ax * dt
                self.linear_velocity[1] += ay * dt
                self.linear_velocity[2] += az * dt
                
                # Apply decay factor to prevent velocity drift
                self.linear_velocity[0] *= self.velocity_decay_factor
                self.linear_velocity[1] *= self.velocity_decay_factor
                self.linear_velocity[2] *= self.velocity_decay_factor
                
                # Check for small velocities and reset if below threshold
                vel_mag = math.sqrt(
                    self.linear_velocity[0] * self.linear_velocity[0] +
                    self.linear_velocity[1] * self.linear_velocity[1] +
                    self.linear_velocity[2] * self.linear_velocity[2]
                )
                
                if vel_mag < self.velocity_reset_threshold:
                    self.linear_velocity = [0.0, 0.0, 0.0]
            
            # Update position by integrating velocity
            self.position[0] += self.linear_velocity[0] * dt
            self.position[1] += self.linear_velocity[1] * dt
            self.position[2] += self.linear_velocity[2] * dt
            
            # Create and publish processed IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'
            
            # Set orientation
            imu_msg.orientation.w = self.orientation_quaternion[0]
            imu_msg.orientation.x = self.orientation_quaternion[1]
            imu_msg.orientation.y = self.orientation_quaternion[2]
            imu_msg.orientation.z = self.orientation_quaternion[3]
            
            # Set angular velocity
            imu_msg.angular_velocity.x = self.angular_velocity[0]
            imu_msg.angular_velocity.y = self.angular_velocity[1]
            imu_msg.angular_velocity.z = self.angular_velocity[2]
            
            # Set linear acceleration
            imu_msg.linear_acceleration.x = self.linear_acceleration[0]
            imu_msg.linear_acceleration.y = self.linear_acceleration[1]
            imu_msg.linear_acceleration.z = self.linear_acceleration[2]
            
            # Publish processed IMU data
            self.processed_imu_publisher.publish(imu_msg)
            
            # Create and publish odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'
            
            # Set position
            odom_msg.pose.pose.position.x = self.position[0]
            odom_msg.pose.pose.position.y = self.position[1]
            odom_msg.pose.pose.position.z = self.position[2]
            
            # Set orientation
            odom_msg.pose.pose.orientation.w = self.orientation_quaternion[0]
            odom_msg.pose.pose.orientation.x = self.orientation_quaternion[1]
            odom_msg.pose.pose.orientation.y = self.orientation_quaternion[2]
            odom_msg.pose.pose.orientation.z = self.orientation_quaternion[3]
            
            # Set velocity
            odom_msg.twist.twist.linear.x = self.linear_velocity[0]
            odom_msg.twist.twist.linear.y = self.linear_velocity[1]
            odom_msg.twist.twist.linear.z = self.linear_velocity[2]
            odom_msg.twist.twist.angular.x = self.angular_velocity[0]
            odom_msg.twist.twist.angular.y = self.angular_velocity[1]
            odom_msg.twist.twist.angular.z = self.angular_velocity[2]
            
            # Publish odometry
            self.odometry_publisher.publish(odom_msg)
            
            # Create and publish orientation as Euler angles
            orient_msg = Vector3Stamped()
            orient_msg.header.stamp = self.get_clock().now().to_msg()
            orient_msg.header.frame_id = 'imu_link'
            orient_msg.vector.x = self.orientation_euler[0]  # roll
            orient_msg.vector.y = self.orientation_euler[1]  # pitch
            orient_msg.vector.z = self.orientation_euler[2]  # yaw
            
            self.orientation_publisher.publish(orient_msg)
            
            # Create and publish velocity
            vel_msg = TwistStamped()
            vel_msg.header.stamp = self.get_clock().now().to_msg()
            vel_msg.header.frame_id = 'imu_link'
            vel_msg.twist.linear.x = self.linear_velocity[0]
            vel_msg.twist.linear.y = self.linear_velocity[1]
            vel_msg.twist.linear.z = self.linear_velocity[2]
            vel_msg.twist.angular.x = self.angular_velocity[0]
            vel_msg.twist.angular.y = self.angular_velocity[1]
            vel_msg.twist.angular.z = self.angular_velocity[2]
            
            self.velocity_publisher.publish(vel_msg)
            
            # Publish TF transform
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = 'odom'
            transform.child_frame_id = 'base_link'
            
            transform.transform.translation.x = self.position[0]
            transform.transform.translation.y = self.position[1]
            transform.transform.translation.z = self.position[2]
            
            transform.transform.rotation.w = self.orientation_quaternion[0]
            transform.transform.rotation.x = self.orientation_quaternion[1]
            transform.transform.rotation.y = self.orientation_quaternion[2]
            transform.transform.rotation.z = self.orientation_quaternion[3]
            
            self.tf_broadcaster.sendTransform(transform)
            
            # Update performance counter
            self.process_count += 1
    
    def report_performance(self):
        """Report processing performance statistics"""
        current_time = time.time()
        elapsed = current_time - self.last_performance_time
        
        if elapsed > 0 and self.process_count > 0:
            process_rate = self.process_count / elapsed
            self.get_logger().info(f'IMU processing rate: {process_rate:.2f} Hz')
            
            # Also report current state
            roll, pitch, yaw = self.orientation_euler
            self.get_logger().info(f'Orientation: roll={math.degrees(roll):.1f}°, pitch={math.degrees(pitch):.1f}°, yaw={math.degrees(yaw):.1f}°')
            
            vx, vy, vz = self.linear_velocity
            vel_mag = math.sqrt(vx*vx + vy*vy + vz*vz)
            self.get_logger().info(f'Velocity: {vel_mag:.2f} m/s')
            
            self.process_count = 0
            self.last_performance_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = IMUProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main() 