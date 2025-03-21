#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange, IntegerRange
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class LidarParamTuner(Node):
    def __init__(self):
        super().__init__('lidar_param_tuner')
        
        # SOR (Statistical Outlier Removal) Parameters
        self.declare_parameter(
            'sor_mean_k',
            2.0,
            ParameterDescriptor(
                description='Mean K neighbors for SOR filtering',
                floating_point_range=[FloatingPointRange(
                    from_value=1.0,
                    to_value=10.0,
                    step=0.1
                )]
            )
        )
        
        self.declare_parameter(
            'sor_stddev',
            0.1,
            ParameterDescriptor(
                description='Standard deviation multiplier for SOR',
                floating_point_range=[FloatingPointRange(
                    from_value=0.01,
                    to_value=1.0,
                    step=0.01
                )]
            )
        )
        
        # RANSAC Parameters
        self.declare_parameter(
            'ransac_iterations',
            100,
            ParameterDescriptor(
                description='RANSAC iterations for ground removal',
                integer_range=[IntegerRange(
                    from_value=50,
                    to_value=1000,
                    step=10
                )]
            )
        )
        
        self.declare_parameter(
            'ransac_threshold',
            5,
            ParameterDescriptor(
                description='RANSAC threshold distance',
                integer_range=[IntegerRange(
                    from_value=1,
                    to_value=20,
                    step=1
                )]
            )
        )
        
        # Euclidean Clustering Parameters
        self.declare_parameter(
            'cluster_tolerance',
            0.6,
            ParameterDescriptor(
                description='Clustering distance tolerance',
                floating_point_range=[FloatingPointRange(
                    from_value=0.1,
                    to_value=2.0,
                    step=0.1
                )]
            )
        )
        
        self.declare_parameter(
            'min_cluster_size',
            30,
            ParameterDescriptor(
                description='Minimum points in cluster',
                integer_range=[IntegerRange(
                    from_value=5,
                    to_value=100,
                    step=1
                )]
            )
        )
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # Create timer for parameter monitoring
        self.create_timer(1.0, self.parameter_status)  # 1Hz status update

    def parameter_callback(self, params):
        """Callback for parameter changes"""
        for param in params:
            # Validate parameters
            if param.name == 'sor_mean_k':
                if param.value < 1.0 or param.value > 10.0:
                    return SetParametersResult(successful=False, reason='SOR mean K out of range')
                    
            elif param.name == 'sor_stddev':
                if param.value < 0.01 or param.value > 1.0:
                    return SetParametersResult(successful=False, reason='SOR stddev out of range')
                    
            elif param.name == 'ransac_iterations':
                if param.value < 50 or param.value > 1000:
                    return SetParametersResult(successful=False, reason='RANSAC iterations out of range')
                    
            elif param.name == 'ransac_threshold':
                if param.value < 1 or param.value > 20:
                    return SetParametersResult(successful=False, reason='RANSAC threshold out of range')
                    
            elif param.name == 'cluster_tolerance':
                if param.value < 0.1 or param.value > 2.0:
                    return SetParametersResult(successful=False, reason='Cluster tolerance out of range')
                    
            elif param.name == 'min_cluster_size':
                if param.value < 5 or param.value > 100:
                    return SetParametersResult(successful=False, reason='Min cluster size out of range')
            
            # Log parameter change
            self.get_logger().info(f'Parameter {param.name} changed to: {param.value}')
            
        return SetParametersResult(successful=True)

    def parameter_status(self):
        """Print current parameter values"""
        params = {
            'SOR Filter': {
                'Mean K': self.get_parameter('sor_mean_k').value,
                'StdDev': self.get_parameter('sor_stddev').value
            },
            'RANSAC': {
                'Iterations': self.get_parameter('ransac_iterations').value,
                'Threshold': self.get_parameter('ransac_threshold').value
            },
            'Clustering': {
                'Tolerance': self.get_parameter('cluster_tolerance').value,
                'Min Size': self.get_parameter('min_cluster_size').value
            }
        }
        
        self.get_logger().info('\nCurrent Parameters:')
        for category, values in params.items():
            self.get_logger().info(f'\n{category}:')
            for param, value in values.items():
                self.get_logger().info(f'  {param}: {value}')

def main(args=None):
    rclpy.init(args=args)
    node = LidarParamTuner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 