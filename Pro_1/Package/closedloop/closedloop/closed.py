#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
import math
from math import atan2, asin
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
class CloseLoopControlNode(Node):

    def __init__(self):
        super().__init__('close_loop_controller')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.imu_subscriber_ = self.create_subscription(Imu, "/imu_plugin/out", self.imu_callback, qos_profile=qos_profile)
        self.current_position = [0.0, 0.0]
        self.current_yaw = 0.0
        self.t = 1
        self.start_time = 0
        self.init_dist=0
        self.target_position = [10.0, 10.0]
        self.kp_linear = 0.5  # Proportional gain
        self.kp_ang = 1.0
        
    def imu_callback(self, imu: Imu):
        wheel_velocities = Float64MultiArray()
        joint_positions = Float64MultiArray()
        linear_acceleration_x = imu.linear_acceleration.x
        linear_acceleration_y = imu.linear_acceleration.y
        # angular_velocity = imu.angular_velocity.z
        quaternion = imu.orientation
        roll, pitch, yaw = self.quaternion_to_euler(quaternion)

        # Numerically integrate the accelerations to estimate position (dead reckoning)
        dt = 0.1  # Time step
        self.current_position[0] += linear_acceleration_x * (dt ** 2) / 2
        self.current_position[1] += linear_acceleration_y * (dt ** 2) / 2
        # self.current_yaw += angular_velocity * dt
        # self.current_yaw = angular_velocity
        self.current_yaw = yaw
        error_x = self.target_position[0] - self.current_position[0]
        error_y = self.target_position[1] - self.current_position[1]
        # print(error_x)
        # Calculate the angle between the current position and the target position
        desired_yaw = math.atan2(error_y, error_x)

        # Calculate the linear velocity for the wheels
        # linear_vel = self.kp * math.sqrt(error_x**2 + error_y**2)
        yaw_error = (self.current_yaw - desired_yaw)
        # Calculate the angular velocity for the steering links
        steer_angle = self.kp_ang * (yaw_error)
        if 0.0873 < abs(yaw_error) < 0.873:
            linear_vel = 0.4
        else:
            linear_vel = 1.1
        if(self.t==1):
            self.start_time = time.time()
            self.t =2
        linear_vel = self.kp_linear * math.sqrt(error_x**2 + error_y**2)
        wheel_velocities.data = [-linear_vel,0.0,0.0]
        current_time = time.time()
        print(current_time)
        elap_time = self.start_time - current_time
        self.start_time = current_time
        dist_moved= elap_time*linear_vel
        self.init_dist = self.init_dist+dist_moved
        print(self.init_dist)
        joint_positions.data = [steer_angle,steer_angle]
        self.wheel_velocities_pub.publish(wheel_velocities)
        self.joint_position_pub.publish(joint_positions)
        if (self.init_dist < -171):
            wheel_velocities.data = [0.0,0.0,0.0]
            self.wheel_velocities_pub.publish(wheel_velocities)
            rclpy.shutdown()


    
    def quaternion_to_euler(self, quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        roll = atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        pitch = asin(2 * (w * y - z * x))
        yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    
    node = CloseLoopControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()