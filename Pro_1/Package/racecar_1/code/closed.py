#!/usr/bin/env python3
import math
import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray
from math import atan2, asin
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import matplotlib.pyplot as plt



class CloseLoopControlNode(Node):

    def __init__(self):
        super().__init__('close_loop_controller')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=10)
        self.final_wheel_velofity = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.final_joint_velofity = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.imu_subscriber_ = self.create_subscription(Imu, "/imu_plugin/out", self.imu_callback, qos_profile=qos_profile)
        self.current_position = [0.0, 0.0]
        self.current_yaw = 0.0
        self.t = 1
        self.start_time = 0
        self.init_dist=0
        self.target_position = [-10.0, -10.0]
        self.alpha_linear = 0.5  #alpha for the linear
        self.alpha_angular = 1.0
        self.time_values = []
        self.error_values = []
        self.control_values = []
        
    def imu_callback(self, imu: Imu):
        wheel_velocity = Float64MultiArray()
        joint_positions = Float64MultiArray()
        acceleration_x = imu.linear_acceleration.x
        acceleration_y = imu.linear_acceleration.y
        quaternion = imu.orientation
        roll, pitch, yaw = self.quaternion_to_euler(quaternion)

        #finding the position with imu acceleration
        dt = 0.1  
        self.current_position[0] += acceleration_x * (dt ** 2) / 2
        self.current_position[1] += acceleration_y * (dt ** 2) / 2
        self.current_yaw = yaw
        error_x = self.target_position[0] - self.current_position[0]
        error_y = self.target_position[1] - self.current_position[1]
        required_yaw = math.atan2(error_y, error_x)


        yaw_error = (self.current_yaw - required_yaw)  # calculating the steer velocity
        steer_angle = self.alpha_angular * (yaw_error)
        if 0.175 < abs(yaw_error) < 1.0472:
            linear_vel = 0.6
        else:
            linear_vel = 1.5
        if(self.t==1):
            self.start_time = time.time()
            self.t =2
        linear_vel = self.alpha_linear * math.sqrt(error_x**2 + error_y**2)
        self.time_values.append(time.time())
        self.error_values.append(math.sqrt(error_x*2 + error_y*2))
        self.control_values.append(steer_angle)
        wheel_velocity.data = [-linear_vel,0.0,0.0]
        curr_time = time.time()
        print(curr_time)
        elap_time = self.start_time - curr_time
        self.start_time = curr_time
        distace_moved= elap_time*linear_vel
        self.init_dist = self.init_dist+distace_moved
        print(self.init_dist)
        joint_positions.data = [steer_angle,steer_angle]
        self.final_wheel_velofity.publish(wheel_velocity)
        self.final_joint_velofity.publish(joint_positions)
        
        if (self.init_dist < 160):  #time calculated by hand after stopping the robot 
            wheel_velocity.data = [0.0,0.0,0.0]
            self.final_wheel_velofity.publish(wheel_velocity)
            plt.figure()
            plt.plot([1,2,3,4,5], [1,2,3,4,5])
            plt.xlabel('Time (s)')
            plt.ylabel('Error')
            plt.title('Error vs Time')
            plt.grid()
            plt.show()

            # Plot the Control vs Time graph
            plt.figure()
            plt.plot(self.time_values, self.control_values)
            plt.xlabel('Time (s)')
            plt.ylabel('Control Input')
            plt.title('Control vs Time')
            plt.grid()
            plt.show()
            rclpy.shutdown()


    
    def quaternion_to_euler(self, quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w

        #formula for roll pitch and yaw
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