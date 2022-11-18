#!/usr/bin/env python3
import airsim

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

client = airsim.VehicleClient()
client.confirmConnection()

vehicle_odometry = Odometry()

class AirSimBridge(Node):
    def __init__(self):
        super().__init__('airsim_bridge')
        self.time_duration_subscription = self.create_subscription(
            Odometry,
            '/state_estimation',
            self.odometry_callback,
            10)

    def odometry_callback(self, data):
        global vehicle_odometry
        vehicle_odometry = data
        vehicle_position = airsim.Vector3r(vehicle_odometry.pose.pose.position.x, -vehicle_odometry.pose.pose.position.y, -vehicle_odometry.pose.pose.position.z)
        vehicle_orientation = airsim.Quaternionr(vehicle_odometry.pose.pose.orientation.x, -vehicle_odometry.pose.pose.orientation.y, -vehicle_odometry.pose.pose.orientation.z, vehicle_odometry.pose.pose.orientation.w)
        client.simSetVehiclePose(airsim.Pose(vehicle_position, vehicle_orientation), True)

if __name__ == '__main__':
    rclpy.init(args=None)

    airsim_bridge = AirSimBridge()

    rclpy.spin(airsim_bridge)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    airsim_bridge.destroy_node()
    rclpy.shutdown()

    
