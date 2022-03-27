#!/usr/bin/env python3
import airsim

import rospy
from nav_msgs.msg import Odometry

client = airsim.VehicleClient()
client.confirmConnection()

vehicle_odometry = Odometry()

def odometry_callback(data):
    global vehicle_odometry
    vehicle_odometry = data
    vehicle_position = airsim.Vector3r(vehicle_odometry.pose.pose.position.x, -vehicle_odometry.pose.pose.position.y, -vehicle_odometry.pose.pose.position.z)
    vehicle_orientation = airsim.Quaternionr(vehicle_odometry.pose.pose.orientation.x, -vehicle_odometry.pose.pose.orientation.y, -vehicle_odometry.pose.pose.orientation.z, vehicle_odometry.pose.pose.orientation.w)
    client.simSetVehiclePose(airsim.Pose(vehicle_position, vehicle_orientation), True)

if __name__ == '__main__':
    rospy.init_node('airsim_bridge', anonymous=True)
    rospy.Subscriber("/state_estimation", Odometry, odometry_callback)

    rospy.spin()

    
