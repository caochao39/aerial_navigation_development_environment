#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
// #include <ros/ros.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

// #include <message_filters/subscriber.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>

// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/TwistStamped.h>
// #include <gazebo_msgs/ModelState.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <gazebo_msgs/msg/model_state.hpp>
#include <gazebo_msgs/msg/entity_state.hpp>
#include <gazebo_msgs/srv/set_entity_state.hpp>

// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "rmw/types.h"
#include "rmw/qos_profiles.h"

using namespace std;

const double PI = 3.1415926;

double realtimeFactor = 1.0;
double windCoeff = 0.05;
double maxRollPitchRate = 20.0;
double rollPitchSmoothRate = 0.1;
double sensorPitch = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 1.0;

float vehicleVelX = 0;
float vehicleVelY = 0;
float vehicleVelZ = 0;
float vehicleVelXG = 0;
float vehicleVelYG = 0;
float vehicleVelZG = 0;

float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleRollCmd = 0;
float vehiclePitchCmd = 0;
float vehicleYawRate = 0;

// void controlHandler(const geometry_msgs::TwistStamped::ConstPtr& controlIn)
void controlHandler(const geometry_msgs::msg::TwistStamped::ConstSharedPtr controlIn)
{
  vehicleRollCmd = controlIn->twist.linear.x;
  vehiclePitchCmd = controlIn->twist.linear.y;
  vehicleYawRate = controlIn->twist.angular.z;
  vehicleVelZG = controlIn->twist.linear.z;
}

int main(int argc, char** argv)
{
  // ros::init(argc, argv, "vehicleSimulator");
  // ros::NodeHandle nh;
  // ros::NodeHandle nhPrivate = ros::NodeHandle("~");
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("vehicleSimulator");

  // nhPrivate.getParam("realtimeFactor", realtimeFactor);
  // nhPrivate.getParam("windCoeff", windCoeff);
  // nhPrivate.getParam("maxRollPitchRate", maxRollPitchRate);
  // nhPrivate.getParam("rollPitchSmoothRate", rollPitchSmoothRate);
  // nhPrivate.getParam("sensorPitch", sensorPitch);
  // nhPrivate.getParam("vehicleX", vehicleX);
  // nhPrivate.getParam("vehicleY", vehicleY);
  // nhPrivate.getParam("vehicleZ", vehicleZ);
  // nhPrivate.getParam("vehicleYaw", vehicleYaw);

  nh->declare_parameter<double>("realtimeFactor", realtimeFactor);
  nh->declare_parameter<double>("windCoeff", windCoeff);
  nh->declare_parameter<double>("maxRollPitchRate", maxRollPitchRate);
  nh->declare_parameter<double>("rollPitchSmoothRate", rollPitchSmoothRate);
  nh->declare_parameter<double>("sensorPitch", sensorPitch);
  nh->declare_parameter<double>("vehicleX", vehicleX);
  nh->declare_parameter<double>("vehicleY", vehicleY);
  nh->declare_parameter<double>("vehicleZ", vehicleZ);
  nh->declare_parameter<double>("vehicleYaw", vehicleYaw);

  nh->get_parameter("realtimeFactor", realtimeFactor);
  nh->get_parameter("windCoeff", windCoeff);
  nh->get_parameter("maxRollPitchRate", maxRollPitchRate);
  nh->get_parameter("rollPitchSmoothRate", rollPitchSmoothRate);
  nh->get_parameter("sensorPitch", sensorPitch);
  nh->get_parameter("vehicleX", vehicleX);
  nh->get_parameter("vehicleY", vehicleY);
  nh->get_parameter("vehicleZ", vehicleZ);
  nh->get_parameter("vehicleYaw", vehicleYaw);

  // ros::Subscriber subControl = nh.subscribe<geometry_msgs::TwistStamped> ("/attitude_control", 5, controlHandler);

  // ros::Publisher pubVehicleOdom = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);

  auto subControl = nh->create_subscription<geometry_msgs::msg::TwistStamped>("/attitude_control", 5, controlHandler);

  auto pubVehicleOdom = nh->create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 5);

  // nav_msgs::Odometry odomData;
  nav_msgs::msg::Odometry odomData;
  odomData.header.frame_id = "map";
  odomData.child_frame_id = "vehicle";

  // tf::TransformBroadcaster tfBroadcaster;
  // tf::StampedTransform odomTrans;
  // odomTrans.frame_id_ = "map";
  // odomTrans.child_frame_id_ = "vehicle";

  auto tfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*nh);
  tf2::Stamped<tf2::Transform> odomTrans;
  geometry_msgs::msg::TransformStamped transformTfGeom ; 
  odomTrans.frame_id_ = "map";
  // odomTrans.child_frame_id_ = "vehicle";

  // ros::Publisher pubModelState = nh.advertise<gazebo_msgs::ModelState> ("/gazebo/set_model_state", 5);
  // gazebo_msgs::ModelState cameraState;
  // cameraState.model_name = "rgbd_camera";
  // gazebo_msgs::ModelState robotState;
  // robotState.model_name = "robot";
  gazebo_msgs::msg::EntityState cameraState;
  cameraState.name = "rgbd_camera";
  gazebo_msgs::msg::EntityState robotState;
  robotState.name = "robot";

  rclcpp::Client<gazebo_msgs::srv::SetEntityState>::SharedPtr client = nh->create_client<gazebo_msgs::srv::SetEntityState>("/set_entity_state");
  auto request  = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();

  printf("\nSimulation started.\n\n");

  tf2::Quaternion quat_tf;

  // ros::Rate rate(200 * realtimeFactor);
  // bool status = ros::ok();

  rclcpp::Rate rate(200 * realtimeFactor);
  bool status = rclcpp::ok();
  while (status) {
    // ros::spinOnce();
    rclcpp::spin_some(nh);

    float vehicleRecRoll = vehicleRoll;
    float vehicleRecPitch = vehiclePitch;

    if (vehicleRollCmd - vehicleRoll > maxRollPitchRate / 200.0) vehicleRoll += maxRollPitchRate / 200.0;
    else if (vehicleRollCmd - vehicleRoll < -maxRollPitchRate / 200.0) vehicleRoll -= maxRollPitchRate / 200.0;
    else vehicleRoll = vehicleRollCmd;
    vehicleRoll = rollPitchSmoothRate * vehicleRoll + (1.0 - rollPitchSmoothRate) * vehicleRecRoll;

    if (vehiclePitchCmd - vehiclePitch > maxRollPitchRate / 200.0) vehiclePitch += maxRollPitchRate / 200.0;
    else if (vehiclePitchCmd - vehiclePitch < -maxRollPitchRate / 200.0) vehiclePitch -= maxRollPitchRate / 200.0;
    else vehiclePitch = vehiclePitchCmd;
    vehiclePitch = rollPitchSmoothRate * vehiclePitch + (1.0 - rollPitchSmoothRate) * vehicleRecPitch;

    float vehicleAccX = 9.8 * tan(vehiclePitch);
    float vehicleAccY = -9.8 * tan(vehicleRoll) / cos(vehiclePitch);

    if (vehicleVelXG < 0) vehicleVelXG += windCoeff * vehicleVelXG * vehicleVelXG  / 200.0;
    else vehicleVelXG -= windCoeff * vehicleVelXG * vehicleVelXG  / 200.0;
    if (vehicleVelYG < 0) vehicleVelYG += windCoeff * vehicleVelYG * vehicleVelYG  / 200.0;
    else vehicleVelYG -= windCoeff * vehicleVelYG * vehicleVelYG  / 200.0;

    vehicleVelXG += (vehicleAccX * cos(vehicleYaw) - vehicleAccY * sin(vehicleYaw)) / 200.0;
    vehicleVelYG += (vehicleAccX * sin(vehicleYaw) + vehicleAccY * cos(vehicleYaw)) / 200.0;

    float velX1 = vehicleVelXG * cos(vehicleYaw) + vehicleVelYG * sin(vehicleYaw);
    float velY1 = -vehicleVelXG * sin(vehicleYaw) + vehicleVelYG * cos(vehicleYaw);
    float velZ1 = vehicleVelZG;

    float velX2 = velX1 * cos(vehiclePitch) - velZ1 * sin(vehiclePitch);
    float velY2 = velY1;
    float velZ2 = velX1 * sin(vehiclePitch) + velZ1 * cos(vehiclePitch);

    vehicleVelX = velX2;
    vehicleVelY = velY2 * cos(vehicleRoll) + velZ2 * sin(vehicleRoll);
    vehicleVelZ = -velY2 * sin(vehicleRoll) + velZ2 * cos(vehicleRoll);

    vehicleX += vehicleVelXG / 200.0;
    vehicleY += vehicleVelYG / 200.0;
    vehicleZ += vehicleVelZG / 200.0;
    vehicleYaw += vehicleYawRate / 200.0;

    // ros::Time timeNow = ros::Time::now();
    rclcpp::Time timeNow = nh->now();

    // geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(vehicleRoll, vehiclePitch, vehicleYaw);
    quat_tf.setRPY(vehicleRoll, vehiclePitch, vehicleYaw);
    geometry_msgs::msg::Quaternion geoQuat;
    tf2::convert(quat_tf, geoQuat);

    // publish 200Hz odometry messages
    odomData.header.stamp = timeNow;
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = vehicleZ;
    odomData.twist.twist.angular.x = 200.0 * (vehicleRoll - vehicleRecRoll);
    odomData.twist.twist.angular.y = 200.0 * (vehiclePitch - vehicleRecPitch);
    odomData.twist.twist.angular.z = vehicleYawRate;
    odomData.twist.twist.linear.x = vehicleVelX;
    odomData.twist.twist.linear.y = vehicleVelY;
    odomData.twist.twist.linear.z = vehicleVelZ;
    // pubVehicleOdom.publish(odomData);
    pubVehicleOdom->publish(odomData);

    // odomTrans.stamp_ = timeNow;
    // odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    // odomTrans.setOrigin(tf::Vector3(vehicleX, vehicleY, vehicleZ));
    // tfBroadcaster.sendTransform(odomTrans);

    // odomTrans.stamp_ = timeNow;
    odomTrans.setRotation(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
    odomTrans.setOrigin(tf2::Vector3(vehicleX, vehicleY, vehicleZ));
    transformTfGeom = tf2::toMsg(odomTrans);
    transformTfGeom.child_frame_id = "vehicle";
    transformTfGeom.header.stamp = timeNow; 
    tfBroadcaster->sendTransform(transformTfGeom);

    // geoQuat = tf::createQuaternionMsgFromRollPitchYaw(vehicleRoll, sensorPitch + vehiclePitch, vehicleYaw);
    quat_tf.setRPY(vehicleRoll, sensorPitch + vehiclePitch, vehicleYaw);
    tf2::convert(quat_tf, geoQuat);

    // publish 200Hz Gazebo model state messages
    cameraState.pose.orientation = geoQuat;
    cameraState.pose.position.x = vehicleX;
    cameraState.pose.position.y = vehicleY;
    cameraState.pose.position.z = vehicleZ;
    // pubModelState.publish(cameraState);
    request->state = cameraState;
    auto response = client->async_send_request(request);
    
    robotState.pose.orientation = geoQuat;
    robotState.pose.position.x = vehicleX;
    robotState.pose.position.y = vehicleY;
    robotState.pose.position.z = vehicleZ;
    // pubModelState.publish(robotState);
    request->state = robotState;
    response = client->async_send_request(request);

    // status = ros::ok();
    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
