#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Joy.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;

const double PI = 3.1415926;

string stateEstimationTopic = "/state_estimation";
int pubSkipNum = 1;
int pubSkipCount = 0;
bool trackingCamBackward = false;
double trackingCamXOffset = 0;
double trackingCamYOffset = 0;
double trackingCamZOffset = 0;
double trackingCamScale = 1.0;
double lookAheadScale = 0.2;
double minSpeed = 0.5;
double maxSpeed = 2.0;
double desiredSpeed = minSpeed;
double velXYGain = 0.3;
double posXYGain = 0.05;
double stopVelXYGain = 0.2;
double stopPosXYGain = 0.2;
double smoothIncrSpeed = 0.75;
double maxRollPitch = 30.0;
double yawGain = 2.0;
double maxRateByYaw = 60.0;
double posZGain = 1.5;
double maxVelByPosZ = 0.5;
double manualSpeedXY = 2.0;
double manualSpeedZ = 1.0;
double manualYawRate = 60.0;
double stopDis = 0.5;
double slowDis = 2.0;
double joyDeadband = 0.1;
bool fixGoalYaw = true;
bool shiftGoalAtStart = false;
double goalYaw = 0;
double goalX = 0;
double goalY = 0;
double goalZ = 1.0;

bool manualMode = true;
int initCount = 100;

float joyFwd = 0;
float joyLeft = 0;
float joyUp = 0;
float joyYaw = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleYaw = 0;

float vehicleVelX = 0;
float vehicleVelY = 0;
float vehicleVelZ = 0;

float vehicleAngRateX = 0;
float vehicleAngRateY = 0;
float vehicleAngRateZ = 0;

geometry_msgs::TwistStamped control_cmd;
ros::Publisher *pubControlPointer;

void stateEstimationHandler(const nav_msgs::Odometry::ConstPtr& odom)
{
  if (initCount >= 0 && shiftGoalAtStart) {
    if (initCount == 0) {
      goalX += trackingCamScale * odom->pose.pose.position.x;
      goalY += trackingCamScale * odom->pose.pose.position.y;
      goalZ += trackingCamScale * odom->pose.pose.position.z;
    }
    initCount--;
    return;
  }

  pubSkipCount--;
  if (pubSkipCount >= 0) {
    return;
  } else {
    pubSkipCount = pubSkipNum;
  }

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odom->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleX = trackingCamScale * odom->pose.pose.position.x;
  vehicleY = trackingCamScale * odom->pose.pose.position.y;
  vehicleZ = trackingCamScale * odom->pose.pose.position.z;
  vehicleVelX = trackingCamScale * odom->twist.twist.linear.x;
  vehicleVelY = trackingCamScale * odom->twist.twist.linear.y;
  vehicleVelZ = trackingCamScale * odom->twist.twist.linear.z;
  vehicleAngRateX = odom->twist.twist.angular.x;
  vehicleAngRateY = odom->twist.twist.angular.y;
  vehicleAngRateZ = odom->twist.twist.angular.z;
  vehicleYaw = yaw;

  if (trackingCamBackward) {
    roll = -roll;
    pitch = -pitch;
    vehicleX = -vehicleX;
    vehicleY = -vehicleY;
    vehicleVelX = -vehicleVelX;
    vehicleVelY = -vehicleVelY;
    vehicleAngRateX = -vehicleAngRateX;
    vehicleAngRateY = -vehicleAngRateY;
  }

  float sinRoll = sin(roll);
  float cosRoll = cos(roll);
  float sinPitch = sin(pitch);
  float cosPitch = cos(pitch);
  float sinYaw = sin(yaw);
  float cosYaw = cos(yaw);

  float pointX1 = trackingCamXOffset;
  float pointY1 = trackingCamYOffset * cosRoll - trackingCamZOffset * sinRoll;
  float pointZ1 = trackingCamYOffset * sinRoll + trackingCamZOffset * cosRoll;

  float pointX2 = pointX1 * cosPitch + pointZ1 * sinPitch;
  float pointY2 = pointY1;
  float pointZ2 = -pointX1 * sinPitch + pointZ1 * cosPitch;

  vehicleX -= pointX2 * cosYaw - pointY2 * sinYaw - trackingCamXOffset;
  vehicleY -= pointX2 * sinYaw + pointY2 * cosYaw - trackingCamYOffset;
  vehicleZ -= pointZ2 - trackingCamZOffset;

  vehicleVelX -= -trackingCamYOffset * vehicleAngRateZ + trackingCamZOffset * vehicleAngRateY;
  vehicleVelY -= -trackingCamZOffset * vehicleAngRateX + trackingCamXOffset * vehicleAngRateZ;
  vehicleVelZ -= -trackingCamXOffset * vehicleAngRateY + trackingCamYOffset * vehicleAngRateX;

  float velX1 = vehicleVelX;
  float velY1 = vehicleVelY * cosRoll - vehicleVelZ * sinRoll;
  float velZ1 = vehicleVelY * sinRoll + vehicleVelZ * cosRoll;

  vehicleVelX = velX1 * cosPitch + velZ1 * sinPitch;
  vehicleVelY = velY1;
  vehicleVelZ = -velX1 * sinPitch + velZ1 * cosPitch;

  float vehicleSpeed = sqrt(vehicleVelX * vehicleVelX + vehicleVelY * vehicleVelY);

  float disToGoalX = goalX - vehicleX;
  float disToGoalY = goalY - vehicleY;
  float disToGoal = sqrt(disToGoalX * disToGoalX + disToGoalY * disToGoalY);
  float dirToGoal = atan2(goalY - vehicleY, goalX - vehicleX) - vehicleYaw;
  if (dirToGoal > PI) dirToGoal -= 2 * PI;
  else if (dirToGoal < -PI) dirToGoal += 2 * PI;

  if (manualMode) {
    float desiredRoll = -stopVelXYGain * (manualSpeedXY * joyLeft - vehicleVelY) - posXYGain * lookAheadScale * manualSpeedXY * joyLeft;
    if (desiredRoll > maxRollPitch * PI / 180.0) desiredRoll = maxRollPitch * PI / 180.0;
    else if (desiredRoll < -maxRollPitch * PI / 180.0) desiredRoll = -maxRollPitch * PI / 180.0;

    float desiredPitch = stopVelXYGain * (manualSpeedXY * joyFwd - vehicleVelX) + posXYGain * lookAheadScale * manualSpeedXY * joyFwd;
    if (desiredPitch > maxRollPitch * PI / 180.0) desiredPitch = maxRollPitch * PI / 180.0;
    else if (desiredPitch < -maxRollPitch * PI / 180.0) desiredPitch = -maxRollPitch * PI / 180.0;

    control_cmd.twist.linear.x = desiredRoll;
    control_cmd.twist.linear.y = desiredPitch;
    control_cmd.twist.linear.z = manualSpeedZ * joyUp;
    control_cmd.twist.angular.z = manualYawRate * joyYaw * PI / 180.0;
  } else {
    float desiredSpeed2 = vehicleSpeed + smoothIncrSpeed;
    float velXYGain2 = velXYGain;
    float posXYGain2 = posXYGain;
    if (desiredSpeed2 < minSpeed) desiredSpeed2 = minSpeed;
    else if (desiredSpeed2 > desiredSpeed) desiredSpeed2 = desiredSpeed;
    if (disToGoal < stopDis) {
      desiredSpeed2 = 0;
      velXYGain2 = stopVelXYGain;
      posXYGain2 = stopPosXYGain;
    } else if (disToGoal < slowDis) {
      float slowSpeed = (maxSpeed * (disToGoal - stopDis) + minSpeed * (slowDis - disToGoal)) / (slowDis - stopDis);
      if (desiredSpeed2 > slowSpeed) desiredSpeed2 = slowSpeed;
    }

    float disX = goalX - vehicleX;
    float disY = goalY - vehicleY;
    float disZ = goalZ - vehicleZ;

    float disX2 = disX * cos(vehicleYaw) + disY * sin(vehicleYaw);
    float disY2 = -disX * sin(vehicleYaw) + disY * cos(vehicleYaw);
    float dis2 = sqrt(disX2 * disX2 + disY2 * disY2);

    float lookAheadDis = lookAheadScale * desiredSpeed;
    if (lookAheadDis > dis2 || disToGoal < stopDis) lookAheadDis = dis2;

    float dirDiff = goalYaw - vehicleYaw;
    if (dirDiff > PI) dirDiff -= 2 * PI;
    else if (dirDiff < -PI) dirDiff += 2 * PI;

    float desiredRoll = 0;
    float desiredPitch = 0;
    if (dis2 > 0.001) {
      desiredRoll = -velXYGain2 * (desiredSpeed2 * disY2 / dis2 - vehicleVelY) - posXYGain2 * disY2 / dis2 * lookAheadDis;
      if (desiredRoll > maxRollPitch * PI / 180.0) desiredRoll = maxRollPitch * PI / 180.0;
      else if (desiredRoll < -maxRollPitch * PI / 180.0) desiredRoll = -maxRollPitch * PI / 180.0;

      desiredPitch = velXYGain2 * (desiredSpeed2 * disX2 / dis2 - vehicleVelX) + posXYGain2 * disX2 / dis2 * lookAheadDis;
      if (desiredPitch > maxRollPitch * PI / 180.0) desiredPitch = maxRollPitch * PI / 180.0;
      else if (desiredPitch < -maxRollPitch * PI / 180.0) desiredPitch = -maxRollPitch * PI / 180.0;
    }

    float desiredYawRate = yawGain * dirDiff;
    if (desiredYawRate > maxRateByYaw * PI / 180.0) desiredYawRate = maxRateByYaw * PI / 180.0;
    else if (desiredYawRate < -maxRateByYaw * PI / 180.0) desiredYawRate = -maxRateByYaw * PI / 180.0;

    float desiredVelZ = posZGain * disZ;
    if (desiredVelZ > maxVelByPosZ) desiredVelZ = maxVelByPosZ;
    else if (desiredVelZ < -maxVelByPosZ) desiredVelZ = -maxVelByPosZ;

    control_cmd.twist.linear.x = desiredRoll;
    control_cmd.twist.linear.y = desiredPitch;
    control_cmd.twist.linear.z = desiredVelZ;
    control_cmd.twist.angular.z = desiredYawRate;
  }

  control_cmd.header.stamp = odom->header.stamp;
  control_cmd.header.frame_id = "vehicle";
  pubControlPointer->publish(control_cmd);
}

void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
  joyFwd = joy->axes[4];
  if (fabs(joyFwd) < joyDeadband) joyFwd = 0;
  joyLeft = joy->axes[3];
  if (fabs(joyLeft) < joyDeadband) joyLeft = 0;
  joyUp = joy->axes[1];
  if (fabs(joyUp) < joyDeadband) joyUp = 0;
  joyYaw = joy->axes[0];
  if (fabs(joyYaw) < joyDeadband) joyYaw = 0;

  if (joy->axes[2] < -0.1) {
    manualMode = false;
  } else {
    manualMode = true;
  }

  if (desiredSpeed < maxSpeed * joyFwd) desiredSpeed = maxSpeed * joyFwd;
  else if (desiredSpeed > maxSpeed * (joyFwd + joyDeadband)) desiredSpeed = maxSpeed * (joyFwd + joyDeadband);
  if (desiredSpeed < minSpeed || joyFwd <= joyDeadband) desiredSpeed = minSpeed;
  else if (desiredSpeed > maxSpeed) desiredSpeed = maxSpeed;
}

void goalHandler(const geometry_msgs::PointStamped::ConstPtr& goal)
{
  goalX = goal->point.x;
  goalY = goal->point.y;
  goalZ = goal->point.z;

  if (!fixGoalYaw) goalYaw = atan2(goalY - vehicleY, goalX - vehicleX);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "controlTuner");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("stateEstimationTopic", stateEstimationTopic);
  nhPrivate.getParam("pubSkipNum", pubSkipNum);
  nhPrivate.getParam("trackingCamBackward", trackingCamBackward);
  nhPrivate.getParam("trackingCamXOffset", trackingCamXOffset);
  nhPrivate.getParam("trackingCamYOffset", trackingCamYOffset);
  nhPrivate.getParam("trackingCamZOffset", trackingCamZOffset);
  nhPrivate.getParam("trackingCamScale", trackingCamScale);
  nhPrivate.getParam("lookAheadScale", lookAheadScale);
  nhPrivate.getParam("minSpeed", minSpeed);
  nhPrivate.getParam("maxSpeed", maxSpeed);
  nhPrivate.getParam("velXYGain", velXYGain);
  nhPrivate.getParam("posXYGain", posXYGain);
  nhPrivate.getParam("stopVelXYGain", stopVelXYGain);
  nhPrivate.getParam("stopPosXYGain", stopPosXYGain);
  nhPrivate.getParam("smoothIncrSpeed", smoothIncrSpeed);
  nhPrivate.getParam("maxRollPitch", maxRollPitch);
  nhPrivate.getParam("yawGain", yawGain);
  nhPrivate.getParam("maxRateByYaw", maxRateByYaw);
  nhPrivate.getParam("posZGain", posZGain);
  nhPrivate.getParam("maxVelByPosZ", maxVelByPosZ);
  nhPrivate.getParam("manualSpeedXY", manualSpeedXY);
  nhPrivate.getParam("manualSpeedZ", manualSpeedZ);
  nhPrivate.getParam("manualYawRate", manualYawRate);
  nhPrivate.getParam("stopDis", stopDis);
  nhPrivate.getParam("slowDis", slowDis);
  nhPrivate.getParam("fixGoalYaw", fixGoalYaw);
  nhPrivate.getParam("shiftGoalAtStart", shiftGoalAtStart);
  nhPrivate.getParam("goalYaw", goalYaw);
  nhPrivate.getParam("goalX", goalX);
  nhPrivate.getParam("goalY", goalY);
  nhPrivate.getParam("goalZ", goalZ);

  desiredSpeed = minSpeed;

  ros::Subscriber subStateEstimation = nh.subscribe<nav_msgs::Odometry> (stateEstimationTopic, 5, stateEstimationHandler);

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);

  ros::Subscriber subGoal = nh.subscribe<geometry_msgs::PointStamped> ("/way_point", 5, goalHandler);

  ros::Publisher pubControl = nh.advertise<geometry_msgs::TwistStamped> ("/attitude_control", 5);
  pubControlPointer = &pubControl;

  ros::spin();

  return 0;
}
