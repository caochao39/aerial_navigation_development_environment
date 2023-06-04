#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "rmw/types.h"
#include "rmw/qos_profiles.h"

#define PLOTPATHSET 1 // set to 0 to save processing and 1 to plot path set

using namespace std;

const double PI = 3.1415926;

// general parameters
string pathFolder;
string stateEstimationTopic = "/state_estimation";
string depthCloudTopic = "/rgbd_camera/points";
double depthCloudDelay = 0;
double depthCamPitchOffset = 0;
double depthCamXOffset = 0;
double depthCamYOffset = 0;
double depthCamZOffset = 0;
bool trackingCamBackward = false;
double trackingCamXOffset = 0;
double trackingCamYOffset = 0;
double trackingCamZOffset = 0;
double trackingCamScale = 1.0;
double scanVoxelSize = 0.1;
const int laserCloudStackNum = 1;
int laserCloudCount = 0;
int pointPerPathThre = 2;
double maxRange = 4.0;
double maxElev = 5.0;
bool keepSurrCloud = true;
double keepHoriDis = 1.0;
double keepVertDis = 0.5;
double lowerBoundZ = -1.2;
double upperBoundZ = 1.2;
double pitchDiffLimit = 35.0;
double pitchWeight = 0.03;
double sensorMaxPitch = 25.0;
double sensorMaxYaw = 40.0;
double yawDiffLimit = 60.0;
double yawWeight = 0.015;
double pathScale = 0.5;
double minPathScale = 0.25;
double pathScaleStep = 0.125;
bool pathScaleBySpeed = true;
double stopDis = 0.5;
bool shiftGoalAtStart = false;
double goalX = 0;
double goalY = 0;
double goalZ = 1.0;

// path parameters, set according to path files
const int pathNum = 4375;
const int groupNum = 25;
float gridVoxelSize = 0.2;
float searchRadiusHori = 1.2;
float searchRadiusVert = 0.8;
float gridVoxelOffsetX = 6.4;
float gridVoxelOffsetY = 9.0;
float gridVoxelOffsetZ = 3.3;
const int gridVoxelNumX = 33;
const int gridVoxelNumY = 91;
const int gridVoxelNumZ = 34;
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY * gridVoxelNumZ;

float joyFwd = 0;
float joyLeft = 0;
float joyUp = 0;
bool manualMode = true;
bool autonomyMode = false;
bool autoAdjustMode = false;
int systemInitDelay = 5;
int stateInitDelay = 100;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudStack[laserCloudStackNum];
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudKeep(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudKeepDwz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr plannerCloudStack(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>());
#endif

int pathList[pathNum] = {0};
float endPitchPathList[pathNum] = {0};
float endYawPathList[pathNum] = {0};
float endZPathList[pathNum] = {0};
int clearPathList[pathNum] = {0};
float clearPathPerGroupScore[groupNum] = {0};
std::vector<int> correspondences[gridVoxelNum];

double laserTime = 0;
bool newlaserCloud = false;

int odomPointerFront = 0;
int odomPointerLast = -1;
const int odomQueLength = 400;
double odomTime[odomQueLength] = {0};
float odomRoll[odomQueLength] = {0};
float odomPitch[odomQueLength] = {0};
float odomYaw[odomQueLength] = {0};
float odomX[odomQueLength] = {0};
float odomY[odomQueLength] = {0};
float odomZ[odomQueLength] = {0};

float trackX = 0;
float trackY = 0;
float trackZ = 0;
float trackRoll = 0;
float trackPitch = 0;
float trackYaw = 0;

pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;

rclcpp::Node::SharedPtr nh;

void stateEstimationHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  if (stateInitDelay >= 0 && shiftGoalAtStart) {
    if (stateInitDelay == 0) {
      goalX += trackingCamScale * odom->pose.pose.position.x;
      goalY += trackingCamScale * odom->pose.pose.position.y;
      goalZ += trackingCamScale * odom->pose.pose.position.z;
    }
    stateInitDelay--;
    return;
  }

  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  float vehicleX = trackingCamScale * odom->pose.pose.position.x;
  float vehicleY = trackingCamScale * odom->pose.pose.position.y;
  float vehicleZ = trackingCamScale * odom->pose.pose.position.z;

  if (trackingCamBackward) {
    roll = -roll;
    pitch = -pitch;
    vehicleX = -vehicleX;
    vehicleY = -vehicleY;
  }

  float pointX1 = trackingCamXOffset;
  float pointY1 = trackingCamYOffset * cos(roll) - trackingCamZOffset * sin(roll);
  float pointZ1 = trackingCamYOffset * sin(roll) + trackingCamZOffset * cos(roll);

  float pointX2 = pointX1 * cos(pitch) + pointZ1 * sin(pitch);
  float pointY2 = pointY1;
  float pointZ2 = -pointX1 * sin(pitch) + pointZ1 * cos(pitch);

  vehicleX -= pointX2 * cos(yaw) - pointY2 * sin(yaw) - trackingCamXOffset;
  vehicleY -= pointX2 * sin(yaw) + pointY2 * cos(yaw) - trackingCamYOffset;
  vehicleZ -= pointZ2 - trackingCamZOffset;

  odomPointerLast = (odomPointerLast + 1) % odomQueLength;

  odomTime[odomPointerLast] = rclcpp::Time(odom->header.stamp).seconds();
  odomRoll[odomPointerLast] = roll;
  odomPitch[odomPointerLast] = pitch;
  odomYaw[odomPointerLast] = yaw;
  odomX[odomPointerLast] = vehicleX;
  odomY[odomPointerLast] = vehicleY;
  odomZ[odomPointerLast] = vehicleZ;
}

void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
{
  if (systemInitDelay > 0) {
    systemInitDelay--;
    return;
  }

  if (odomPointerLast < 0) {
    return;
  }

  laserTime = odomTime[odomPointerLast] - depthCloudDelay;

  while (odomPointerFront != odomPointerLast) {
    int odomPointerNext = (odomPointerFront + 1) % odomQueLength;
    if (fabs(laserTime - odomTime[odomPointerFront]) < fabs(laserTime - odomTime[odomPointerNext])) {
      break;
    }
    odomPointerFront = (odomPointerFront + 1) % odomQueLength;
  }

  float sinDepthCamPitch = sin(depthCamPitchOffset);
  float cosDepthCamPitch = cos(depthCamPitchOffset);

  float sinVehicleRoll = sin(odomRoll[odomPointerFront]);
  float cosVehicleRoll = cos(odomRoll[odomPointerFront]);
  float sinVehiclePitch = sin(odomPitch[odomPointerFront]);
  float cosVehiclePitch = cos(odomPitch[odomPointerFront]);
  float sinVehicleYaw = sin(odomYaw[odomPointerFront]);
  float cosVehicleYaw = cos(odomYaw[odomPointerFront]);

  float vehicleX = odomX[odomPointerFront];
  float vehicleY = odomY[odomPointerFront];
  float vehicleZ = odomZ[odomPointerFront];

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloud);
  int laserCloudSize = laserCloud->points.size();

  laserCloudCrop->clear();
  for (int i = 0; i < laserCloudSize; i++) {
    if (laserCloud->points[i].z < maxRange) {
      laserCloudCrop->push_back(laserCloud->points[i]);
    }
  }

  laserCloudDwz->clear();
  downSizeFilter.setInputCloud(laserCloudCrop);
  downSizeFilter.filter(*laserCloudDwz);

  int laserCloudDwzSize = laserCloudDwz->points.size();
  for (int i = 0; i < laserCloudDwzSize; i++) {
      float pointX1 = laserCloudDwz->points[i].z;
      float pointY1 = -laserCloudDwz->points[i].x;
      float pointZ1 = -laserCloudDwz->points[i].y;

      float pointX2 = pointX1 * cosDepthCamPitch + pointZ1 * sinDepthCamPitch + depthCamXOffset;
      float pointY2 = pointY1 + depthCamYOffset;
      float pointZ2 = -pointX1 * sinDepthCamPitch + pointZ1 * cosDepthCamPitch + depthCamZOffset;

      float pointX3 = pointX2;
      float pointY3 = pointY2 * cosVehicleRoll - pointZ2 * sinVehicleRoll;
      float pointZ3 = pointY2 * sinVehicleRoll + pointZ2 * cosVehicleRoll;

      float pointX4 = pointX3 * cosVehiclePitch + pointZ3 * sinVehiclePitch;
      float pointY4 = pointY3;
      float pointZ4 = -pointX3 * sinVehiclePitch + pointZ3 * cosVehiclePitch;

      laserCloudDwz->points[i].x = pointX4 * cosVehicleYaw - pointY4 * sinVehicleYaw + vehicleX;
      laserCloudDwz->points[i].y = pointX4 * sinVehicleYaw + pointY4 * cosVehicleYaw + vehicleY;
      laserCloudDwz->points[i].z = pointZ4 + vehicleZ;
  }

  if (keepSurrCloud) {
    for (int i = 0; i < laserCloudDwzSize; i++) {
      float disX = laserCloudDwz->points[i].x - vehicleX;
      float disY = laserCloudDwz->points[i].y - vehicleY;
      float disZ = laserCloudDwz->points[i].z - vehicleZ;

      if (sqrt(disX * disX + disY * disY) < keepHoriDis && fabs(disZ) < keepVertDis) {
        laserCloudKeep->push_back(laserCloudDwz->points[i]);
      }
    }

    *laserCloudDwz += *laserCloudKeepDwz;

    laserCloudKeepDwz->clear();
    downSizeFilter.setInputCloud(laserCloudKeep);
    downSizeFilter.filter(*laserCloudKeepDwz);

    laserCloudKeep->clear();
    int laserCloudKeepDwzSize = laserCloudKeepDwz->points.size();
    for (int i = 0; i < laserCloudKeepDwzSize; i++) {
      float disX = laserCloudKeepDwz->points[i].x - vehicleX;
      float disY = laserCloudKeepDwz->points[i].y - vehicleY;
      float disZ = laserCloudKeepDwz->points[i].z - vehicleZ;

      if (sqrt(disX * disX + disY * disY) < keepHoriDis && fabs(disZ) < keepVertDis) {
        laserCloudKeep->push_back(laserCloudKeepDwz->points[i]);
      }
    }
  }

  newlaserCloud = true;
}

void trackPointHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  trackRoll = roll;
  trackPitch = pitch;
  trackYaw = yaw;
  trackX = odom->pose.pose.position.x;
  trackY = odom->pose.pose.position.y;
  trackZ = odom->pose.pose.position.z;
}

int readPlyHeader(FILE *filePtr)
{
  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
      }
    }
  }

  return pointNum;
}

void readStartPaths()
{
  string fileName = pathFolder + "/startPaths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, groupID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  fclose(filePtr);
}

#if PLOTPATHSET == 1
void readPaths()
{
  string fileName = pathFolder + "/paths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZI point;
  int pointSkipNum = 14;
  int pointSkipCount = 0;
  int val1, val2, val3, val4, val5, pathID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum) {
      pointSkipCount++;
      if (pointSkipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
      }
    }
  }

  fclose(filePtr);
}
#endif

void readPathList()
{
  string fileName = pathFolder + "/pathList.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  if (pathNum != readPlyHeader(filePtr)) {
    RCLCPP_INFO(nh->get_logger(), "Incorrect path number, exit.");
    exit(1);
  }

  int val1, val2, val3, val4, val5, pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++) {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;
      endPitchPathList[pathID] = -atan2(endZ, sqrt(endX * endX + endY * endY)) * 180.0 / PI;
      endYawPathList[pathID] = atan2(endY, endX) * 180.0 / PI;
      endZPathList[pathID] = endZ;
    }
  }

  fclose(filePtr);
}

void readCorrespondences()
{
  string fileName = pathFolder + "/correspondences.txt";

  FILE *filePtr = fopen(fileName.c_str(), "rb");
  if (filePtr == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  short pathID;
  int val1, gridVoxelID;
  for (int i = 0; i < gridVoxelNum; i++) {
    val1 = fread(&gridVoxelID, 4, 1, filePtr);
    if (val1 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
    }

    while (1) {
      val1 = fread(&pathID, 2, 1, filePtr);
      if (val1 != 1) {
        RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
          exit(1);
      }

      if (pathID != -1) {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
          correspondences[gridVoxelID].push_back(pathID);
        }
      } else {
        break;
      }
    }
  }

  fclose(filePtr);
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  if (joy->axes[2] >= -0.1 || joy->axes[5] < -0.1) {
    joyFwd = joy->axes[4];
    joyLeft = yawDiffLimit * joy->axes[3];
    joyUp = pitchDiffLimit * joy->axes[1];
  }

  if (joy->axes[5] < -0.1) {
    manualMode = true;
    autonomyMode = false;
    autoAdjustMode = false;
  } else {
    manualMode = false;

    if (joy->axes[2] < -0.1) {
      if (!autonomyMode) joyFwd = 1.0;
      autonomyMode = true;
    } else {
      autonomyMode = false;
      autoAdjustMode = false;
    }
  }

  if (joy->buttons[5] > 0.5) {
    laserCloudKeep->clear();
    laserCloudKeepDwz->clear();
  }
}

void goalHandler(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal)
{
  goalX = goal->point.x;
  goalY = goal->point.y;
  goalZ = goal->point.z;

  if (goalZ > maxElev) goalZ = maxElev;
}

void autoModeHandler(const std_msgs::msg::Float32::ConstSharedPtr autoMode)
{
  if (autonomyMode) {
    if (autoMode->data < -0.5) {
      autoAdjustMode = true;
    } else {
      autoAdjustMode = false;
    }

    joyFwd = autoMode->data;
    if (joyFwd < 0) joyFwd = 0;
    else if (joyFwd > 1.0) joyFwd = 1.0;
  }
}

void clearSurrCloudHandler(const std_msgs::msg::Empty::ConstSharedPtr clear)
{
  laserCloudKeep->clear();
  laserCloudKeepDwz->clear();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("localPlanner");

  nh->declare_parameter<std::string>("pathFolder", pathFolder);
  nh->declare_parameter<std::string>("stateEstimationTopic", stateEstimationTopic);
  nh->declare_parameter<bool>("autonomyMode", autonomyMode);
  nh->declare_parameter<std::string>("depthCloudTopic", depthCloudTopic);
  nh->declare_parameter<double>("depthCloudDelay", depthCloudDelay);
  nh->declare_parameter<double>("depthCamPitchOffset", depthCamPitchOffset);
  nh->declare_parameter<double>("depthCamXOffset", depthCamXOffset);
  nh->declare_parameter<double>("depthCamYOffset", depthCamYOffset);
  nh->declare_parameter<double>("depthCamZOffset", depthCamZOffset);
  nh->declare_parameter<bool>("trackingCamBackward", trackingCamBackward);
  nh->declare_parameter<double>("trackingCamXOffset", trackingCamXOffset);
  nh->declare_parameter<double>("trackingCamYOffset", trackingCamYOffset);
  nh->declare_parameter<double>("trackingCamZOffset", trackingCamZOffset);
  nh->declare_parameter<double>("trackingCamScale", trackingCamScale);
  nh->declare_parameter<double>("scanVoxelSize", scanVoxelSize);
  nh->declare_parameter<int>("pointPerPathThre", pointPerPathThre);
  nh->declare_parameter<double>("maxRange", maxRange);
  nh->declare_parameter<double>("maxElev", maxElev);
  nh->declare_parameter<bool>("keepSurrCloud", keepSurrCloud);
  nh->declare_parameter<double>("keepHoriDis", keepHoriDis);
  nh->declare_parameter<double>("keepVertDis", keepVertDis);
  nh->declare_parameter<double>("lowerBoundZ", lowerBoundZ);
  nh->declare_parameter<double>("upperBoundZ", upperBoundZ);
  nh->declare_parameter<double>("pitchDiffLimit", pitchDiffLimit);
  nh->declare_parameter<double>("pitchWeight", pitchWeight);
  nh->declare_parameter<double>("sensorMaxPitch", sensorMaxPitch);
  nh->declare_parameter<double>("sensorMaxYaw", sensorMaxYaw);
  nh->declare_parameter<double>("yawDiffLimit", yawDiffLimit);
  nh->declare_parameter<double>("yawWeight", yawWeight);
  nh->declare_parameter<double>("pathScale", pathScale);
  nh->declare_parameter<double>("minPathScale", minPathScale);
  nh->declare_parameter<double>("pathScaleStep", pathScaleStep);
  nh->declare_parameter<bool>("pathScaleBySpeed", pathScaleBySpeed);
  nh->declare_parameter<double>("stopDis", stopDis);
  nh->declare_parameter<bool>("shiftGoalAtStart", shiftGoalAtStart);
  nh->declare_parameter<double>("goalX", goalX);
  nh->declare_parameter<double>("goalY", goalY);
  nh->declare_parameter<double>("goalZ", goalZ);

  nh->get_parameter("pathFolder", pathFolder);
  nh->get_parameter("stateEstimationTopic", stateEstimationTopic);
  nh->get_parameter("autonomyMode", autonomyMode);
  nh->get_parameter("depthCloudTopic", depthCloudTopic);
  nh->get_parameter("depthCloudDelay", depthCloudDelay);
  nh->get_parameter("depthCamPitchOffset", depthCamPitchOffset);
  nh->get_parameter("depthCamXOffset", depthCamXOffset);
  nh->get_parameter("depthCamYOffset", depthCamYOffset);
  nh->get_parameter("depthCamZOffset", depthCamZOffset);
  nh->get_parameter("trackingCamBackward", trackingCamBackward);
  nh->get_parameter("trackingCamXOffset", trackingCamXOffset);
  nh->get_parameter("trackingCamYOffset", trackingCamYOffset);
  nh->get_parameter("trackingCamZOffset", trackingCamZOffset);
  nh->get_parameter("trackingCamScale", trackingCamScale);
  nh->get_parameter("scanVoxelSize", scanVoxelSize);
  nh->get_parameter("pointPerPathThre", pointPerPathThre);
  nh->get_parameter("maxRange", maxRange);
  nh->get_parameter("maxElev", maxElev);
  nh->get_parameter("keepSurrCloud", keepSurrCloud);
  nh->get_parameter("keepHoriDis", keepHoriDis);
  nh->get_parameter("keepVertDis", keepVertDis);
  nh->get_parameter("lowerBoundZ", lowerBoundZ);
  nh->get_parameter("upperBoundZ", upperBoundZ);
  nh->get_parameter("pitchDiffLimit", pitchDiffLimit);
  nh->get_parameter("pitchWeight", pitchWeight);
  nh->get_parameter("sensorMaxPitch", sensorMaxPitch);
  nh->get_parameter("sensorMaxYaw", sensorMaxYaw);
  nh->get_parameter("yawDiffLimit", yawDiffLimit);
  nh->get_parameter("yawWeight", yawWeight);
  nh->get_parameter("pathScale", pathScale);
  nh->get_parameter("minPathScale", minPathScale);
  nh->get_parameter("pathScaleStep", pathScaleStep);
  nh->get_parameter("pathScaleBySpeed", pathScaleBySpeed);
  nh->get_parameter("stopDis", stopDis);
  nh->get_parameter("shiftGoalAtStart", shiftGoalAtStart);
  nh->get_parameter("goalX", goalX);
  nh->get_parameter("goalY", goalY);
  nh->get_parameter("goalZ", goalZ);

  if (goalZ > maxElev) goalZ = maxElev;
  if (autonomyMode) {
    manualMode = false;
    joyFwd = 1.0;
  }

  auto subStateEstimation = nh->create_subscription<nav_msgs::msg::Odometry>(stateEstimationTopic, 5, stateEstimationHandler);

  auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>(depthCloudTopic, 5, laserCloudHandler);

  auto subTrackPoint = nh->create_subscription<nav_msgs::msg::Odometry>("/track_point_odom", 5, trackPointHandler);

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);

  auto subGoal = nh->create_subscription<geometry_msgs::msg::PointStamped> ("/way_point", 5, goalHandler);

  auto subAutoMode = nh->create_subscription<std_msgs::msg::Float32> ("/auto_mode", 5, autoModeHandler);

  auto subClearSurrCloud = nh->create_subscription<std_msgs::msg::Empty> ("/clear_surr_cloud", 5, clearSurrCloudHandler);

  auto pubPath = nh->create_publisher<nav_msgs::msg::Path>("/path", 5);
  nav_msgs::msg::Path path;

  #if PLOTPATHSET == 1
  auto pubFreePaths = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/free_paths", 2);

  auto pubLaserCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/collision_avoidance_cloud", 2);
  #endif

  RCLCPP_INFO(nh->get_logger(), "Reading path files.");

  for (int i = 0; i < laserCloudStackNum; i++) {
    laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  for (int i = 0; i < groupNum; i++) {
    startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  #if PLOTPATHSET == 1
  for (int i = 0; i < pathNum; i++) {
    paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  #endif
  for (int i = 0; i < gridVoxelNum; i++) {
    correspondences[i].resize(0);
  }

  downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

  readStartPaths();
  #if PLOTPATHSET == 1
  readPaths();
  #endif
  readPathList();
  readCorrespondences();

  RCLCPP_INFO(nh->get_logger(), "Initialization complete.");

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (newlaserCloud) {
      newlaserCloud = false;

      laserCloudStack[laserCloudCount]->clear();
      *laserCloudStack[laserCloudCount] = *laserCloudDwz;
      laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

      plannerCloudStack->clear();
      for (int i = 0; i < laserCloudStackNum; i++) {
        *plannerCloudStack += *laserCloudStack[i];
      }

      plannerCloud->clear();
      downSizeFilter.setInputCloud(plannerCloudStack);
      downSizeFilter.filter(*plannerCloud);

      #if PLOTPATHSET == 1
      sensor_msgs::msg::PointCloud2 plannerCloud2;
      pcl::toROSMsg(*plannerCloud, plannerCloud2);
      plannerCloud2.header.stamp = rclcpp::Time(static_cast<uint64_t>(laserTime * 1e9));
      plannerCloud2.header.frame_id = "map";
      pubLaserCloud->publish(plannerCloud2);
      #endif

      float sinTrackPitch = sin(trackPitch);
      float cosTrackPitch = cos(trackPitch);
      float sinTrackYaw = sin(trackYaw);
      float cosTrackYaw = cos(trackYaw);

      int plannerCloudSize = plannerCloud->points.size();
      for (int i = 0; i < plannerCloudSize; i++) {
        float pointX1 = plannerCloud->points[i].x - trackX;
        float pointY1 = plannerCloud->points[i].y - trackY;
        float pointZ1 = plannerCloud->points[i].z - trackZ;

        float pointX2 = pointX1 * cosTrackYaw + pointY1 * sinTrackYaw;
        float pointY2 = -pointX1 * sinTrackYaw + pointY1 * cosTrackYaw;
        float pointZ2 = pointZ1;

        plannerCloud->points[i].x = pointX2 * cosTrackPitch - pointZ2 * sinTrackPitch;
        plannerCloud->points[i].y = pointY2;
        plannerCloud->points[i].z = pointX2 * sinTrackPitch + pointZ2 * cosTrackPitch;
      }

      bool pathPublished = false;
      float pathScaleOri = pathScale;
      if (manualMode || (autonomyMode && autoAdjustMode)) pathScale = minPathScale;
      else if (pathScaleBySpeed) pathScale *= joyFwd;
      if (pathScale < minPathScale) pathScale = minPathScale;

      while (pathScale >= minPathScale) {
        for (int i = 0; i < pathNum; i++) {
          clearPathList[i] = 0;
        }
        for (int i = 0; i < groupNum; i++) {
          clearPathPerGroupScore[i] = 0;
        }

        float goalX1 = (goalX - trackX) * cosTrackYaw + (goalY - trackY) * sinTrackYaw;
        float goalY1 = -(goalX - trackX) * sinTrackYaw + (goalY - trackY) * cosTrackYaw;
        float goalZ1 = goalZ - trackZ;

        float relativeGoalX = (goalX1 * cosTrackPitch - goalZ1 * sinTrackPitch);
        float relativeGoalY = goalY1;
        float relativeGoalZ = (goalX1 * sinTrackPitch + goalZ1 * cosTrackPitch);

        float relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
        float relativeGoalPitch = -atan2(relativeGoalZ, sqrt(relativeGoalX * relativeGoalX 
                              + relativeGoalY * relativeGoalY)) * 180.0 / PI;
        float relativeGoalYaw = atan2(relativeGoalY, relativeGoalX) * 180.0 / PI;

        if (relativeGoalPitch < -pitchDiffLimit) relativeGoalPitch = -pitchDiffLimit;
        else if (relativeGoalPitch > pitchDiffLimit) relativeGoalPitch = pitchDiffLimit;
        if (relativeGoalYaw < -yawDiffLimit) relativeGoalYaw = -yawDiffLimit;
        else if (relativeGoalYaw > yawDiffLimit) relativeGoalYaw = yawDiffLimit;

        if (manualMode || (autonomyMode && autoAdjustMode)) {
          relativeGoalDis = 1000.0;
          relativeGoalPitch = 0;
          relativeGoalYaw = 0;
        } else if (!autonomyMode) {
          relativeGoalDis = 1000.0;
          relativeGoalPitch = -joyUp;
          relativeGoalYaw = joyLeft;
        }

        for (int i = 0; i < plannerCloudSize; i++) {
          float x = plannerCloud->points[i].x / pathScale;
          float y = plannerCloud->points[i].y / pathScale;
          float z = plannerCloud->points[i].z / pathScale;
          float dis = sqrt(x * x + y * y);

          if (x > 0 && (dis <= (relativeGoalDis + stopDis) / pathScale || relativeGoalX < 0) && 
              z > lowerBoundZ / pathScale && z < upperBoundZ / pathScale) {
            float scaleY = x / gridVoxelOffsetX + searchRadiusHori / gridVoxelOffsetY
                         * (gridVoxelOffsetX - x) / gridVoxelOffsetX;
            float scaleZ = x / gridVoxelOffsetX + searchRadiusVert / gridVoxelOffsetZ
                         * (gridVoxelOffsetX - x) / gridVoxelOffsetX;

            int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x) / gridVoxelSize);
            int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y / scaleY) / gridVoxelSize);
            int indZ = int((gridVoxelOffsetZ + gridVoxelSize / 2 - z / scaleZ) / gridVoxelSize);
            if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY && 
                indZ >= 0 && indZ < gridVoxelNumZ) {
              int ind = gridVoxelNumY * gridVoxelNumZ * indX + gridVoxelNumZ * indY + indZ;
              int blockedPathByVoxelNum = correspondences[ind].size();
              for (int j = 0; j < blockedPathByVoxelNum; j++) {
                clearPathList[correspondences[ind][j]]++;
              }
            }
          }
        }

        for (int i = 0; i < pathNum; i++) {
          float vehiclePitch = odomPitch[odomPointerFront];
          float vehicleYaw = odomYaw[odomPointerFront];
          float pitch = endPitchPathList[i];
          float yaw = trackYaw * 180.0 / PI + endYawPathList[i];
          float pitchDiff = fabs(pitch + trackPitch * 180.0 / PI - vehiclePitch * 180.0 / PI - depthCamPitchOffset * 180.0 / PI);
          float yawDiff = fabs(yaw - vehicleYaw * 180.0 / PI);
          if (yawDiff > 180.0) yawDiff = 360.0 - yawDiff;
          float elev = trackZ + endZPathList[i];
          if (yawDiff > sensorMaxYaw || pitchDiff > sensorMaxPitch || elev > maxElev) {
            clearPathList[i] += pointPerPathThre;
            continue;
          }
          if (clearPathList[i] < pointPerPathThre) {
            float pitchDiff = fabs(relativeGoalPitch - endPitchPathList[i]);
            if (pitchDiff > pitchDiffLimit) {
              pitchDiff = pitchDiffLimit;
            }
            float yawDiff = fabs(relativeGoalYaw - endYawPathList[i]);
            if (yawDiff > 180.0) {
              yawDiff = 360.0 - yawDiff;
            }

            float score = (1 - pitchWeight * pitchDiff) * (1 - yawWeight * yawDiff);
            if (score < 0) score = 0;
            clearPathPerGroupScore[pathList[i]] += score + 0.000001;
          }
        }

        float maxScore = 0;
        int selectedGroupID = -1;
        for (int i = 0; i < groupNum; i++) {
          if (maxScore < clearPathPerGroupScore[i]) {
            maxScore = clearPathPerGroupScore[i];
            selectedGroupID = i;
          }
        }

        if (selectedGroupID >= 0 && (relativeGoalDis > stopDis || relativeGoalX > 0)) {
          int selectedPathLength = startPaths[selectedGroupID]->points.size();
          path.poses.resize(selectedPathLength);
          for (int i = 0; i < selectedPathLength; i++) {
            float x = startPaths[selectedGroupID]->points[i].x;
            float y = startPaths[selectedGroupID]->points[i].y;
            float z = startPaths[selectedGroupID]->points[i].z;
            float dis = sqrt(x * x + y * y);

            if (dis <= relativeGoalDis / pathScale || relativeGoalX < 0) {
              path.poses[i].pose.position.x = pathScale * x;
              path.poses[i].pose.position.y = pathScale * y;
              path.poses[i].pose.position.z = pathScale * z;
            } else {
              path.poses.resize(i);
              break;
            }
          }

          path.header.stamp = rclcpp::Time(static_cast<uint64_t>(laserTime * 1e9));
          path.header.frame_id = "track_point";
          pubPath->publish(path);
          pathPublished = true;

          #if PLOTPATHSET == 1
          freePaths->clear();
          pcl::PointXYZI point;
          for (int i = 0; i < pathNum; i++) {
            if (clearPathList[i] < pointPerPathThre) {
              int freePathLength = paths[i]->points.size();
              for (int j = 0; j < freePathLength; j++) {
                point = paths[i]->points[j];
                float dis = sqrt(point.x * point.x + point.y * point.y);
                if (dis <= (relativeGoalDis + stopDis) / pathScale || relativeGoalX < 0) {
                  point.x *= pathScale;
                  point.y *= pathScale;
                  point.z *= pathScale;

                  freePaths->push_back(point);
                }
              }
            }
          }

          sensor_msgs::msg::PointCloud2 freePaths2;
          pcl::toROSMsg(*freePaths, freePaths2);
          freePaths2.header.stamp = rclcpp::Time(static_cast<uint64_t>(laserTime * 1e9));
          freePaths2.header.frame_id = "track_point";
          pubFreePaths->publish(freePaths2);
          #endif
        }

        if (selectedGroupID < 0) {
          pathScale -= pathScaleStep;
        } else {
          break;
        }
      }
      pathScale = pathScaleOri;

      if (!pathPublished) {
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;

        path.header.stamp = rclcpp::Time(static_cast<uint64_t>(laserTime * 1e9));
        path.header.frame_id = "track_point";
        pubPath->publish(path);

        #if PLOTPATHSET == 1
        freePaths->clear();
        sensor_msgs::msg::PointCloud2 freePaths2;
        pcl::toROSMsg(*freePaths, freePaths2);
        freePaths2.header.stamp = rclcpp::Time(static_cast<uint64_t>(laserTime * 1e9));
        freePaths2.header.frame_id = "track_point";
        pubFreePaths->publish(freePaths2);
        #endif
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
