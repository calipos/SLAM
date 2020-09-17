#include "ScanRegistration.h"


//#include <tf/transform_datatypes.h>


namespace loam {


bool ScanRegistration::setupROS(/*ros::NodeHandle& node, ros::NodeHandle& privateNode, */RegistrationParams& config_out)
{
  //// subscribe to IMU topic
  //_subImu = node.subscribe<sensor_msgs::Imu>("/imu/data", 50, &ScanRegistration::handleIMUMessage, this);
  //// advertise scan registration topics
  //_pubLaserCloud            = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 2);
  //_pubCornerPointsSharp     = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2);
  //_pubCornerPointsLessSharp = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2);
  //_pubSurfPointsFlat        = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2);
  //_pubSurfPointsLessFlat    = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2);
  //_pubImuTrans              = node.advertise<sensor_msgs::PointCloud2>("/imu_trans", 5);

  return true;
}



void ScanRegistration::handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn)
{
  //tf::Quaternion orientation;
  //tf::quaternionMsgToTF(imuIn->orientation, orientation);
  //double roll, pitch, yaw;
  //tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  //Vector3 acc;
  //acc.x() = float(imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81);
  //acc.y() = float(imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81);
  //acc.z() = float(imuIn->linear_acceleration.x + sin(pitch)             * 9.81);

  //IMUState newState;
  //newState.stamp = fromROSTime( imuIn->header.stamp);
  //newState.roll = roll;
  //newState.pitch = pitch;
  //newState.yaw = yaw;
  //newState.acceleration = acc;

  //updateIMUData(acc, newState);
}


void ScanRegistration::publishResult()
{
  auto sweepStartTime = toROSTime(sweepStart());
  // publish full resolution and feature point clouds
  //publishCloudMsg(_pubLaserCloud, laserCloud(), sweepStartTime, "/camera");
  //publishCloudMsg(_pubCornerPointsSharp, cornerPointsSharp(), sweepStartTime, "/camera");
  //publishCloudMsg(_pubCornerPointsLessSharp, cornerPointsLessSharp(), sweepStartTime, "/camera");
  //publishCloudMsg(_pubSurfPointsFlat, surfacePointsFlat(), sweepStartTime, "/camera");
  //publishCloudMsg(_pubSurfPointsLessFlat, surfacePointsLessFlat(), sweepStartTime, "/camera");

  //// publish corresponding IMU transformation information
  //publishCloudMsg(_pubImuTrans, imuTransform(), sweepStartTime, "/camera");
}

} // end namespace loam
