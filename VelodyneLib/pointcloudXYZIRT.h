/*

#ifndef VELODYNE_POINTCLOUD_POINTCLOUDXYZIRT_H
#define VELODYNE_POINTCLOUD_POINTCLOUDXYZIRT_H

#include "datacontainerbase.h"
#include <string>

namespace velodyne_pointcloud
{
class PointcloudXYZIRT : public velodyne_rawdata::DataContainerBase
{
public:
  PointcloudXYZIRT(const double max_range, const double min_range, const std::string& target_frame,
                   const std::string& fixed_frame, const unsigned int scans_per_block);

  virtual void newLine();

  virtual void setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg);

  virtual void addPoint(float x, float y, float z, const uint16_t ring, const uint16_t azimuth,
                        const float distance, const float intensity, const float time);

  sensor_msgs::PointCloud2Iterator<float> iter_x, iter_y, iter_z, iter_intensity, iter_time;
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring;
};
}  // namespace velodyne_pointcloud

#endif  // VELODYNE_POINTCLOUD_POINTCLOUDXYZIRT_H
*/