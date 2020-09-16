

#ifndef VELODYNE_POINTCLOUD_ORGANIZED_CLOUDXYZIRT_H
#define VELODYNE_POINTCLOUD_ORGANIZED_CLOUDXYZIRT_H

#include "datacontainerbase.h" 
#include <string>

namespace velodyne_pointcloud
{
class OrganizedCloudXYZIRT : public velodyne_rawdata::DataContainerBase
{
public:
  OrganizedCloudXYZIRT(const double max_range, const double min_range, const std::string& target_frame,
                       const std::string& fixed_frame, const unsigned int num_lasers,
                       const unsigned int scans_per_block);

  virtual void newLine();

  virtual void setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg);

  virtual void addPoint(float x, float y, float z, const uint16_t ring, const uint16_t azimuth, const float distance,
                        const float intensity, const float time);

private:
  int iter_x, iter_y, iter_z, iter_intensity, iter_time;
  int iter_ring;
};
}
#endif  // VELODYNE_POINTCLOUD_ORGANIZED_CLOUDXYZIRT_H
