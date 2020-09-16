#ifndef VELODYNE_POINTCLOUD_TRANSFORM_H
#define VELODYNE_POINTCLOUD_TRANSFORM_H

#include <string>
#include <memory>
#include "DataStruct.h"
#include "rawdata.h"
//#include <ros/ros.h>
//#include "tf/message_filter.h"
//#include "message_filters/subscriber.h"
//#include <diagnostic_updater/diagnostic_updater.h>
//#include <diagnostic_updater/publisher.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <velodyne_pointcloud/rawdata.h>
//#include <velodyne_pointcloud/pointcloudXYZIRT.h>
//#include <dynamic_reconfigure/server.h>
//#include <velodyne_pointcloud/TransformNodeConfig.h>

namespace velodyne_pointcloud
{
    struct TransformNodeConfig
    {
        double min_range;
        double max_range;
        double view_direction;
        double view_width;
        std::string target_frame;
        std::string fixed_frame;
        bool organize_cloud;
    };
    using TransformNodeCfg = velodyne_pointcloud::TransformNodeConfig;

    class Transform
    {
    public:
        Transform(velodyne_pointcloud::TransformNodeConfig& config, uint32_t level);
        ~Transform()
        {
        }
        void processScan(const LidarPackets& aPacket);
        std::shared_ptr<velodyne_rawdata::DataContainerBase> container_ptr;
    private:
        //void processScan(const velodyne_msgs::VelodyneScan::ConstPtr& scanMsg);

        std::shared_ptr<velodyne_rawdata::RawData> data_;


        /// configuration parameters
        typedef struct
        {
            std::string target_frame;  ///< target frame
            std::string fixed_frame;   ///< fixed frame
            bool organize_cloud;       ///< enable/disable organized cloud structure
            double max_range;          ///< maximum range to publish
            double min_range;          ///< minimum range to publish
            uint16_t num_lasers;       ///< number of lasers
        }
        Config;
        Config config_;

        bool first_rcfg_call;


    };
}  // namespace velodyne_pointcloud

#endif  // VELODYNE_POINTCLOUD_TRANSFORM_H
