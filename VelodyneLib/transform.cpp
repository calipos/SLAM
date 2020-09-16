
#include "transform.h"

#include "pointcloudXYZIRT.h"
#include "organized_cloudXYZIRT.h"

namespace velodyne_pointcloud
{
    /** @brief Constructor. */
    Transform::Transform(velodyne_pointcloud::TransformNodeConfig& config, uint32_t level) :
        data_(new velodyne_rawdata::RawData),
        first_rcfg_call(true)
    {
        int calibrationRet = data_->setup();
        if (calibrationRet==0)
        {
            ROS_DEBUG_STREAM("Calibration file loaded.");             
            config_.num_lasers = static_cast<uint16_t>(data_->calibration_.num_lasers);
        }
        else
        {
            config_.num_lasers = 16;
            ROS_ERROR_STREAM("Could not load calibration file!");
        }
        {
            ROS_INFO_STREAM("Reconfigure request.");
            data_->setParameters(config.min_range, config.max_range,
                config.view_direction, config.view_width);
            config_.target_frame = config.target_frame;
            config_.fixed_frame = config.fixed_frame;
            ROS_INFO_STREAM("Target frame ID now: " << config_.target_frame);
            ROS_INFO_STREAM("Fixed frame ID now: " << config_.fixed_frame);
            config_.min_range = config.min_range;
            config_.max_range = config.max_range;
            if ( config.organize_cloud != config_.organize_cloud) 
            {
                config_.organize_cloud = config.organize_cloud;
                if (config_.organize_cloud)
                {
                    ROS_INFO_STREAM("Using the organized cloud format...");
                    container_ptr = std::shared_ptr<OrganizedCloudXYZIRT>(
                        new OrganizedCloudXYZIRT(config_.max_range, config_.min_range,
                            config_.target_frame, config_.fixed_frame,
                            config_.num_lasers, data_->scansPerPacket()));
                }
                else
                {
                    //container_ptr = std::shared_ptr<PointcloudXYZIRT>(
                    //    new PointcloudXYZIRT(config_.max_range, config_.min_range,
                    //        config_.target_frame, config_.fixed_frame,
                    //        data_->scansPerPacket()));
                }
            }
            container_ptr->configure(config_.max_range, config_.min_range, config_.fixed_frame, config_.target_frame);
        }
    }
    void Transform::processScan(const LidarPackets& packets)
    {
        container_ptr->setup(&packets);
        for (int i = 0; i < packets.data.size(); i++)
        {
            data_->unpack(packets.data[i], *container_ptr, packets.headerStamp);
        }

    }


    //void Transform::processScan(const velodyne_msgs::VelodyneScan::ConstPtr& scanMsg)
    //{
    //    container_ptr->setup(scanMsg);
    //    for (size_t i = 0; i < scanMsg->data.size(); ++i)
    //    {
    //        data_->unpack(scanMsg->data[i], *container_ptr, scanMsg->headerStamp);
    //    } 
    //}

} // namespace velodyne_pointcloud
