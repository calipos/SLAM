
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
        std::optional<velodyne_pointcloud::Calibration> calibration = data_->setup();
        if (calibration)
        {
            ROS_DEBUG_STREAM("Calibration file loaded."); 
            config_.num_lasers = static_cast<uint16_t>(calibration->num_lasers);
        }
        else
        {
            config_.num_lasers = 16;
            ROS_ERROR_STREAM("Could not load calibration file!");
        }
        reconfigure(config,  level);
    }

    void Transform::reconfigure(
        velodyne_pointcloud::TransformNodeConfig& config, uint32_t level)
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


        if ( config.organize_cloud != config_.organize_cloud) {

            config_.organize_cloud = config.organize_cloud;
            if (config_.organize_cloud)
            {
                ROS_INFO_STREAM("Using the organized cloud format...");
                container_ptr = std::shared_ptr<OrganizedCloudXYZIRT>(
                    new OrganizedCloudXYZIRT(config_.max_range, config_.min_range,
                        config_.target_frame, config_.fixed_frame,
                        config_.num_lasers, data_->scansPerPacket()));
                //container_ptr = std::shared_ptr<velodyne_rawdata::DataContainerBase>(
                //    new velodyne_rawdata::DataContainerBase(config_.max_range, config_.min_range,
                //        config_.target_frame, config_.fixed_frame, 0, false,
                //        config_.num_lasers, data_->scansPerPacket()));
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

    void Transform::processScan(const LidarPackets& packets)
    {
        container_ptr->setup(&packets);
        for (int i = 0; i < packets.data.size(); i++)
        {
            data_->unpack(packets.data[i], *container_ptr, packets.headerStamp);
        }
    }

    /** @brief Callback for raw scan messages.
     *
     *  @pre TF message filter has already waited until the transform to
     *       the configured @c frame_id can succeed.
     */
    void
        Transform::processScan(const velodyne_msgs::VelodyneScan::ConstPtr& scanMsg)
    {
        container_ptr->setup(scanMsg);

        // process each packet provided by the driver
        for (size_t i = 0; i < scanMsg->data.size(); ++i)
        {
            // calculate individual transform for each packet to account for ego
            // during one rotation of the velodyne sensor
            data_->unpack(scanMsg->data[i], *container_ptr, scanMsg->headerStamp);
        } 
    }

} // namespace velodyne_pointcloud
