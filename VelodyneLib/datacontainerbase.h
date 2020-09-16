#ifndef VELODYNE_POINTCLOUD_DATACONTAINERBASE_H
#define VELODYNE_POINTCLOUD_DATACONTAINERBASE_H


//#include <tf2_ros/transform_listener.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <velodyne_msgs/VelodyneScan.h>
//#include <sensor_msgs/point_cloud2_iterator.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <algorithm>
#include <cstdarg>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include "DataStruct.h"
#ifdef NDEBUG
#pragma comment(lib,"shlwapi.lib")
#pragma comment(lib,"pcl_common.lib")
#pragma comment(lib,"pcl_filters.lib")
#pragma comment(lib,"pcl_features.lib")
#pragma comment(lib,"pcl_sample_consensus.lib")
#pragma comment(lib,"pcl_io.lib")
#pragma comment(lib,"pcl_io_ply.lib")
#pragma comment(lib,"pcl_kdtree.lib")
#pragma comment(lib,"pcl_registration.lib")
#pragma comment(lib,"pcl_search.lib")
#pragma comment(lib,"pcl_segmentation.lib")
#pragma comment(lib,"pcl_surface.lib")
#pragma comment(lib,"pcl_visualization.lib")
#pragma comment(lib,"flann_cpp_s.lib")
#endif
#ifdef _DEBUG
#pragma comment(lib,"shlwapi.lib")
#pragma comment(lib,"pcl_common.lib")
#pragma comment(lib,"pcl_filters.lib")
#pragma comment(lib,"pcl_features.lib")
#pragma comment(lib,"pcl_sample_consensus.lib")
#pragma comment(lib,"pcl_io.lib")
#pragma comment(lib,"pcl_io_ply.lib")
#pragma comment(lib,"pcl_kdtree.lib")
#pragma comment(lib,"pcl_registration.lib")
#pragma comment(lib,"pcl_search.lib")
#pragma comment(lib,"pcl_segmentation.lib")
#pragma comment(lib,"pcl_surface.lib")
#pragma comment(lib,"pcl_surface.lib")
#pragma comment(lib,"flann_cpp_s.lib")
#endif
namespace velodyne_rawdata
{
    struct header_
    {
        std::string frame_id;
        double stamp;
    };
    struct scanPoints
    {
        header_ header;
        int width;
        int height;
        uint8_t is_dense;
        pcl::PointCloud<pcl::PointXYZI>::Ptr xyzi;
        std::vector<uint16_t>ringIdx;
        std::vector<float>time;
        std::vector<int>fields;
        int point_step;
        int row_step;
        int totalCnt;
        scanPoints()
        {
            xyzi = NULL;
            ringIdx.clear();
        }
        void dataResize(const int& cnt)
        {
            if (xyzi == NULL)
            {
                totalCnt = cnt;
                pcl::PointCloud<pcl::PointXYZI>::Ptr temp(new pcl::PointCloud<pcl::PointXYZI>);
                temp->points.resize(cnt);
                xyzi = temp;
                ringIdx.resize(cnt);
                time.resize(cnt);
            }
            else
            {
                //xyzi->clear();
                //ringIdx.clear();
                //time.clear();
            }
        }
    };
            
    class DataContainerBase
    {
    public:
        DataContainerBase(const double max_range, const double min_range, const std::string& target_frame,
            const std::string& fixed_frame, const unsigned int init_width, const unsigned int init_height,
            const bool is_dense, const unsigned int scans_per_packet)//, int fields, ...)
            : config_(max_range, min_range, target_frame, fixed_frame, init_width, init_height, is_dense, scans_per_packet)
        {
            cloud.fields.clear();
            int offset = 1;
            cloud.point_step = offset;
            cloud.row_step = init_width * cloud.point_step;
            
        }

        struct Config
        {
            double max_range;          ///< maximum range to publish
            double min_range;          ///< minimum range to publish
            std::string target_frame;  ///< output frame of final point cloud
            std::string fixed_frame;   ///< world fixed frame for ego motion compenstation
            unsigned int init_width;
            unsigned int init_height;
            bool is_dense;
            unsigned int scans_per_packet;

            Config(double max_range, double min_range, std::string target_frame, std::string fixed_frame,
                unsigned int init_width, unsigned int init_height, bool is_dense, unsigned int scans_per_packet)
                : max_range(max_range)
                , min_range(min_range)
                , target_frame(target_frame)
                , fixed_frame(fixed_frame)
                , init_width(init_width)
                , init_height(init_height)
                , is_dense(is_dense)
                , scans_per_packet(scans_per_packet)
            {
                ROS_INFO_STREAM("Initialized container with "
                    << "min_range: " << min_range << ", max_range: " << max_range
                    << ", target_frame: " << target_frame << ", fixed_frame: " << fixed_frame
                    << ", init_width: " << init_width << ", init_height: " << init_height
                    << ", is_dense: " << is_dense << ", scans_per_packet: " << scans_per_packet);
            }
        };
   
        virtual void setup(const velodyne_msgs::VelodyneScan::ConstPtr& scan_msg)
        {
            sensor_frame = scan_msg->frame_id;
            cloud.header.stamp = scan_msg->headerStamp;
            cloud.dataResize(scan_msg->data.size() * config_.scans_per_packet * cloud.point_step);
            cloud.width = config_.init_width;
            cloud.height = config_.init_height;
            cloud.is_dense = static_cast<uint8_t>(config_.is_dense);
        }

        //virtual void addPoint(float x, float y, float z, const uint16_t ring, const uint16_t azimuth, const float distance,
        //    const float intensity, const float time) = 0;
        //virtual void newLine() = 0;
        virtual void addPoint(float x, float y, float z, const uint16_t ring, const uint16_t azimuth, const float distance,
            const float intensity, const float time) {};
        virtual void newLine() {};

        const scanPoints& finishCloud()
        {
            cloud.dataResize(cloud.point_step * cloud.width * cloud.height);

            if (!config_.target_frame.empty())
            {
                cloud.header.frame_id = config_.target_frame;
            }
            else if (!config_.fixed_frame.empty())
            {
                cloud.header.frame_id = config_.fixed_frame;
            }
            else
            {
                cloud.header.frame_id = sensor_frame;
            }

            ROS_DEBUG_STREAM("Prepared cloud width" << cloud.height * cloud.width
                << " Velodyne points, time: " << cloud.header.stamp);
            return cloud;
        }

 

        void configure(const double max_range, const double min_range, const std::string fixed_frame,
            const std::string target_frame)
        {
            config_.max_range = max_range;
            config_.min_range = min_range;
            config_.fixed_frame = fixed_frame;
            config_.target_frame = target_frame;

        }
        
        //pcl::PCLPointCloud2 cloud;
        scanPoints cloud;
        int idx;

        inline void transformPoint(float& x, float& y, float& z)
        {
            Eigen::Vector3f p = Eigen::Vector3f(x, y, z);
            if (!config_.fixed_frame.empty())
            {
                p = tf_matrix_to_fixed * p;
            }
            if (!config_.target_frame.empty())
            {
                p = tf_matrix_to_target * p;
            }
            x = p.x();
            y = p.y();
            z = p.z();
        }

        inline bool pointInRange(float range)
        {
            return (range >= config_.min_range && range <= config_.max_range);
        }

    protected:
        Config config_;
        Eigen::Affine3f tf_matrix_to_fixed;
        Eigen::Affine3f tf_matrix_to_target;
        std::string sensor_frame;
    };
}

#endif  // VELODYNE_POINTCLOUD_DATACONTAINERBASE_H
