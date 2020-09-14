#ifndef VELODYNE_POINTCLOUD_DATACONTAINERBASE_H
#define VELODYNE_POINTCLOUD_DATACONTAINERBASE_H
// Copyright (C) 2012, 2019 Austin Robot Technology, Jack O'Quin, Joshua Whitley, Sebastian P¨¹tz
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//#include <tf2_ros/transform_listener.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <velodyne_msgs/VelodyneScan.h>
//#include <sensor_msgs/point_cloud2_iterator.h>

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <algorithm>
#include <cstdarg>
#include "DataStruct.h"

namespace velodyne_rawdata
{
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
            manage_tf_buffer();

            cloud.header.stamp = scan_msg->headerStamp;
            cloud.data.resize(scan_msg->data.size() * config_.scans_per_packet * cloud.point_step);
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

        const ringPts& finishCloud()
        {
            cloud.data.resize(cloud.point_step * cloud.width * cloud.height);

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

        void manage_tf_buffer()
        {
        }

        void configure(const double max_range, const double min_range, const std::string fixed_frame,
            const std::string target_frame)
        {
            config_.max_range = max_range;
            config_.min_range = min_range;
            config_.fixed_frame = fixed_frame;
            config_.target_frame = target_frame;

            manage_tf_buffer();
        }

        ringPts cloud;



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
