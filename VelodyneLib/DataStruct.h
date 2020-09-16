#ifndef _DATA_STRUCT_H_
#define _DATA_STRUCT_H_ 
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include<time.h>

#define ROS_WARN(x) printf(x)
#define ROS_ERROR(x) printf(x)
#define ROS_INFO_STREAM(x) std::cout<<x<<std::endl;
#define ROS_ERROR_STREAM(x) std::cout<<x<<std::endl;
#define ROS_DEBUG_STREAM(x) std::cout<<x<<std::endl;
#define ROS_WARN_STREAM_THROTTLE(x,y)
#define PIx2 (6.28318530717958647692528676656)
#define PI (3.1415926535897932384626433832795)

struct velodyneUdpPacket
{
    std::vector<char> data;
    velodyneUdpPacket() { data.resize(sizeof(double) + 1206); }
};
struct LidarPackets
{
    std::string frame_id;
    int packetNumInside_;
    double headerStamp;
    std::vector<velodyneUdpPacket>data;//time+velodyneUdpPacket
    LidarPackets()
    {
        packetNumInside_ = 0;
        data = std::vector<velodyneUdpPacket>(0);
    }
    int copyTo(LidarPackets& other) const
    {
        other.frame_id = frame_id;
        other.packetNumInside_ = packetNumInside_;
        other.headerStamp = headerStamp;
        other.data.resize(data.size());
        memcpy(&(other.data)[0], &data[0], data.size());
        return 0;
    }
};
namespace ros
{
    typedef   double Time;
    struct NodeHandle
    {};
}
namespace velodyne_msgs
{
    typedef velodyneUdpPacket VelodynePacket;
    namespace VelodyneScan
    {
        typedef const LidarPackets* ConstPtr;
    }

}
#endif // !_DATA_STRUCT_H_