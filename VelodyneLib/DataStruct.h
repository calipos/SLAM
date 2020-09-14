#ifndef _DATA_STRUCT_H_
#define _DATA_STRUCT_H_ 
#include <string>
#include <iostream>
#include <sstream>
#include <vector>
#include<time.h>
#ifdef _MSC_VER
#include <stdio.h>
#include <winsock2.h>
#pragma comment(lib,"ws2_32.lib")
#else
#include <unistd.h>
#include <string.h>
#include <stdio.h> 
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h> 
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#endif // 0
#include "../DataPipe/datapipe.h"
#define ROS_WARN(x) printf(x)
#define ROS_ERROR(x) printf(x)
#define ROS_INFO_STREAM(x) std::cout<<x<<std::endl;
#define ROS_ERROR_STREAM(x) std::cout<<x<<std::endl;
#define ROS_DEBUG_STREAM(x) std::cout<<x<<std::endl;
#define ROS_WARN_STREAM_THROTTLE(x,y)
#define PIx2 (6.28318530717958647692528676656)
#define PI (3.1415926535897932384626433832795)

struct param
{
    std::string frame_id;
    std::string model;
    double rpm;
    int npackets;
    bool timestamp_first_packet;
    double cut_angle0; //in
    double cut_angle;
    double time_offset;
    unsigned int confd;
    sockaddr_in serveraddr;
    int addr_length;
    int last_azimuth_;
};
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
struct pt3
{
    float x;
    float y;
    float z;
};
struct header_
{
    double stamp;
    std::string frame_id;
};
struct ringPts
{
    header_ header;
    std::vector<int>fields;
    std::vector<pt3>data;
    int point_step;
    int row_step;
    int width;
    int height;
    uint8_t is_dense;
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