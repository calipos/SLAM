#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <sstream>
#include <mutex>
#include <condition_variable>
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
#include "timer.h"
#define PIx2 (6.28318530717958647692528676656)
#define PI (3.1415926535897932384626433832795)
#define	BUF_SIZE	1210 
#define UDPPORT 2368

std::mutex mtx; // 全局相互排斥锁.
std::condition_variable cv; // 全局条件变量.
bool ready = true; // 全局标志位.
bool closeThread = false; // 全局标志位.

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
struct ringPts
{};
int velodyneDriver(param& config_)
{
    config_.last_azimuth_ = -1;
    config_.time_offset = 0;
    config_.frame_id = "velodyne";
    config_.model = "VLP16"; 
    double packet_rate;                   // packet frequency (Hz)
    std::string model_full_name;
    if ((config_.model == "64E_S2") || (config_.model == "64E_S2.1")) // generates 1333312 points per second
    {                                   // 1 packet holds 384 points
        packet_rate = 3472.17;            // 1333312 / 384
        model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "64E")
    {
        packet_rate = 2600.0;
        model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "64E_S3") // generates 2222220 points per second (half for strongest and half for lastest)
    {                                 // 1 packet holds 384 points
        packet_rate = 5787.03;          // 2222220 / 384
        model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "32E")
    {
        packet_rate = 1808.0;
        model_full_name = std::string("HDL-") + config_.model;
    }
    else if (config_.model == "32C")
    {
        packet_rate = 1507.0;
        model_full_name = std::string("VLP-") + config_.model;
    }
    else if (config_.model == "VLP16")
    {
        packet_rate = 754;             // 754 Packets/Second for Last or Strongest mode 1508 for dual (VLP-16 User Manual)
        model_full_name = "VLP-16";
    }
    else
    {
        return -1;
        packet_rate = 2600.0;
    }
    std::string deviceName(std::string("Velodyne ") + model_full_name);
    config_.rpm = 600.0;
    std::cout << deviceName << " rotating at " << config_.rpm << " RPM" << std::endl;
    double frequency = (config_.rpm / 60.0);   
    config_.npackets = (int)ceil(packet_rate / frequency); 
    std::cout << config_.npackets << " packets per scan" << std::endl;
    config_.timestamp_first_packet = false;
    if (config_.timestamp_first_packet)
        std::cout << "Setting velodyne scan start time to timestamp of first packet" << std::endl; 
    config_.cut_angle0 = -0.01;
    double cut_angle = -0.01;
    if (cut_angle < 0.0)
    {
        std::cout << "Cut at specific angle feature deactivated." << std::endl;;
    }
    else if (cut_angle < PIx2)
    {
        std::cout << "Cut at specific angle feature activated. Cutting velodyne points always at " << cut_angle << " rad." << std::endl;;
    }
    else
    {
        std::cout << "cut_angle parameter is out of range. Allowed range is between 0.0 and 2*PI or negative values to deactivate this feature." << std::endl;
        cut_angle = -0.01;
    }
        
    config_.cut_angle = int((cut_angle * 360 / PIx2) * 100);
    const double diag_freq = packet_rate / config_.npackets;
    std::cout << "expected frequency: "<< diag_freq <<"  (Hz)" << std::endl;;

    // raw packet output topic
    //output_ =  node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);

    int confd;
 
    char recvline[BUF_SIZE];
    struct sockaddr_in serveraddr;
#ifdef _MSC_VER
    WSADATA wsd;
    int iRet = 0;
    if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0)
    {
        iRet = WSAGetLastError();
        printf("WSAStartup failed !\n");
        return -1;
    }
#else
    bzero(&serveraddr, sizeof(serveraddr));
#endif
    // 使用socket()，生成套接字文件描述符；
    if ((confd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        std::cout<< "socket() error" <<std::endl;
        return -1;
    } 
    serveraddr.sin_family = AF_INET;
#ifdef _MSC_VER
    serveraddr.sin_addr.S_un.S_addr = (INADDR_ANY);
#else
    serveraddr.sin_addr.s_addr = (INADDR_ANY);
#endif
    serveraddr.sin_port = htons(UDPPORT);
    int ret = bind(confd, (struct sockaddr*)&serveraddr, sizeof(serveraddr));
    if (ret < 0)
    {
        std::cout << "bind() error" << std::endl;
#ifdef _MSC_VER
        closesocket(confd);
        WSACleanup();
#else
        close(confd);
#endif
        return -1;
    } 
    config_.confd = confd;
    config_.serveraddr = serveraddr;
    return 0;
}

int getPacket(char* packet, param& config_)
{
    double time1 = getCurrentTime();

#ifndef _MSC_VER
    // Unfortunately, the Linux kernel recvfrom() implementation
// uses a non-interruptible sleep() when waiting for data,
// which would cause this method to hang if the device is not
// providing data.  We poll() the device first to make sure
// the recvfrom() will not block.
// Note, however, that there is a known Linux kernel bug:
//   Under Linux, select() may report a socket file descriptor
//   as "ready for reading", while nevertheless a subsequent
//   read blocks.  This could for example happen when data has
//   arrived but upon examination has wrong checksum and is
//   discarded.  There may be other circumstances in which a
//   file descriptor is spuriously reported as ready.  Thus it
//   may be safer to use O_NONBLOCK on sockets that should not
//   block.
// poll() until input available
    struct pollfd fds[1];
    fds[0].fd = config_.confd;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 1000; // one second (in msec)
    do
    {
        int retval = poll(fds, 1, POLL_TIMEOUT);
        if (retval < 0)             // poll() error?
        {
            if (errno != EINTR)
                std::cout << "poll() error: " << strerror(errno) << std::endl;;
            return -1;
        }
        if (retval == 0)            // poll() timeout?
        {
            std::cout << "Velodyne poll() timeout" << std::endl;;
            return -1;
        }
        if ((fds[0].revents & POLLERR)
            || (fds[0].revents & POLLHUP)
            || (fds[0].revents & POLLNVAL)) // device error?
        {
            std::cout << "poll() reports Velodyne error" << std::endl;;
            return -1;
}
    } while ((fds[0].revents & POLLIN) == 0);
#endif
    int recv_length = 0;
#ifdef _MSC_VER
    recv_length = recvfrom(config_.confd, packet+sizeof(double), BUF_SIZE, 0, (sockaddr*)&(config_.serveraddr), &(config_.addr_length));
#else
    recv_length = recvfrom(config_.confd, packet + sizeof(double), BUF_SIZE, 0, (struct sockaddr*)&(config_.serveraddr), &(config_.addr_length));
#endif
    if (recv_length<0)
    {
        return -1;
    }
    std::cout << "recv_length = " << recv_length << std::endl;
    double time2 = getCurrentTime();
    *(double*)packet = (time2 + time1) *.5 + config_.time_offset;
    return 0;
}

template<>
template<>
int DataPipe<LidarPackets>::initSource<param>(param& config_)
{
    int ret = velodyneDriver(config_);
    return ret;
}
template<>
template<>
int DataPipe<LidarPackets>::getData<param>(param& config_)
{    
    auto& udpPackets = pushData2();
    udpPackets.data.clear();
    udpPackets.data.reserve(config_.npackets);
    if (config_.cut_angle >= 0) //Cut at specific angle feature enabled
    {
        while (true)
        {
            velodyneUdpPacket tmp_packet;
            while (true)
            {                
                std::unique_lock <std::mutex> lck(mtx);
                ready = false;                
                int rc = getPacket(&tmp_packet.data[0], config_);
                if (rc == 0) break;       // got a full packet?
                if (rc < 0) return -1; // end of file reached?
            }
            udpPackets.data.emplace_back(tmp_packet);

            // Extract base rotation of first block in packet
            std::size_t azimuth_data_pos = 100 * 0 + 2;
            int azimuth = *((uint16_t*)(&tmp_packet.data[azimuth_data_pos]));

            //if first packet in scan, there is no "valid" last_azimuth_
            if (config_.last_azimuth_ == -1) {
                config_.last_azimuth_ = azimuth;
                continue;
            }
            if ((config_.last_azimuth_ < config_.cut_angle && config_.cut_angle <= azimuth)
                || (config_.cut_angle <= azimuth && azimuth < config_.last_azimuth_)
                || (azimuth < config_.last_azimuth_ && config_.last_azimuth_ < config_.cut_angle))
            {
                config_.last_azimuth_ = azimuth;
                break; // Cut angle passed, one full revolution collected
            }
            config_.last_azimuth_ = azimuth;
        }
    }
    else // standard behaviour
    {
        udpPackets.data.resize(config_.npackets);
        for (int i = 0; i < config_.npackets; ++i)
        {
            velodyneUdpPacket&tmp_packet= udpPackets.data[i];
            while (true)
            {
                std::unique_lock <std::mutex> lck(mtx);
                ready = false;
                int rc = getPacket(&tmp_packet.data[0], config_);
                if (rc == 0) break;       // got a full packet?
                if (rc < 0) return -1; // end of file reached?
            }
        }
    }
    if (config_.timestamp_first_packet) 
    {
        udpPackets.headerStamp = *(double*)(&udpPackets.data[0]);
    }
    else 
    {
        udpPackets.headerStamp = *(double*)(&udpPackets.data[udpPackets.data.size() - 1]);
    }
    udpPackets.frame_id = config_.frame_id;
}
template<>
template<>
int DataPipe<LidarPackets>::setQueueAttr<param>(param& config_)
{
    cuurentPosFlag = 0;
    totalSize = -1; 
    for (int i = 0; i < queue.size(); i++)
    {
        queue[i].data.reserve(config_.npackets);
    }
    return 0;
}

LidarPackets tempData;
int pullData(const void*pipePtr) 
{
    const DataPipe<LidarPackets>& pipe = *(const DataPipe<LidarPackets>*)pipePtr;
    {
        std::unique_lock <std::mutex> lck(mtx);
        ready = false;
    }
    if (pipe.totalSize<0)
    {
        return -1;
    }
    int lastIdx = pipe.cuurentPosFlag - 1;
    if (lastIdx < 0)lastIdx = pipe.queue.size() - 1;
    pipe.queue[lastIdx].copyTo(tempData);
    {
        std::unique_lock <std::mutex> lck(mtx);
        ready = true;
        cv.notify_all();
    }
    //config_.fixed_frame must == ""
    //for (int i = 0; i < tempData.data.size(); ++i)
    //{
    //    data_->unpack(scanMsg->packets[i], *container_ptr, scanMsg->header.stamp);
    //}
    //output_.publish(container_ptr->finishCloud());
    return 0; ;
}
int ThreadProc1(DataPipe<LidarPackets>& pipe)
{

    param config_;
    pipe.initSource(config_);
    pipe.setQueueAttr(config_);
    while (true)
    {
        pipe.getData(config_);
    }
    return 0;
}
int main()
{
    DataPipe<LidarPackets> pipe(200);
    std::thread t1(ThreadProc1, std::ref(pipe));
    std::this_thread::sleep_for(std::chrono::seconds(2));
    pullData(&pipe);
    for (;;)
    {
        {
            std::unique_lock <std::mutex> lck(mtx);
            ready = false;
        }
        {
            std::unique_lock <std::mutex> lck(mtx);
            ready = true; 
            cv.notify_all();
        }
    } 
    t1.join();
    return 0;
}
