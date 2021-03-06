#include <fstream>
#include <functional>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <boost/asio.hpp>
#include "../logger/logger.h"
#include "../DataPipe/datapipe.h"
#include "transform.h"
#include "timer.h"
#define USE_VITUAL_UDP 1
#define PCL_SHOW 1
#if  PCL_SHOW>0
#include <pcl/visualization/cloud_viewer.h>
#endif //   PCL_SHOW>0

#define	BUF_SIZE__	(1210) 

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
    int last_azimuth_;
    boost::asio::io_service io_serviceA;
    boost::asio::ip::udp::socket*udp_socket;
};
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
    LOG(INFO) << deviceName << " rotating at " << config_.rpm << " RPM";
    double frequency = (config_.rpm / 60.0);   
    config_.npackets = (int)ceil(packet_rate / frequency); 
    LOG(INFO) << config_.npackets << " packets per scan";
    config_.timestamp_first_packet = false;
    if (config_.timestamp_first_packet)
        LOG(INFO) << "Setting velodyne scan start time to timestamp of first packet";
    config_.cut_angle0 = -0.01;
    double cut_angle = -0.01;
    if (cut_angle < 0.0)
    {
        LOG(INFO) << "Cut at specific angle feature deactivated.";
    }
    else if (cut_angle < PIx2)
    {
        LOG(INFO) << "Cut at specific angle feature activated. Cutting velodyne points always at " << cut_angle << " rad.";
    }
    else
    {
        LOG(INFO) << "cut_angle parameter is out of range. Allowed range is between 0.0 and 2*PI or negative values to deactivate this feature.";
        cut_angle = -0.01;
    }
        
    config_.cut_angle = int((cut_angle * 360 / PIx2) * 100);
    const double diag_freq = packet_rate / config_.npackets;
    LOG(INFO) << "expected frequency: " << diag_freq << "  (Hz)";

    // raw packet output topic
    //output_ =  node.advertise<velodyne_msgs::VelodyneScan>("velodyne_packets", 10);

    
    config_.udp_socket = new boost::asio::ip::udp::socket(config_.io_serviceA); 
    boost::asio::ip::udp::endpoint local_add(boost::asio::ip::udp::v4(), 2368);
    config_.udp_socket->open(local_add.protocol());
    config_.udp_socket->bind(local_add);

    return 0;
}

int getPacket(char* packet, param& config_)
{
    double time1 = getCurrentTime();
    int recv_length = 0;
    {
        boost::asio::ip::udp::endpoint  sendpoint;
        try
        {
            recv_length = config_.udp_socket->receive_from(boost::asio::buffer(packet + sizeof(double), 1206 ), sendpoint);
        }
        catch (const std::exception&e)
        {
            std::cout<<e.what()<<std::endl;
        }
    }
    //std::cout << "recv_length = " << recv_length << std::endl;
    if (recv_length<0)
    {
        return -1;
    }
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
int lidarGetUdp(param& config_, LidarPackets* mem)
{
    LidarPackets& udpPackets = *(LidarPackets*)mem;
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
            std::cout << azimuth << std::endl;
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
            velodyneUdpPacket& tmp_packet = udpPackets.data[i];
            while (true)
            {
                std::unique_lock <std::mutex> lck(mtx);
                if (ready)
                {
                    int rc = getPacket(&tmp_packet.data[0], config_);
                    if (rc == 0) break;       // got a full packet?
                    if (rc < 0) return -1; // end of file reached?
                }
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
    return 0;
}
 
template<>
template<>
int DataPipe<LidarPackets>::setQueueAttr<param>(param& config_)
{
    for (int i = 0; i < queue.size(); i++)
    {
        queue[i].data.reserve(config_.npackets);
    }
    return 0;
}

LidarPackets tempData;
int pullData(void*pipePtr) 
{
    DataPipe<LidarPackets>& pipe = *(DataPipe<LidarPackets>*)pipePtr;    
    pipe.pop(&tempData);
    return 0; ;
}
int ThreadProc1(DataPipe<LidarPackets>& pipe)
{
#ifdef USE_VITUAL_UDP>0
    param config_;
    CHECK(0==pipe.initSource(config_));
    pipe.setQueueAttr(config_);
    while (true)
    {
        pipe.pushData<int(param& config_,LidarPackets* packet), param>(lidarGetUdp, config_);        
    }
#else
    std::vector<char> data(BUF_SIZE__ * 10000);
    std::ifstream inF;
    inF.open("D:/SLAM/package.dat", std::ifstream::binary);
    inF.read(&data[0], BUF_SIZE__ * 10000);
    inF.close();
    int thisIdx = 0;
    for (int i = 0; i<pipe.queueLength_; i++)
    { 
        pipe.queue[i].headerStamp = getCurrentTime();
        pipe.queue[i].packetNumInside_ = 76;
        pipe.queue[i].data.resize(76);
        for (int  j = 0; j < 76; j++)
        {
            pipe.queue[i].data[j].data.resize(1206 + sizeof(double));
            int pos = thisIdx * BUF_SIZE__;
            if (pos + BUF_SIZE__ >= data.size())
            {
                pos = 0;
            }
            memcpy(&((&pipe.queue[i].data[j].data[0]+ sizeof(double))[0]),&data[pos], 1206 );
            *(double*)(&pipe.queue[i].data[j].data[0]) = getCurrentTime();
            thisIdx++;            
        } 
    }
    pipe.totalSize = 5000;
    pipe.cuurentPosFlag = 10;
#endif
    return 0;
}
int main()
{
    DataPipe<LidarPackets> pipe(100);
    std::thread t1(ThreadProc1, std::ref(pipe));
    std::this_thread::sleep_for(std::chrono::seconds(2));    
    velodyne_pointcloud::TransformNodeConfig config;        
    config.organize_cloud = true;
    config.min_range = 0.4;
    config.max_range = 130;
    pcl::PointCloud<pcl::PointXYZI>::Ptr showPc(new pcl::PointCloud<pcl::PointXYZI>);
    velodyne_pointcloud::Transform tf(config, 0); 
#if  PCL_SHOW>0
    pcl::visualization::CloudViewer viewer("pcd viewer");
#endif //   PCL_SHOW>0
    for (int i = 0; i < pipe.queueLength_; i++)
    {
        if (pullData(&pipe) < 0)continue;
        tf.processScan(tempData);
        //tf.processScan(pipe.queue[0]);
        LOG(INFO) << tf.container_ptr->idx;
        if (i == pipe.queueLength_ - 1)
        {
            i = -1;
        }
#if  PCL_SHOW>0
        pcl::copyPointCloud(*tf.container_ptr->cloud.xyzi, *showPc);
        showPc->height = 1;
        showPc->width = tf.container_ptr->idx;
        showPc->points.resize(tf.container_ptr->idx);
        viewer.showCloud(showPc);
        //std::this_thread::sleep_for(std::chrono::milliseconds(20));
#endif //   PCL_SHOW>0
        if (i == pipe.queueLength_ - 1)
        {
            i = -1;
        }
    }


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
