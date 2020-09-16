#include <fstream>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <boost/asio.hpp>
#ifdef NDEBUG
#include <pcl/visualization/cloud_viewer.h>
#endif
#include "../DataPipe/datapipe.h"
#include "transform.h"
#include "timer.h"
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
#ifdef NDEBUG
    param config_;
    std::cout << pipe.initSource(config_) << std::endl;;
    pipe.setQueueAttr(config_);
    while (true)
    {
        pipe.getData(config_);
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
    DataPipe<LidarPackets> pipe(200);
    std::thread t1(ThreadProc1, std::ref(pipe));
    std::this_thread::sleep_for(std::chrono::seconds(2));    
    velodyne_pointcloud::TransformNodeConfig config;        
    config.organize_cloud = true;
    config.min_range = 0.4;
    config.max_range = 130;
    velodyne_pointcloud::Transform tf(config, 0); 
#ifdef NDEBUG
    pcl::visualization::CloudViewer viewer("Cluster viewer");
    pcl::PointCloud<pcl::PointXYZI>::Ptr showPc(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < pipe.queueLength_; i++)
    {
        //continue;
        if (pullData(&pipe) < 0)continue;
        tf.processScan(tempData);

        std::cout << tf.container_ptr->idx << std::endl;
        //tf.processScan(pipe.queue[i]);
        pcl::copyPointCloud(*tf.container_ptr->cloud.xyzi, *showPc);
        showPc->height = 1;
        showPc->width = tf.container_ptr->idx;
        showPc->points.resize(tf.container_ptr->idx);
        viewer.showCloud(showPc);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        if (i == pipe.queueLength_-1)
        {
            i = -1;
        }
    }
#else
    //pcl::visualization::CloudViewer viewer("pcd viewer");    
    for (int i = 0; i < pipe.queueLength_; i++)
    {
        if (pullData(&pipe) < 0)continue;
        tf.processScan(tempData);
        tf.processScan(pipe.queue[i]);
        std::cout << tf.container_ptr->idx << std::endl;
        if (i == pipe.queueLength_ - 1)
        {
            i = -1;
        }
        //viewer.showCloud(tf.container_ptr->cloud.xyzi);
    }


#endif
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
