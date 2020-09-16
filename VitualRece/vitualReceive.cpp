#include <iostream>
#include<string>
#include <boost/asio.hpp>
//#include <stdlib.h>

using namespace std;
using namespace boost::asio;
//void main()
//{
//    io_service io_serviceA;
//    ip::udp::socket udp_socket(io_serviceA);
//    ip::udp::endpoint local_add(ip::udp::v4(), 2368);
//    udp_socket.open(local_add.protocol());
//    udp_socket.bind(local_add);
//    char receive_str[1206] = { 0 };
//    while (1)
//    {
//        ip::udp::endpoint  sendpoint;
//        udp_socket.receive_from(buffer(receive_str, 1206), sendpoint);
//        cout << "get = " << receive_str << endl; 
//    }
//    cin.get();
//}
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
    boost::asio::ip::udp::socket* udp_socket;
};
int p1(param&p)
{
    p.udp_socket = new ip::udp::socket(p.io_serviceA);
    ip::udp::endpoint local_add(ip::udp::v4(), 2368);
    p.udp_socket->open(local_add.protocol());
    p.udp_socket->bind(local_add);
    char buffer[1206];
   
        ip::udp::endpoint  sendpoint;
        int len = p.udp_socket->receive_from(boost::asio::buffer(buffer, 1206), sendpoint);
        cout << "get = " << len << endl;
    
    return 0;
}
int p2(param& p, char* buffer)
{
    while (1)
    {
        ip::udp::endpoint  sendpoint;
        int len = p.udp_socket->receive_from(boost::asio::buffer(buffer, 1206), sendpoint);
        cout << "get = " << len << endl;
    }
    return 0;
}
int main()
{
    char buffer[1206];
    param  p;
    p1(p);
    p2(p,buffer);
    return 0;
}