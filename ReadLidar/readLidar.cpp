#include <iostream>
#include <string>
#include <vector>
#include <fstream>
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
#endif // 0

#define log(x) {std::cout<<x<<std::endl;}

#define	BUF_SIZE	1210 
#define UDPPORT 2368
#define SERVERIP  "192.168.1.201"

using namespace std;

int main_test() {
    int confd;
#ifdef _MSC_VER
    int addr_length;
#else
    unsigned int addr_length;
#endif
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
        log("socket() error");
        exit(1);
    }
    log("socket() ok");
    log(confd);
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
        log("bind fail:");
#ifdef _MSC_VER
        closesocket(confd);
        WSACleanup();
#else
        close(confd);
#endif
        return -1;
    }
    else
    {
        log("recv ready!!!\n");
    }


    log("begin recvfrom");
    // 接收服务器的数据，recvfrom() ；
    int recv_length = 0;
    for (;;)
    { 
#ifdef _MSC_VER
        recv_length = recvfrom(confd, recvline, BUF_SIZE, 0, (sockaddr*)&serveraddr, &addr_length);
#else
        recv_length = recvfrom(confd, recvline, BUF_SIZE, 0, (struct sockaddr*)&serveraddr, &addr_length);
#endif
        cout << "recv_length = " << recv_length << endl;
    }
    log("begin end");
    
    //cout << recvline << endl;

    // 关闭套接字，close() ；
#ifdef _MSC_VER
    closesocket(confd);
    WSACleanup();
#else
    close(confd);
#endif
    return 0;
}



int collect_lidar_data(std::vector<char>&data)
{
    int confd;
#ifdef _MSC_VER
    int addr_length;
#else
    unsigned int addr_length;
#endif
    data.resize(100000 * BUF_SIZE);
    //char recvline[BUF_SIZE];
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
        log("socket() error");
        exit(1);
    }
    log("socket() ok");
    log(confd);
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
        log("bind fail:");
#ifdef _MSC_VER
        closesocket(confd);
        WSACleanup();
#else
        close(confd);
#endif
        return -1;
    }
    else
    {
        log("recv ready!!!\n");
    }

    char* startPos = &data[0];
    log("begin recvfrom");
    for (int i = 0; ; i++)
    {
        if (BUF_SIZE*i< data.size())
        {
            char* recvline = &data[BUF_SIZE * i];
            int recv_length = 0;
#ifdef _MSC_VER
            recv_length = recvfrom(confd, recvline, BUF_SIZE, 0, (sockaddr*)&serveraddr, &addr_length);
#else
            recv_length = recvfrom(confd, recvline, BUF_SIZE, 0, (struct sockaddr*)&serveraddr, &addr_length);
#endif
            cout << "recv_length = " << recv_length << endl;
        }
        else
        {
            break;
        }
    }
    log("begin end");
#ifdef _MSC_VER
    closesocket(confd);
    WSACleanup();
#else
    close(confd);
#endif
    return 0;
}

int main()
{
    //std::vector<char> data;
    //collect_lidar_data(data);
    //ofstream ouF;
    //ouF.open("./package.dat", std::ofstream::binary);
    //ouF.write(&data[0], data.size());
    //ouF.close();
    //return 0;
     
    main_test();
}