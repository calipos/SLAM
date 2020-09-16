#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#ifdef _MSC_VER
#include<stdio.h>
#include<tchar.h>
#include <iostream>
#include <WinSock2.h>
#include <Windows.h>

#pragma comment(lib, "ws2_32.lib")
#define	UDP_BUF_SIZE	1210 
#define UDPPORT 2368
//int main_vitualLidar()
int main()
{
	std::vector<char> data(UDP_BUF_SIZE * 10000);
	std::ifstream inF;
	inF.open("D:/SLAM/package.dat", std::ifstream::binary);
	inF.read(&data[0], UDP_BUF_SIZE * 10000);
	inF.close();

    WSAData wsd;       
    SOCKET soSend;     
    int nRet = 0;
    int dwSendSize = 0;
    int iRet = 0;
    if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0)
    {
        iRet = WSAGetLastError();
        printf("WSAStartup failed !\n");
        return -1;
    }
    //soSend = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    soSend = socket(AF_INET, SOCK_DGRAM, 0);
    if (soSend == SOCKET_ERROR) {
        std::cout << "socket Error = " << WSAGetLastError() << std::endl;
        return 1;
    }
    char on = 1;
    int ret = setsockopt(soSend, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
    if (ret < 0)
    {
        perror("setsockopt fail\n");
        return -1;
    }
     
    SOCKADDR_IN siLocal{0};
    siLocal.sin_family = AF_INET;
    siLocal.sin_port = htons(UDPPORT);
    siLocal.sin_addr.s_addr = inet_addr("192.168.0.136");

    for (int i = 0; ; i++) 
    {
        int thisIdx = i % 10000;
        int pos = thisIdx * UDP_BUF_SIZE;
        nRet = sendto(soSend, &data[pos], 1206, 0, (SOCKADDR*)&siLocal, sizeof(siLocal));
        if (nRet == SOCKET_ERROR) 
        {
            std::cout << "sendto Error " << WSAGetLastError() << std::endl;
            break;
        }
        else
        {
            std::cout << thisIdx <<" : "<< nRet << std::endl;
        }
    }
    closesocket(soSend);
    WSACleanup();

    return 0;

}

#endif