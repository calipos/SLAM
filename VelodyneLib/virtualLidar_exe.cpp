#include <iostream>
#include <fstream>
#include <string>
#include <vector>
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

#pragma comment(lib, "ws2_32.lib")
#define	BUF_SIZE	1210 
#define UDPPORT 2368
int main(int argc,char** argv)
{
    if(argc!=2)
    {
        printf("usage : exe  package_path !\n");
        return -1;
    }
	std::vector<char> data(BUF_SIZE * 10000);
	std::ifstream inF;
	inF.open("D:/SLAM/package.dat", std::ifstream::binary);
	inF.read(&data[0], BUF_SIZE * 10000);
	inF.close();

      
        
    int nRet = 0;
    int dwSendSize = 0;
#ifdef _MSC_VER
    SOCKET soSend; 
    WSAData wsd; 
    int iRet = 0;
    if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0)
    {
        iRet = WSAGetLastError();
        printf("WSAStartup failed !\n");
        return -1;
    }
#else
    int soSend; 
#endif
    soSend = socket(AF_INET, SOCK_DGRAM, 0);
    if (soSend <0) {
        std::cout << "socket Error " << std::endl;
        return 1;
    }
    char on = 1;
    int ret = setsockopt(soSend, SOL_SOCKET, SO_BROADCAST, &on, sizeof(on));
//    if (ret < 0)
//    {
//        perror("setsockopt fail\n");
//        return -1;
//    }
 
    struct sockaddr_in siLocal{0};
    siLocal.sin_family = AF_INET;
    siLocal.sin_port = htons(UDPPORT);
    siLocal.sin_addr.s_addr = inet_addr("192.168.0.136");

    for (int i = 0; ; i++) 
    {
        int thisIdx = i % 10000;
        int pos = thisIdx * BUF_SIZE;  
#ifdef _MSC_VER
	nRet = sendto(soSend, &data[pos], 1206, 0, (sockaddr_in*)&siLocal, sizeof(siLocal));
#else
	nRet = sendto(soSend, &data[pos], 1206, 0, (struct sockaddr *)(&siLocal), sizeof(struct sockaddr));
#endif
        if (nRet <0) 
        {
            std::cout << "sendto Error "   << std::endl;
            break;
        }
        else
        {
            std::cout << thisIdx <<" : "<< nRet << std::endl;
        }
    }
#ifdef _MSC_VER
        closesocket(soSend);
        WSACleanup();
#else
        close(soSend);
#endif

    return 0;

}

