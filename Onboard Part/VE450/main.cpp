#include <iostream>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "VE450.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;


int main(int argc, char** argv) {
  // Initialize server variables
  int sock_fd;
  struct sockaddr_in serv_addr;
  int client_fd;
  struct sockaddr_in client_add;
  char buff[101];
  char command[101];
  socklen_t len;
  int n;

  // Initialize onboard variables
  int functionTimeout = 1;

  // Setup OSDK.
  std::cout << "VE450 program starts!\n";
  LinuxSetup linuxEnvironment(argc, argv);
  Vehicle* vehicle = linuxEnvironment.getVehicle();
  if (vehicle == NULL) {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }
  

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);
  
  // get gps information
  Telemetry::GPSInfo gpsData;
  gpsData = vehicle->broadcast->getGPSInfo();
  std::cout << "latitude=" << gpsData.latitude << std::endl;
  std::cout << "longtitude=" << gpsData.longitude << std::endl;
  
  // open server
  // 创建socket连接
  sock_fd=socket(AF_INET,SOCK_STREAM,0);
  if(sock_fd==-1)
  {
      perror("create socket error!");
      return 0;
  }
  else
  {
      printf("Success to create socket %d\n",sock_fd);
  }

  //设置server地址结构
  bzero(&serv_addr,sizeof(serv_addr));    //初始化结构占用内存
  serv_addr.sin_family=AF_INET;    //设置传输层类型IPv4
  serv_addr.sin_port=htons(EHCO_PORT);    //设置端口号
  serv_addr.sin_addr.s_addr=htons(INADDR_ANY);    //设置服务器IP地址
  bzero(&(serv_addr.sin_zero),8);

  //绑定端口
  if(bind(sock_fd,(struct sockaddr*)&serv_addr,sizeof(serv_addr))!=0)
  {
       printf("bind address fail %d\n",errno);
       close(sock_fd);
       return 0;
  }
  else
  {
       printf("Success to bind address!\n");
  }

  //监听端口
  if(listen(sock_fd,MAX_CLIENT_NUM!=0))
  {
      perror("listen socket error!\n");
      close(sock_fd);
      return 0;
  }
  else
  {
      printf("Success to listen\n");
  }

  //创建连接客户端的对应套接字
  len=sizeof(client_add);
  client_fd=accept(sock_fd,(struct sockaddr*)&client_add,&len);
  if(client_fd<=0)
  {
      perror("accept error!");
      close(sock_fd);
      return 0;
  }
  //接收客户端发来的数据
  while(1)
  {
    if((n=recv(client_fd,buff,100,0))>0){
      buff[n]=='\0';
      command[n]='\0';
      strncpy(command,buff,n);
      printf("receive byetes = %d data=%s\n",n,command);
      fflush(stdout);
      send(client_fd,command,n,0);
      if(strncmp(command,"Take Off",8)==0){
        TakeoffAnyway(vehicle);
      }
      else if(strncmp(command,"land",4)==0){
        LandingAnyway(vehicle);
      }
      else if(strncmp(command,"forward",7)==0){
          moveByPositionOffset(vehicle,5,0,0,0);
        }
      else if(strncmp(command,"quit",4)==0)
          break;
    }
  }
    

  std::cout << "Automatic takeoff and landing test for VE450 starts..." << std::endl;
  /*TakeoffAnyway(vehicle);
  moveByPositionOffset(vehicle,0,0,0,0);
  moveByPositionOffset(vehicle,0,0,0,0);
  moveByPositionOffset(vehicle,0,0,0,0);
  LandingAnyway(vehicle);
  std::cout
          << "Test over."
          <<std::endl;

  // Display interactive prompt
  /*std::cout
      << "| Available commands:                                            |"
      << std::endl;
  std::cout
      << "| [a] Takeoff + Landing                                |"
      << std::endl;
  std::cout
      << "| [b] Takeoff + Position Control + Landing             |"
      << std::endl;

  char inputChar;
  std::cin >> inputChar;

  switch (inputChar) {
    case 'a':
      monitoredTakeoff(vehicle);
      monitoredLanding(vehicle);
      break;
    case 'b':
      monitoredTakeoff(vehicle);
      moveByPositionOffset(vehicle, 0, 6, 6, 30);
      moveByPositionOffset(vehicle, 6, 0, -3, -30);
      moveByPositionOffset(vehicle, -6, -6, 0, 0);
      monitoredLanding(vehicle);
      break;

    default:
      break;
  }*/

  // close server
  close(client_fd);
  close(sock_fd);
  return 0;
} 
