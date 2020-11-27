#include <iostream>
#include <cstdio>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "VE450.hpp"
#include <errno.h>

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
  int reuse=1;
  struct timeval tv;
  // Initialize onboard variables
  int functionTimeout = 1;
  int battery;
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
  
  //reuseaddr
  int reuseaddr=setsockopt(sock_fd,SOL_SOCKET,SO_REUSEADDR,&reuse,sizeof(tv));
  
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
  len=sizeof(client_add);
  client_fd=accept(sock_fd,(struct sockaddr*)&client_add,&len);
  

  if(client_fd<=0)
  {
      perror("accept error!");
      close(sock_fd);
      return 0;
  }
  else
  {
      send(client_fd,"access established.\n",20,0);
  }
  //接收客户端发来的数据
  /*
  float64_t latitude = 106.0;
  float64_t longitude = 32.0;
  WaypointMission wayPoint(vehicle);
  WayPointInitSettings WayPointDesti = {(uint8_t)1, (float32_t)5, (float32_t)5, (uint8_t)2, (uint8_t)1, 
                                       (uint8_t)0, (uint8_t)0, (uint8_t)1, (uint8_t)0, (float64_t)latitude, 
                                       (float64_t)longitude, (float32_t)0, (uint8_t)0};
  wayPoint.init(&WayPointDesti);
  wayPoint.start();
  */

  while(1)
  {
      
            // receive battery information
            
      Telemetry::Battery batteryInfo = vehicle->broadcast->getBatteryInfo();
      battery=(int)batteryInfo.percentage;
      char da[100]="ba";
      da[100]='\0';

      char batstr[3];
      sprintf(batstr,"%d", battery);
      send(client_fd,strcat(da,batstr),100,0);
      n=recv(client_fd,buff,100,0);
      if(n<=0){
        
        std::cout<<"disconnected, reconnecting"<<std::endl;
        fflush(stdout);
        sleep(4);
        client_fd=accept(sock_fd,(struct sockaddr*)&client_add,&len);
        send(client_fd,"reconnected",15,0);
      }
      else{


      buff[n]=='\0';
      command[n]='\0';
      strncpy(command,buff,n);
      printf("receive bytes = %d data=%s\n",n,command);
      fflush(stdout);
      send(client_fd,command,n,0);
      if(strncmp(command,"Take Off",8)==0){
        monitoredTakeoff(vehicle);
      }
      else if(strncmp(command,"land",4)==0){
        monitoredLanding(vehicle);
      }
      else if(strncmp(command,"forward",7)==0){
        moveByPositionOffset(vehicle,5,0,0,0);
      }
      else if(strncmp(command,"quit",4)==0)
          break;
      }
    
  }
  
     
  std::cout << "startGlobalPositionBroadcast(vehicle) = " << startGlobalPositionBroadcast(vehicle) << std::endl;


  
  std::cout << "Successfully quit and communication closed." << std::endl;
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
