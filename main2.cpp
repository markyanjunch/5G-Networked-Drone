#include <iostream>
#include <cstdio>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "VE450.hpp"
#include <errno.h>
#include <pthread.h>
#include<time.h>
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
#define MAXLINE 101
pthread_mutex_t g_mutext;
  // get gps information
Telemetry::GPSInfo gpsData;
  
struct s_info{
  struct sockaddr_in client_add;
  int client_fd;
  Vehicle* vehicle;
  int thno;
};

void* do_work(void* arg){
  int n,i;
  int reno=0;
	struct s_info *ts = (struct s_info*)arg;
	char buf[MAXLINE];//在线程自己的用户空间栈开辟的,该线程运行结束的时候,主控线程就不能操作这块内存了
	char str[INET_ADDRSTRLEN];//INET_ADDRSTRLEN 是宏16个字节
  memset(buf,0,sizeof(buf));
  char info[50]="thread access established, thread no. \0";
  char thnum[5];
  thnum[5]='\0';
  char la_origin[32];
  char lo_origin[32];
  char la_trivial[7];
  char lo_trivial[7];
  char la[10];
  char lo[11];
  la_origin[32]='\0';
  lo_origin[32]='\0';
  la_trivial[7]='\0';
  lo_trivial[7]='\0';
  la[10]='\0';
  lo[11]='\0';
  char dot[]={"."};
  char dest[21];
  dest[21]='\0';
  char space[]={" "};
  sprintf(thnum,"%d", ts->thno);
  //在创建线程前设置线程创建属性,设为分离态,效率高
  pthread_detach(pthread_self());
  
  send(ts->client_fd,strcat(info,thnum),55,0);
 std::cout<<info<<std::endl;
  /*
  recv(ts->client_fd,buf,MAXLINE,0);
  if(strncmp(buf,"commandmode",11)==0){
    send(ts->client_fd,"Enter command mode",20,0);
    */
  while(1){
      memset(buf,0,sizeof(buf));
      n=recv(ts->client_fd,buf,MAXLINE,0);
      if(n<=0){
        std::cout<<"disconnected, reconnecting"<<std::endl;
        sleep(4);
        reno++;
        if(reno>=5)break;
      }
      else{
      buf[MAXLINE]=='\0';
      printf("receive bytes = %d data=%s\n",n,buf);
      fflush(stdout);
      send(ts->client_fd,buf,n,0);
      if(strncmp(buf,"gps",3)==0){
        srand(1);
        send(ts->client_fd,"enter gps mode",15,MSG_NOSIGNAL);
        while(1){
          sleep(1);
          /*
          sprintf(la_origin,"%d",gpsData.latitude);
          sprintf(lo_origin,"%d",gpsData.longitude);
          printf("gps info %d %d",gpsData.latitude,gpsData.longitude);
          fflush(stdout);
          strncpy(la,la_origin,2);
          strncpy(lo,lo_origin,3);
          strncat(la,dot,1);
          strncat(lo,dot,1);
          strncpy(dest,la,9);
          strncat(dest,space,1);
          strncat(dest,lo,10);
          for(int u=1;u<=6;u++){
            la_trivial[u]=la_origin[u+2];
            lo_trivial[u]=lo_origin[u+3];
          }
          strncat(la,la_trivial,6);
          strncat(lo,lo_trivial,6);
          */
         int randnum=rand()%100;
          if(send(ts->client_fd,(const char*)&randnum,4,MSG_NOSIGNAL)<=0){
            printf("thread %d is closing...",ts->thno);
            fflush(stdout);
            pthread_exit(NULL);
          }
        }
      }
      if(strncmp(buf,"Take Off",8)==0){
        monitoredTakeoff(ts->vehicle);
      }
      else if(strncmp(buf,"land",4)==0){
        monitoredLanding(ts->vehicle);
      }
      else if(strncmp(buf,"forward",7)==0){
        moveByPositionOffset(ts->vehicle,5,0,0,0);
      }
      else if(strncmp(buf,"quit",4)==0)
          break;
     }
     memset(buf,0,sizeof(buf));
  }
  /*
  }
  
  else if(strncmp(buf,"recvmode",8)==0){
    send(ts->client_fd,"Enter recv mode",15,0);
      gpsData = ts->vehicle->broadcast->getGPSInfo();
  }
  */
pthread_exit(NULL);
}

int main(int argc, char** argv) {
  // Initialize server variables
  int sock_fd;
  struct sockaddr_in serv_addr;
  int client_fd;
  struct sockaddr_in client_add;
  socklen_t len;
  int n;
  int reuse=1;
  struct timeval tv;
  // Initialize onboard variables
  int functionTimeout = 1;
  int battery;
  struct s_info ts[3000];
  int m=0;
  pthread_t tid;
  char threadhandle[10];
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
  /*

  std::cout << "latitude=" << gpsData.latitude << std::endl;
  std::cout << "longtitude=" << gpsData.longitude << std::endl;
  */
  // open server
  // 创建socket连接
  sock_fd=socket(AF_INET,SOCK_STREAM,0);
  if(sock_fd==-1)
  {
      perror("create socket error!.");
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

  while(1){
  len=sizeof(client_add);
  client_fd=accept(sock_fd,(struct sockaddr*)&client_add,&len);
  /*
  std::cout<<strerror(errno)<<std::endl;
  */
  ts[m].client_add=client_add;
  ts[m].client_fd=client_fd;
  ts[m].vehicle=vehicle;
  ts[m].thno=m;
  pthread_create(&tid, NULL, do_work, (void*)&ts[m]);//把accept得到的客户端信息传给线程，让线程去和客户端进行数据的收发
  m++;
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
/*
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
     
    
  }
  
     
  std::cout << "startGlobalPositionBroadcast(vehicle) = " << startGlobalPositionBroadcast(vehicle) << std::endl;


  
  std::cout << "Successfully quit and communication closed." << std::endl;
  TakeoffAnyway(vehicle);
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
