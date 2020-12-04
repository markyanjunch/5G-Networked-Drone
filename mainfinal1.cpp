#include <iostream>
#include <cstdio>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "VE450.hpp"
#include <errno.h>
#include <pthread.h>
#include  <time.h>
#include <netinet/tcp.h>
#include <math.h>
#include <cmath>
using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;
#define MAXLINE 101
#define PI 3.14159265358979323846264338327950288
pthread_mutex_t g_mutext;
  // get gps information
Telemetry::GPSInfo gpsData;
Telemetry::Battery batteryInfo;
  
struct s_info{
  struct sockaddr_in client_add;
  int client_fd;
  Vehicle* vehicle;
  int thno;
};

void* do_work(void* arg){
    //在创建线程前设置线程创建属性,设为分离态,效率高
  pthread_detach(pthread_self());
  int n,i;
  int on=1;
  int reno=0;
	struct s_info *ts = (struct s_info*)arg;
	char buf[MAXLINE];//在线程自己的用户空间栈开辟的,该线程运行结束的时候,主控线程就不能操作这块内存了
	char str[INET_ADDRSTRLEN];//INET_ADDRSTRLEN 是宏16个字节
  char info[17]="Thread No.\0";
  char thnum[5];
  thnum[5]='\0';
  char la[20];
  char lo[20];
  la[20]='\0';
  lo[20]='\0';
  char dymsg[39];
  dymsg[39]='\0';
  int battery;
  char ba[2];
  ba[2]='\0';
  sprintf(thnum,"%d", ts->thno);
  printf("%s\n",strcat(info,thnum));
  //send(ts->client_fd,info,17,0);
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
        if(reno>=3)break;
      }
      else{
      buf[MAXLINE]=='\0';
      printf("receive bytes = %d data=%s\n",n,buf);
      fflush(stdout);
      
      if(strncmp(buf,"gps",3)==0){
        setsockopt(ts->client_fd,IPPROTO_TCP,TCP_NODELAY,&on,sizeof(on));
        //send(ts->client_fd,"enter gps mode",15,MSG_NOSIGNAL);
        while(1){
          memset(buf,0,sizeof(buf));
          memset(dymsg,0,sizeof(dymsg));
          gpsData = ts->vehicle->broadcast->getGPSInfo();
          gpsData.latitude = gpsData.latitude-18426;
          gpsData.longitude = gpsData.longitude+47135;
          batteryInfo = ts->vehicle->broadcast->getBatteryInfo();
          battery=(int)batteryInfo.percentage;
                 
          sleep(1);
          sprintf(la,"%d",gpsData.latitude);
          sprintf(lo,"%d",gpsData.longitude);
          strcat(dymsg,la);
          strcat(dymsg,lo);
          
          if ((battery>=10)&&(battery<100))
          {
           sprintf(ba,"%d",battery);
            strcat(dymsg,ba);
          }
          /*
          else if(battery<10)
          {
            ba[1]=0;
            ba[2]=0;
            ba[3]=(char)battery;
            strcat(dymsg,ba);
          }
          else{
            strcat(dymsg,"100");
          }
          */
          if(send(ts->client_fd,dymsg,39,MSG_NOSIGNAL)<=0){
            printf("thread %d is closing...",ts->thno);
            fflush(stdout);
            pthread_exit(NULL);
            sleep(0.5);
          }
        }
      }
      send(ts->client_fd,buf,n,0);
      if(strncmp(buf,"Take Off",8)==0){
        monitoredTakeoff(ts->vehicle);
      }
      else if(strncmp(buf,"land",4)==0){
        monitoredLanding(ts->vehicle);
      }
      else if(strncmp(buf,"forward",7)==0){
        moveByPositionOffset(ts->vehicle,5,0,0,0);
      }
      else if(strncmp(buf,"backward",8)==0){
        moveByPositionOffset(ts->vehicle,-5,0,0,0);
      }
      else if(strncmp(buf,"left",4)==0){
        moveByPositionOffset(ts->vehicle,0,-5,0,0);
      }
      else if(strncmp(buf,"right",5)==0){
        moveByPositionOffset(ts->vehicle,0,5,0,0);
      }
      else if(strncmp(buf,"upper",5)==0){
        moveByPositionOffset(ts->vehicle,0,0,0.5,0);
      }
      else if(strncmp(buf,"lower",5)==0){
        moveByPositionOffset(ts->vehicle,0,0,0.5,0);
      }
      else if(strncmp(buf,"hotpoint",8)==0){
        int     hotptInitRadius;
        int     responseTimeout = 1;
        hotptInitRadius = 10;
        runHotpointMission(ts->vehicle, hotptInitRadius, responseTimeout);
      }
      else if(strncmp(buf,"waypoint",8)==0){
        
        Telemetry::Vector3f deltaNED;
        
        // Global position retrieved via broadcast
        Telemetry::GlobalPosition currentGPS;
        Telemetry::GlobalPosition targetGPS;
        //currentGPS = ts->vehicle->broadcast->getGlobalPosition();
        //gpsData = ts->vehicle->broadcast->getGPSInfo();

        currentGPS.latitude = static_cast<double>(gpsData.latitude)/static_cast<double>(10000000.0000000)*DEG2RAD;
        currentGPS.longitude = static_cast<double>(gpsData.longitude)/static_cast<double>(10000000.0000000)*DEG2RAD;
        std::cout << "gpsData.latitude=" << gpsData.latitude << std::endl;
        
        printf("currentGPS.latitude= %.7lf\n", currentGPS.latitude);
        printf("currentGPS.longitude= %.7lf\n", currentGPS.longitude);
        std::string buf_str = buf;
        std::string buf_str_la = buf_str.substr(8,10);
        std::string buf_str_lo = buf_str.substr(18,11);
        std::cout << "buf_str=" << buf_str<< std::endl;
        std::cout << "buf_str_la=" << buf_str_la<< std::endl;
        std::cout << "buf_str_lo=" << buf_str_lo<< std::endl;
        targetGPS  = {(double)atof(buf_str_la.c_str())*DEG2RAD, (double)atof(buf_str_lo.c_str())*DEG2RAD,currentGPS.altitude,currentGPS.height,
        currentGPS.health};
        //std::cout<<"current:"<<currentGPS.latitude<<" "<<currentGPS.longitude<<std::endl;
        std::cout<<"target:"<<targetGPS.latitude<<" "<<targetGPS.longitude<<std::endl;
        localOffsetFromGpsOffset(ts->vehicle, deltaNED,
                                static_cast<void*>(&targetGPS),
                                static_cast<void*>(&currentGPS));
        std::cout<<"ready to move"<<std::endl;
        std::cout<<"NED:"<<deltaNED.x<<" "<<deltaNED.y<<std::endl;
        moveByPositionOffset(ts->vehicle,-deltaNED.y,deltaNED.x,0,0);
      }
      else if(strncmp(buf,"quit",4)==0)
          break;
     }
    
  }
  /*
  }
  
  else if(strncmp(buf,"recvmode",8)==0){
    send(ts->client_fd,"Enter recv mode",15,0);
      
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
