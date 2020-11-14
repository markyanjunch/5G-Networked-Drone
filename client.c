#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>

#define EHCO_PORT 6001


int main()
{
    int sock_fd;
    struct sockaddr_in serv_addr;
    char buff[50];
    char tmp_buff[100];
    int n,i;

    //创建socket
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
    serv_addr.sin_addr.s_addr=inet_addr("114.116.238.69");    //设置服务器IP地址
    bzero(&(serv_addr.sin_zero),8);

    //连接服务端
    if(-1==connect(sock_fd,(struct sockaddr*)&serv_addr,sizeof(serv_addr)))
    {
         perror("connect fail!");
         close(sock_fd);
         return 0;
    }
    printf("Success connect to server!\n");

    //发送并接收缓冲的数据
    while(1)
    {
        gets(buff);
        send(sock_fd,buff,100,0);
        n=recv(sock_fd,tmp_buff,100,0);
        tmp_buff[n]='\0';
        printf("data send:%s receive: %s\n",buff,tmp_buff);
        if(0==strncmp(tmp_buff,"quit",4))
        {
            break;
        }
    }
    close(sock_fd);
    return 0;
}
