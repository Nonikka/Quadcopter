#include<stdio.h>  
#include<stdlib.h>  
#include<string.h>  
#include<errno.h>  
#include<sys/types.h>  
#include<sys/socket.h>  
#include<netinet/in.h>  
#define DEFAULT_PORT 8099 
#define MAXLINE 4096  
int main(int argc, char** argv)  
{  
    int    socket_fd, connect_fd;  
    struct sockaddr_in     servaddr;  
    char    buff[4096],axis1[5],axis2[5],axis3[5],axis4[5];  
    int     n;  
    int _axis[6];
    //初始化Socket  
    if( (socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1 ){  
    printf("create socket error: %s(errno: %d)\n",strerror(errno),errno);  
    exit(0);  
    }  
    //初始化  
    memset(&servaddr, 0, sizeof(servaddr));  
    servaddr.sin_family = AF_INET;  
    servaddr.sin_addr.s_addr = htonl(INADDR_ANY);//IP地址设置成INADDR_ANY,让系统自动获取本机的IP地址。  
    servaddr.sin_port = htons(DEFAULT_PORT);//设置的端口为DEFAULT_PORT  
  
    //将本地地址绑定到所创建的套接字上  
    if( bind(socket_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) == -1){  
    printf("bind socket error: %s(errno: %d)\n",strerror(errno),errno);  
    exit(0);  
    }  
    //开始监听是否有客户端连接  
    if( listen(socket_fd, 10) == -1){  
    printf("listen socket error: %s(errno: %d)\n",strerror(errno),errno);  
    exit(0);  
    }  
    printf("======waiting for client's request======\n");  
    while(1){  
//阻塞直到有客户端连接，不然多浪费CPU资源。  
        if( (connect_fd = accept(socket_fd, (struct sockaddr*)NULL, NULL)) == -1){  
        printf("accept socket error: %s(errno: %d)",strerror(errno),errno);  
        continue;  
    }  
//接受客户端传过来的数据  
    n = recv(connect_fd, buff, MAXLINE, 0);  
//向客户端发送回应数据  
    if(!fork()){ /*子进程*/  
        if(send(connect_fd, "Hello,you are connected!\n", 26,0) == -1)  
        perror("send error");  
        close(connect_fd);  
        exit(0);  
    }  
    buff[n] = '\0';  
    axis1[0] = buff[6];
    axis1[1] = buff[7];
    axis1[2] = buff[8];
    axis1[3] = buff[9];
    axis1[4] = '\0';
    _axis[0] =  atoi(axis1);
    
    axis2[0] = buff[11];
    axis2[1] = buff[12];
    axis2[2] = buff[13];
    axis2[3] = buff[14];
    axis2[4] = '\0';
    _axis[1] =  atoi(axis2);//前后控制
    
    axis3[0] = buff[16];
    axis3[1] = buff[17];
    axis3[2] = buff[18];
    axis3[3] = buff[19];
    axis3[4] = '\0';//不加会导致转换错误
    _axis[2] =  atoi(axis3);//左右控制
    /*
    _axis[1] =  atoi(buff[10])*100 + atoi(buff[11])*10 + atoi(buff[12])*1;
    _axis[2] =  atoi(buff[14])*100 + atoi(buff[15])*10 + atoi(buff[16])*1;
    _axis[3] =  atoi(buff[18])*100 + atoi(buff[19])*10 + atoi(buff[20])*1;
    _axis[4] =  atoi(buff[22])*100 + atoi(buff[23])*10 + atoi(buff[24])*1;
    _axis[5] =  atoi(buff[26])*100 + atoi(buff[27])*10 + atoi(buff[28])*1;
    printf("recv msg from client: %d %d %d %d %d %d\n",_axis[0],_axis[1],_axis[2],_axis[3],_axis[4],_axis[5]);*/
    printf("\n\nrecv msg from client: %s %d %d %d\n\n",buff,_axis[0],_axis[1],_axis[2]);
    close(connect_fd);  
    }  
    close(socket_fd);  
}  
