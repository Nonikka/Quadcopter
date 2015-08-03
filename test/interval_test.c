#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <errno.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <softPwm.h>

/*int getitimer(int which, struct itimerval *value);
int setitimer(int which, struct itimerval*newvalue, struct itimerval* oldvalue);
struct timeval
{
long tv_sec; //秒
long tv_usec; //微秒
};
struct itimerval
{
struct timeval it_interval; //时间间隔
struct timeval it_value;   //当前时间计数
};*/

static char msg[] = "time is running out";
static int len;
// 向标准错误输出信息，告诉用户时间到了
void prompt_info(int signo)
{
write(STDERR_FILENO, msg, len);
}
// 建立信号处理机制
void init_sigaction(void)
{
struct sigaction tact;
/*信号到了要执行的任务处理函数为prompt_info*/
tact.sa_handler = prompt_info;
tact.sa_flags = 0;
/*初始化信号集*/
sigemptyset(&tact.sa_mask);
/*建立信号处理机制*/
sigaction(SIGALRM, &tact, NULL);
}
void init_time()
{
struct itimerval value;
/*设定执行任务的时间间隔为2秒0微秒*/
value.it_value.tv_sec = 2;
value.it_value.tv_usec = 0;
/*设定初始时间计数也为2秒0微秒*/
value.it_interval = value.it_value;
/*设置计时器ITIMER_REAL*/
setitimer(ITIMER_REAL, &value, NULL);
}
int main(void)
{
len = strlen(msg);
init_sigaction();
init_time();
while(1)
    {
    write(STDERR_FILENO, "123", 3);
    delay(1000);
    }
exit(0);
}