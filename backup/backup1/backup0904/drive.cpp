/***********************************************************************************************
****平台：A3352+LINUX
****功能：服务器交互程序。
************************************************************************************************/
#include <stdio.h>
#include <cstdint>
#include <string>
#include <string.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <malloc.h>
#include <algorithm>
#include <vector>

#include "src/include/Lidar.h"
#include "src/include/Serial.h"

#define UART_DEV_UPPER "/dev/ttyO4" //等同于char *uart4 = "/dev/ttyS4";用uart4代替
#define DEV_PATH45 "/sys/class/gpio/gpio45"
#define DEV_PATH46 "/sys/class/gpio/gpio46"

//通用:
std::string SSN, VNB, ENB;
bool wifiopen = false;

char Voice_begin[2] = {0xFD, 0x00};
char Voicelength[1];
char Voice_end[1] = {0x0a};

//定义帧头
char start[3] = {0x0a, 'A', 'B'};
//定义结束符和中间分隔符
char end = 0x0d, mid = ':',comma = ',';
//------------------------------------------------------------------------------------------------
//*************************************报警音编码数组*********************************************
//"车辆预热"           
char YURE[] = {0xFD,0x00,0x0A,0x01,0x01,0xB3,0xB5,0xC1,0xBE,0xD4,0xA4,0xC8,0xC8,'\0'}; 

//"您已超速"
char CHAOSU[] = {0xFD,0x00,0x0A,0x01,0x01,0xC4,0xFA,0xD2,0xD1,0xB3,0xAC,0xCB,0xD9,'\0'}; 

//前方有停止线，请减速
char STOP[] = {0xFD,0x00,0x0E,0x01,0x01,0xC7,0xB0,0xB7,0xBD,0xD3,0xD0,0xCD,0xA3,0xD6,0xB9,0xCF,0xDF,'\0'};

//"车辆已驶入违规区域"                                         
char JINRU[] = {0xFD,0x00,0x12,0x01,0x01,0xC4,0xFA,0xD2,0xD1,0xBD,0xF8,0xC8,0xEB,0xCE,0xA5,0xB9,0xE6,0xC7,0xF8,0xD3,0xF2,'\0'}; 

//"您已越界"
char YUEJIE[] = {0xFD,0x00,0x0A,0x01,0x01,0xC4,0xFA,0xD2,0xD1,0xD4,0xBD,0xBD,0xE7,'\0'};    

//"车辆已驶出限制区"
char SHICHU[] = {0xFD,0x00,0x12,0x01,0x01,0xB3,0xB5,0xC1,0xBE,0xD2,0xD1,0xCA,0xBB,0xB3,0xF6,0xCF,0xDE,0xD6,0xC6,0xC7,0xF8,'\0'};

//违规进入电子围栏区域
char WEILAN[] = {0xFD,0x00,0x16,0x01,0x01,0xCE,0xA5,0xB9,0xE6,0xBD,0xF8,0xC8,0xEB,0xB5,0xE7,0xD7,0xD3,0xCE,0xA7,0xC0,0xB8,0xC7,0xF8,0xD3,0xF2,'\0'};

//"驶出电子围栏"
char SHICHUWEILAN[] = {0xFD,0x00,0x0E,0x01,0x01,0xCA,0xBB,0xB3,0xF6,0xB5,0xE7,0xD7,0xD3,0xCE,0xA7,0xC0,0xB8,'\0'};

//"您已疲劳驾驶"                               
char PILAO[] = {0xFD,0x00,0x0E,0x01,0x01,0xC4,0xFA,0xD2,0xD1,0xC6,0xA3,0xC0,0xCD,0xBC,0xDD,0xCA,0xBB,'\0'};    

//"超时停车"    
char CHAOSHI[] = {0xFD,0x00,0x0A,0x01,0x01,0xB3,0xAC,0xCA,0xB1,0xCD,0xA3,0xB3,0xB5,'\0'};                              

char speedcompareisready=0;
//------------------------------------------------------------------------------------------------
//*******************************************OBD发送ID********************************************
int OBDCnt = 0;
static char OBDPayloadSerial[10][320];
static char OBDpayload[300], OBDpayload_h2[40];
//head1[2] = {'#','#'};//采用这种形式
char head1[2] = {0x23, 0x23};                   //##
char head2[4] = {0x56, 0x44, 0x41, 0x54};       //VDAT 消息名称
char head3[3] = {0x53, 0x53, 0x4e};             //SSN 编号
char head4[5] = {0x56, 0x4e, 0x42, 0x42, 0x44}; //VNBBD 车号
char head5[3] = {0x45, 0x4e, 0x42};             //ENB  员工号
char head6[4] = {0x54, 0x59, 0x50, 0x31};       //TYP1 车型
char head7[3] = {0x4d, 0x49, 0x4c};             //MIL 实际累计里程
char head8[3] = {0x45, 0x4e, 0x47};             //ENG 发动机运行时间累计
char head9[3] = {0x54, 0x4d, 0x4c};             //TML 里程表总里程
char head10[3] = {0x53, 0x4d, 0x4c};            //SML 里程表小里程
char head11[3] = {0x52, 0x4d, 0x4c};            //RML 续航里程
char head12[3] = {0x42, 0x41, 0x54};            //BAT 电池电压
char head13[3] = {0x56, 0x45, 0x53};            //VES 车速
char head14[3] = {0x52, 0x4f, 0x53};            //ROS 转速
char head15[3] = {0x57, 0x41, 0x54};            //WAT 水温
char head16[3] = {0x4f, 0x49, 0x4c};            //OIL 耗油量
char head17[3] = {0x4f, 0x49, 0x4b};            //OIK 百公里瞬时耗油
char head18[3] = {0x4f, 0x49, 0x48};            //OIH 小时瞬时油耗
char head19[3] = {0x45, 0x41, 0x54};            //EAT 发动机进气温度
char head20[3] = {0x4d, 0x41, 0x50};            //MAP 进气歧管绝对压力
char head21[3] = {0x41, 0x46, 0x4c};            //AFL 空气流量
char head22[3] = {0x46, 0x55, 0x4c};            //FUL 门锁状态
char head23[3] = {0x54, 0x50, 0x50};            //TPP 动力踏板油门位置
char head24[3] = {0x46, 0x42, 0x53};            //FBS 脚刹状态
char head25[3] = {0x48, 0x42, 0x53};            //HBS 手刹状态
char head26[3] = {0x47, 0x45, 0x53};            //GES 档位状态
char head27[3] = {0x41, 0x43, 0x43};            //ACC acc信号
char head28[3] = {0x41, 0x43, 0x53};            //ACS 空调开关
char head29[3] = {0x54, 0x4f, 0x52};            //TOR 扭矩
char head30[3] = {0x57, 0x4c, 0x49};            //WLI 示宽灯信号
char head31[3] = {0x48, 0x4c, 0x49};            //HLI 远光灯信号
char head32[3] = {0x4c, 0x4c, 0x49};            //LLI 近光灯信号
char head33[3] = {0x45, 0x4e, 0x4c};            //ENL 发动机负荷
char head34[3] = {0x4f, 0x49, 0x50};            //OIP 机油压力
char head35[3] = {0x4f, 0x49, 0x4d};            //OIM 机油温度
char head36[3] = {0x46, 0x55, 0x45};            //FUE 喷油量
char head37[3] = {0x46, 0x55, 0x54};            //FUT 燃油温度
char head38[3] = {0x41, 0x50, 0x52};            //APR 大气压力
char head39[3] = {0x50, 0x41, 0x54};            //PAT 增压空气温度
char head40[3] = {0x47, 0x53, 0x43};            //GSC 非OBD里程采集器状态码
char head41[4] = {0x58, 0x58, 0x40, 0x40};      //XX@@




//------------------------------------------------------------------------------------------------
//*******************************************蓝牙数据*********************************************
int BLUECnt = 0;
static char BLUEPayloadSerial[10][750];
static char BLUEpayload[750], BLUEpayload_h2[40];
std::string date = "180000000000";
static char BDdate[15]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,'\0'};
//北斗
int BDCnt = 0;
int SpeedCnt=0;
static char BDPayloadSerial[10][600];
static char SpeedPayloadSerial[10][600];
char BDpayload[600];
char Speedpayload[600];

//其他

std::string BAP = "0.0", TEM = "0.0";
bool ZX_NTgps = false, HW_NTgps = false;
//
char spdlimit[10];
char spdmax[10];
char spdnew[10];
//
float speedlimit,speedmax,speednew;


int SLN = 4;
int DRS = -9999;
int URS = -9999;
int DCR = -9999;
int UCR = -9999;
uint32_t DCNCnt = 1;
uint32_t UCNCnt = 1;
int rssi = 25;
int rssi_ = 24;



std::string ZXipLocal = "0.0.0.0", HWipLocal = "0.0.0.0", WSN = "0000"; //本地网络属性,等待赋值

int HWcilentfd = -1, ZXcilentfd = -1, whichCilent, cilentfd;
bool HWconnected = false, ZXconnected = false;
static char socket_recvbuf[1024];
static char socket_confbuf[512];
static char NTsocket_recvbuf[4096];
bool Socket_is_Send = false; //网络发送中标志位
// pthread_t HWsocketTcp_id, ZXsocketTcp_id, OBDsend_id, BDsend_id, BLUEsend_id, OTHERsend_id, nTrip_id, irq_id;
typedef struct PT
{
    pthread_t HWsocketTcp;
    pthread_t ZXsocketTcp;
    pthread_t OBDsend;
    pthread_t BDsend;
    pthread_t BLUEsend;
    pthread_t OTHERsend;
    pthread_t HWnTrip;
    pthread_t ZXnTrip;
    pthread_t irq;
    pthread_t getInfo;
    pthread_t ZXAT;
} pt;
pt *id = new pt;

std::vector<std::string> config;
CSerial Serial_OBD, Serial_ddp, Serial_Mcu, Serial_Com, mabizhongxing_Com;

/*
    Qbase64
*/
std::string Ntrip_hwQ64, Ntrip_zxQ64;
static char base64_table[255];
void base64_tableinit()
{
    int i, j;
    bzero(base64_table, 255);
    for (j = 0, i = 'A'; i <= 'Z'; i++) /*填base64编码表*/
        base64_table[i] = j++;
    for (i = 'a'; i <= 'z'; i++)
        base64_table[i] = j++;
    for (i = '0'; i <= '9'; i++)
        base64_table[i] = j++;
    base64_table['+'] = j++;
    base64_table['/'] = j++;
    base64_table['='] = j;
}

char *base64_encode(const char *cptr, char **rptr)
{
    char *res;
    int clen, len;
    len = strlen(cptr);
    clen = len / 3;
    res = static_cast<char *>(malloc(clen + 3 * 2 + len));
    if (cptr == NULL || res == NULL)
        return NULL;
    for (*rptr = res; clen--;)
    {
        *res++ = *cptr >> 2 & 0x3f;
        *res = *cptr++ << 4 & 0x30;
        *res++ |= *cptr >> 4;
        *res = (*cptr++ & 0x0f) << 2;
        *res++ |= *cptr >> 6;
        *res++ = *cptr++ & 0x3f;
    }
    if (int32_t i = len % 3)
    {
        if (i == 1)
        {
            *res++ = *cptr >> 2 & 0x3f;
            *res++ = *cptr << 4 & 0x30;
            *res++ = '=';
            *res++ = '=';
        }
        else
        {
            *res++ = *cptr >> 2 & 0x3f;
            *res = *cptr++ << 4 & 0x30;
            *res++ |= *cptr >> 4;
            *res++ = (*cptr & 0x0f) << 2;
            *res++ = '=';
        }
    }
    *res = '=';
    for (res = *rptr; *res != '='; res++)
        *res = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/="[*res];
    rptr[0][strlen(*rptr) - 1] = '\0';
    return *rptr;
}
/*
    线程函数1:华为连接心跳
*/
void *HWsocketTcp(void *ptr)
{
    uint8_t HWconnectCnt = 0;
    int HWconnectfd = -1;
    int recbyte;
    char socketCnt[5];
    unsigned int socket_cnt = 1, HWTimeoutCnt = 0;
    struct sockaddr_in HW_s_add;
   	uint8_t HWConfigtime=1;
    
    //公网服务器端口号
    uint16_t HWportnum;
    //公网服务器IP地址
    std::string HWipaddr;
    //读取配置文件
    std::string adport = config[0];
    //查找:分隔符
    std::string::size_type index = adport.find(":");
    //根据分隔符，分别取IP地址和端口号
    if (index > 0)
    {
        //取IP地址,adport得前index个字符
        HWipaddr = adport.substr(0, index);
        //取端口号，index之后的字符
        HWportnum = abs(atoi(adport.substr(index + 1, adport.length() - 1 - index).c_str()));
        
        //输出华为端口号和IP地址
        std::cout << "HWipaddr: " << HWipaddr << "  " << "HWportnum: " << HWportnum << std::endl;
    }
    else
    {
        std::cout << "HWaddr is wrong , check the config.config" << std::endl;
        return 0;
    }
    
    
    
    struct sigaction sa;
    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, 0);
    
    //数据上传时间间隔，从配置文件获取
    uint64_t sleepTime = abs(atoi(config[35].c_str()));
    //加一个控制条件替换true，即在什么情况下进行心跳交互
    while (true)
    {
        /*
            输入shell命令生成网卡
        */
        //使用shell命令与公网服务器建立TCP连接
        system("echo \"AT^NDISDUP=1,1\" > /dev/ttyUSB0");
        sleep(3);
        system("udhcpc -i usb0");
        sleep(2);
				
				//与公网服务器建立Socket连接
        HWcilentfd = socket(AF_INET, SOCK_STREAM, 0);
        //如果HW模块与服务器成功建立Socket连接，HWcilentfd！=-1
        if (HWcilentfd == -1)
        {
            //建立Socket连接失败
            printf("HWcilentfd error!\n");
            //关闭Socket连接
            close(HWcilentfd);
        }
        
        //如果建立Socket连接失败，采取一下措施
        while (HWconnectfd == -1)
        {
            bzero(&HW_s_add, sizeof(struct sockaddr_in));
            HW_s_add.sin_family = AF_INET;
            HW_s_add.sin_addr.s_addr = inet_addr(HWipaddr.c_str());
            HW_s_add.sin_port = htons(HWportnum);
            //printf("s_addr = %#x ,port : %#x\r\n", HW_s_add.sin_addr.s_addr, HW_s_add.sin_port);
            
            //华为模块联网
            HWconnectfd = connect(HWcilentfd, (struct sockaddr *)(&HW_s_add), sizeof(struct sockaddr));
            //输出华为模块联网返回值
            printf("HWconnectfd %d , connect .. !\n", HWconnectfd);
            //华为模块联网次数计数器
            HWconnectCnt++;
            if (HWconnectCnt > 3 && HWconnectfd == -1)
            {
                HWconnected = false;
                HWconnectCnt = 0;
                std::cout << "no HW net.. ," << std::endl;
                break;
            }
            //如果华为连网成功，给出提示信息
            if (HWconnectfd != -1)
            {
                std::cout << "HW connected" << std::endl;
                HWconnectCnt = 0;
                HWconnected = true;
                break;
            }
            //sleep(1);
            usleep(200000);
        }
        
        //如果成功建立Socket连接，华为模块向服务器发送心跳包，并接收服务器返回数据
        while (HWconnected)
        {
            
            //HW heartpadload 10~30s confirm one time
            //延时2.5s
            usleep(2500000);
            //心跳数据包字符数组
            char socket_heartpadload[50];
            //校验位字符数组
            char Crc[2];
            //将socketCnt前5个字节置0
            bzero(socketCnt, 5);
            //将心跳计数格式化到socketCnt数组
            sprintf(socketCnt, "%05d", socket_cnt);
            //以下拼接心跳数据包
            //拼接帧头##02
            strcpy(socket_heartpadload, "##02");
            //拼接心跳计数器数值
            strcat(socket_heartpadload, socketCnt);
            //拼接 SGOC SSN字符
            strcat(socket_heartpadload, ",SGOC,SSN");
						//拼接SSN号
            strcat(socket_heartpadload, SSN.c_str());
            //拼接逗号分隔符
            strcat(socket_heartpadload, ",");
            //校验位计算
            char socket_heartpadload_CRC = '#'; //CRC
            for (uint16_t k = 1; k < strlen(socket_heartpadload); k++)
            {
                socket_heartpadload_CRC ^= socket_heartpadload[k];
            }
            //格式化校验位
            sprintf(Crc, "%02X", socket_heartpadload_CRC);
            //拼接校验位
            strcat(socket_heartpadload, Crc); //CRC
            //拼接结束符
            strcat(socket_heartpadload, "@@");
           	//清空接收缓冲区
            bzero(socket_recvbuf, 1024);
            //向服务器发送心跳数据包
            send(HWcilentfd, socket_heartpadload, strlen(socket_heartpadload), 0);
            //接收心跳返回数据
            recbyte = read(HWcilentfd, socket_recvbuf, 1024);
            //心跳计数加1
            socket_cnt++;
            //打印心跳数据包
            printf("HW heart:%s\n", socket_heartpadload);
            //如果没有收到服务器发回的心跳响应数据输出报错信息
            if (recbyte == -1)
            {
                printf("read service data fail !\n");
            }
            //接收到服务器返回的响应信息，校验并打印
            else if (socket_recvbuf[0] == '#' && socket_recvbuf[2] == '9')
            { 
                //printf("HW heart received\n");
                std::cout << "HW heart recvbuf: " << socket_recvbuf << std::endl;
            } 
            else
            {
                HWTimeoutCnt++;
                //输出接收超时信息
                printf("Timeout %d!\n", HWTimeoutCnt);
                if (HWTimeoutCnt > 2)
                {
                    HWTimeoutCnt = 0;
                    HWconnectfd = -1;
                    HWconnected = false;
                }
            }
            //延时2.5s
            usleep(2500000);


//            sleep(1);
        
        
            //华为模块请求服务器配置信息，上电请求一次获取配置后不再重复
          if(HWConfigtime==1){
			//只向服务器请求一次配置信息，获取之后不再请求
			HWConfigtime++;
			//服务器配置信息缓存数组
            char socket_configure[50];
            //char Crc1[2];
            //请求配置次数计数值清零
            bzero(socketCnt, 5);
            //拼接计数值
            sprintf(socketCnt, "%05d", socket_cnt);            
            strcpy(socket_configure, "##00");
            strcat(socket_configure, socketCnt);
            //拼接SSN字段
            strcat(socket_configure, ",GGDC,SSN");
            strcat(socket_configure, SSN.c_str());
            //拼接TYPGGDC
            strcat(socket_configure, ",TYPGGDC");
            //拼接VNB字段
            strcat(socket_configure, ",VNB");
            strcat(socket_configure, VNB.c_str()); 
            //拼接逗号分隔符       
            strcat(socket_configure, ",");
			//拼接校验位
            char socket_configure_CRC = '#'; //CRC
            for (uint16_t k = 1; k < strlen(socket_configure); k++)
            {
                socket_configure_CRC ^= socket_configure[k];
            }
            sprintf(Crc, "%02X", socket_configure_CRC);
            strcat(socket_configure, Crc); //CRC
            strcat(socket_configure, "@@");

            //清空配置信息存储数组
            bzero(socket_confbuf, 512);
            //发送请求服务器配置数据包
            send(HWcilentfd, socket_configure, strlen(socket_configure), 0);
            socket_cnt++;
            //打印请求服务器配置数据包
            printf("HW configure request sended\n");
            printf("HW configure request:%s\n", socket_configure);
            //读取服务器下发的配置数据
            recbyte = read(HWcilentfd, socket_confbuf, 512);
            int ismax=0;
            int islimit=0;
           	//无数据，读取失败，给出报错信息
            if (recbyte == -1)
            {
                printf("read service data fail !\n");
            }
            //读取成功，校验帧头，并打印配置信息
            else if (socket_confbuf[0] == '#' && socket_confbuf[2] == '9')
            { 
                printf("HW configure request information received\n");
                std::cout << "HW configure recvbuf: " << socket_confbuf << std::endl;
                speedcompareisready=1;
            } 

            //超时判断，超时给出提示信息
            else
            {
                HWTimeoutCnt++;
                printf("Timeout %d!\n", HWTimeoutCnt);
                if (HWTimeoutCnt > 2)
                {
                    HWTimeoutCnt = 0;
                    HWconnectfd = -1;
                    HWconnected = false;
                }
	        }
            
            //ti qu su du xian zhi
            for(int i=0,pos=0;i<255;i++)
            {   
                //SPD 
                if(socket_confbuf[i]=='S'&&socket_confbuf[i+1]=='P'&&socket_confbuf[i+2]=='D')
                {
                    pos=i+3;
                    islimit=1;
                }
                if(islimit==1)
                {
                    if(socket_confbuf[i]==','){
                        spdlimit[i-pos]='\0';
                        islimit=0;
                    } 
                    else{
                        spdlimit[i-pos]=socket_confbuf[i];
                    }   
                }
                
                //SPDMAX  
                if(socket_confbuf[i]=='M'&&socket_confbuf[i+1]=='A'&&socket_confbuf[i+2]=='X')
                {
                    pos=i+3;
                    ismax=1;
                }
                if(ismax==1)
                {
                    if(socket_confbuf[i]==','){
                        spdmax[i-pos]='\0';
                        ismax=0;
                    }  
                    else{
                        spdmax[i-pos]=socket_confbuf[i];
                    }   
                }
                
            }
            printf("\r\n \r\n \r\n \r\nspdlimit:%s\r\n",spdlimit);
            printf("spdmax:%s\r\n \r\n \r\n \r\n",spdmax);
            
            speedlimit=atof(spdlimit);
            //speedlimit=(float)atoi(spdlimit);
            printf("speedlimit:%.3f\r\n",speedlimit);
            speedmax=atof(spdmax);
            printf("speedmax:%.3f\r\n \r\n \r\n \r\n",speedmax);

            Serial_Mcu.WritePort((char *)socket_confbuf, strlen(socket_confbuf));

	            
          }
          

        }
        printf("HW close connect\n");
        // HWconnectfd = -1;
        // ZXconnectfd = -1;
        // close(HWcilentfd);
        // close(ZXcilentfd);
    }
    return 0;
} //线程1:socket

/*
    线程函数2:中兴连接心跳
*/
void *ZXsocketTcp(void *ptr)
{
    uint8_t ZXconnectCnt = 0;
    int ZXconnectfd = -1;
    int recbyte;
    char socketCnt[5];
    uint32_t socket_cnt = 1, ZXTimeoutCnt = 0;
    struct sockaddr_in ZX_s_add;
    uint16_t ZXportnum;
    std::string ZXipaddr;
    std::string adport = config[1];
    std::string::size_type index = adport.find(":");
    if (index > 0)
    {
        ZXportnum = abs(atoi(adport.substr(index + 1, adport.length() - 1 - index).c_str()));
        ZXipaddr = adport.substr(0, index);
    }
    else
    {
        std::cout << "ZXaddr is wrong , check the config.conf" << std::endl;
        return 0;
    }
    // ZXportnum = 8812;
    // ZXipaddr = "10.2.3.36";
    struct sigaction sa;
    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, 0);
    uint64_t sleepTime = abs(atoi(config[35].c_str()));
    while (true)
    {
        /*
            输入shell命令生成网卡
        */
        system("echo \"AT+CFUN=1\"\r > /dev/ttyUSB5");
        sleep(3);
        system("echo \"AT+ZGACT=1,1\"\r > /dev/ttyUSB5");
        sleep(3);
        system("udhcpc -i eth2");
        sleep(3);
        system("echo \"AT+CFUN=1\"\r > /dev/ttyUSB5");
        sleep(3);
        system("echo \"AT+ZGACT=1,1\"\r > /dev/ttyUSB5");
        sleep(3);
        system("udhcpc -i eth2");

        ZXcilentfd = socket(AF_INET, SOCK_STREAM, 0);
        if (ZXcilentfd == -1)
        {
            printf("ZXcilentfd error!\r\n");
            close(ZXcilentfd);
        }
        while (ZXconnectfd == -1)
        {
            bzero(&ZX_s_add, sizeof(struct sockaddr_in));
            ZX_s_add.sin_family = AF_INET;
            ZX_s_add.sin_addr.s_addr = inet_addr(ZXipaddr.c_str());
            ZX_s_add.sin_port = htons(ZXportnum);
            // printf("s_addr = %#x ,port : %#x\r\n", ZX_s_add.sin_addr.s_addr, ZX_s_add.sin_port);
            ZXconnectfd = connect(ZXcilentfd, (struct sockaddr *)(&ZX_s_add), sizeof(struct sockaddr));
            printf("ZXconnectfd %d\r\n", ZXconnectfd);
            if (ZXconnectCnt++ > 3 && ZXconnectfd == -1)
            {
                ZXconnected = false;
                ZXconnectCnt = 0;
                std::cout << "no ZX net.. ," << std::endl;
                break;
            }
            if (ZXconnectfd != -1)
            {
                std::cout << "ZX connected" << std::endl;
                ZXconnectCnt = 0;
                ZXconnected = true;
                break;
            }
            sleep(1);
        }
        while (ZXconnected)
        {
            sleep(sleepTime);
            char socket_heartpadload[50];
            char Crc[2];
            bzero(socketCnt, 5);
            sprintf(socketCnt, "%05d", socket_cnt);
            strcpy(socket_heartpadload, "##02");
            strcat(socket_heartpadload, socketCnt);
            strcat(socket_heartpadload, ",SGOC,SSN");
            strcat(socket_heartpadload, SSN.c_str());
            strcat(socket_heartpadload, ",");
            char socket_heartpadload_CRC = '#'; //CRC
            for (uint16_t k = 1; k < strlen(socket_heartpadload); k++)
            {
                socket_heartpadload_CRC ^= socket_heartpadload[k];
            }
            sprintf(Crc, "%02X", socket_heartpadload_CRC);
            strcat(socket_heartpadload, Crc); //CRC
            strcat(socket_heartpadload, "@@");
            bzero(socket_recvbuf, 1024);
            send(ZXcilentfd, socket_heartpadload, strlen(socket_heartpadload), 0);
            socket_cnt++;
            std::cout << "ZXcilentfd:" << ZXcilentfd << std::endl;
            printf("ZX heart:%s\r\n", socket_heartpadload);
            if (-1 == (recbyte = read(ZXcilentfd, socket_recvbuf, 1024)))
            {
                printf("read service data fail !\r\n");
            }
            else if (socket_recvbuf[0] == '#' && socket_recvbuf[2] == '9')
            { // recv a send package
                std::cout << "ZX heart recvbuf: " << socket_recvbuf << std::endl;
            } //else  recv a send package
            else
            {
                ZXTimeoutCnt++;
                printf("Timeout %d!\r\n", ZXTimeoutCnt);
                if (ZXTimeoutCnt > 2)
                {
                    ZXTimeoutCnt = 0;
                    ZXconnectfd = -1;
                    ZXconnected = false;
                }
            }
            //printf("ZXconnected %d\r\n", ZXconnected);
        } //while(HWconnected||ZXconnected)              socketsend
        printf("ZX close connect\r\n");
        // HWconnectfd = -1;
        // ZXconnectfd = -1;
        // close(HWcilentfd);
        // close(ZXcilentfd);
    }
    return 0;
} //线程2:socket

/*
    线程函数3:OBD数据发送
*/
void *OBDsend(void *ptr)
{
    char socketCnt[5];
    char Crc[2];
    uint32_t socket_cnt = 1;
    strcpy(OBDpayload_h2, ",VBAT,SSN");
    strcat(OBDpayload_h2, SSN.c_str());
    strcat(OBDpayload_h2, ",VNB");
    strcat(OBDpayload_h2, VNB.c_str());
    strcat(OBDpayload_h2, ",ENB");
    strcat(OBDpayload_h2, ENB.c_str());
    strcat(OBDpayload_h2, ",TYP1,");
    uint64_t sleepTime = abs(atoi(config[35].c_str()));
    int Maxlength_ = OBDCnt;
    while (true)
    //while (false)
    {
        sleep(sleepTime); //abs(atoi(config[20].c_str()))
        if (OBDCnt > 0)
            std::cout << "OBD sending... " << OBDCnt << std::endl;
        int Maxlength = Maxlength_; //**abs(atoi(config[20].c_str()))
        //Maxlength = (Maxlength < OBDCnt) ? Maxlength : OBDCnt;
        for (int i = 0; i < Maxlength; i++)
        {
            bzero(socketCnt, 5);
            sprintf(socketCnt, "%05d", socket_cnt);
            strcpy(OBDpayload, "##00");
            strcat(OBDpayload, socketCnt);

            //std::cout << "i..." << i << std::endl;
            //std::cout << "OBDCnt..." << OBDCnt << std::endl;
            strcat(OBDpayload, OBDpayload_h2);
            Socket_is_Send = true;
            strcat(OBDpayload, OBDPayloadSerial[i]);
            Socket_is_Send = false;
            char _CRC = '#'; //CRC
            for (uint16_t k = 1; k < strlen(OBDpayload); k++)
            { //crc
                _CRC ^= OBDpayload[k];
            }
            sprintf(Crc, "%02X", _CRC);
            strcat(OBDpayload, Crc); //CRC
            strcat(OBDpayload, "@@");
            if (cilentfd > 0)
            {
                send(cilentfd, OBDpayload, strlen(OBDpayload), 0);
                socket_cnt++;
            }
            printf("OBD: %s\n", OBDpayload);
            //printf("OBD: %s\n", OBDPayloadSerial[i]);
            bzero(OBDPayloadSerial[i], sizeof(OBDPayloadSerial[i]));
        }
        OBDCnt = 0;
    }
    return 0;
}













/*
    线程函数4:BD数据发送
*/
void *BDsend(void *ptr)
{
    //char BDpayload_h2[40];
    static char socketCnt[5];
    static char Crc[3];
    uint32_t socket_cnt = 1;
    //",GGDS,SSN***,VNB***,ENB***,"

    //北斗采样六次 上传三包
    //采样等待时间 6s
    uint64_t sleepTime = abs(atoi(config[35].c_str()));
    //上传包数 3
    int Maxlength_ = abs(atoi(config[34].c_str()));
		//加判断条件，最好是联网情况，联网正常再上传数据
    while (true)
    {
        //BD数据上传时间间隔
        sleep(sleepTime);
				//上传包数
        int Maxlength = Maxlength_;
				//接收的包数和需上传包数比较，取较小者，避免接收的数据少时出错
        Maxlength = (Maxlength < BDCnt) ? Maxlength : BDCnt;
        //打印接收的BD数据包数
        std::cout << "BDCnt..." << BDCnt << std::endl;	
        //if (BDCnt > 0)
        //    std::cout << "BDCnt..." << BDCnt << std::endl;

				//打包上传BD数据到服务器
        for (int i = 0; i < Maxlength; i++)
        {
            //socketCnt置0
            bzero(socketCnt, 5);
            //格式化计数值到socketCnt
            sprintf(socketCnt, "%05d", socket_cnt);
            //拼接帧头 ##00
            strcpy(BDpayload, "##00");
            //拼接计数值
            strcat(BDpayload, socketCnt);
            //拼接GGDS SSN 字段
            strcat(BDpayload, ",GGDS,SSN");
            //拼接SSN号
            strcat(BDpayload, SSN.c_str());
						//拼接VNB字段
            strcat(BDpayload, ",VNB");
            //拼接VNB号
            strcat(BDpayload, VNB.c_str());
						//拼接ENB字段
            strcat(BDpayload, ",ENB");
            //拼接ENB号
            strcat(BDpayload, ENB.c_str());
						//拼接逗号拼接符
            strcat(BDpayload, ",");
            
            Socket_is_Send = true;
            //拼接BD GGDS数据
            strcat(BDpayload, BDPayloadSerial[i]);
            Socket_is_Send = false;
            //拼接校验字符
            char _CRC = '#'; //CRC
            for (uint16_t k = 1; k < strlen(BDpayload); k++)
            {
                _CRC ^= BDpayload[k];
            }
            sprintf(Crc, "%02X", _CRC);
            Crc[2] = '\0';
            strcat(BDpayload, Crc); //CRC
            //拼接结束符
            strcat(BDpayload, "@@");
            
            //根据联网状态和联网模式（公网/专网）向服务器发送BD数据
            if (cilentfd > 0)
            {
                send(cilentfd, BDpayload, strlen(BDpayload), 0);
                socket_cnt++;
            }
            //将数据打印输出
            printf("BD upload data: %s\n", BDpayload);


            int bdpos,isbdspeed;
            for(int j=0;j<50;j++){
                if(BDPayloadSerial[i][j]=='S' && BDPayloadSerial[i][j+1]=='P' && BDPayloadSerial[i][j+2]=='D')
                {
                    bdpos=j+3;
                    isbdspeed=1;
                }
                if(isbdspeed==1)
                {
                    if(BDPayloadSerial[i][j]==',')
                    {
                        spdnew[j-bdpos]='\0';
                        isbdspeed=0;
                    }
                    else{
                        spdnew[j-bdpos]=BDPayloadSerial[i][j];
                    }

                }

            }
            printf("\r\nspdnew:%s\r\n",spdnew);        
            speednew=atof(spdnew);
            printf("\r\nspeednew:%.3f\r\n",speednew);
            //huo qu pei zhi hou zai bi jiao shang chuan gao jing 
            if(speedcompareisready==1 && speednew>=speedlimit && speednew<=speedmax){
                //chaosu
                printf("\r\nspeed over the limit line!\r\n");
                Serial_Mcu.WritePort((char *)CHAOSU, 13);
                //##0000373,GGDC,SSN1802131077,TYPGGVA,VNBB023,ENB12345,LGT116.5864258,LTT40.0767746,SPD50.73,MSP40.00,SDT20180820155936,DRT253.6,62

                sprintf(socketCnt, "%05d", socket_cnt);
                //拼接帧头 ##00
                strcpy(Speedpayload, "##00");
                //拼接计数值
                strcat(Speedpayload, socketCnt);
                //拼接GGDS SSN 字段
                strcat(Speedpayload, ",GGDC,SSN");
                //拼接SSN号
                strcat(Speedpayload, SSN.c_str());
                strcat(Speedpayload, ",TYPGGVA");
				//拼接VNB字段
                strcat(Speedpayload, ",VNB");
                //拼接VNB号
                strcat(Speedpayload, VNB.c_str());
				//拼接ENB字段
                strcat(Speedpayload, ",ENB");
                //拼接ENB号
                strcat(Speedpayload, ENB.c_str());
				//拼接逗号拼接符
                strcat(Speedpayload, ",");
                Socket_is_Send = true;
                //拼接BD GGDC数据
                strcat(Speedpayload, SpeedPayloadSerial[i]);
                Socket_is_Send = false;
                //拼接校验字符
                char _CRC = '#'; //CRC
                for (uint16_t k = 1; k < strlen(Speedpayload); k++)
                {
                    _CRC ^= Speedpayload[k];
                }
                sprintf(Crc, "%02X", _CRC);
                Crc[2] = '\0';
                strcat(Speedpayload, Crc); //CRC
                //拼接结束符
                strcat(Speedpayload, "@@");
            
                //根据联网状态和联网模式（公网/专网）向服务器发送BD数据
                if (cilentfd > 0)
                {
                    send(cilentfd, Speedpayload, strlen(Speedpayload), 0);
                    socket_cnt++;
                }
                //将数据打印输出
                printf("Speed alarm upload data: %s\n", Speedpayload);


            }
            else if(speednew>speedmax){
                //chaoguozuidasudu
                printf("\r\nspeed over the max line!\r\n"); 
                //Serial_Mcu.WritePort((char *)CHAOSU, 13);   
            }
            else{
                printf("\r\nspeed is normal.\r\n");

            }
            //bzero(BDpayload,sizeof(BDpayload));
            bzero(BDPayloadSerial[i], sizeof(BDPayloadSerial[i]));
        }
        BDCnt = 0;
    }
    return 0;
}
















/*
    线程函数5:蓝牙数据发送
*/
void *BLUEsend(void *ptr)
{
    char socketCnt[5];
    char Crc[2];
    uint32_t socket_cnt = 1;
    strcpy(BLUEpayload_h2, ",BLUE,SSN");
    strcat(BLUEpayload_h2, SSN.c_str());
    strcat(BLUEpayload_h2, ",SDC");

    int Maxlength_ = abs(atoi(config[23].c_str()));
    uint64_t sleepTime = abs(atoi(config[24].c_str()));
    while (true)
    {
        sleep(sleepTime);
        int Maxlength = Maxlength_;
        Maxlength = (Maxlength < BLUECnt) ? Maxlength : BLUECnt;
        if (BLUECnt > 0)
            //std::cout << "BLUE sending... " << BLUECnt << std::endl;
        for (int i = 0; i < Maxlength; i++)
        {
            bzero(socketCnt, 5);
            sprintf(socketCnt, "%05d", socket_cnt);

            strcpy(BLUEpayload, "##00");
            strcat(BLUEpayload, socketCnt);
            strcat(BLUEpayload, BLUEpayload_h2);

            Socket_is_Send = true;
            strcat(BLUEpayload, BLUEPayloadSerial[i]);
            Socket_is_Send = false;

            strcat(BLUEpayload, "STM");
            strcat(BLUEpayload, BDdate);
            //strcat(BLUEpayload, date.c_str());
            strcat(BLUEpayload, ",");
            char _CRC = '#'; //CRC
            for (uint16_t k = 1; k < strlen(BLUEpayload); k++)
            {
                _CRC ^= BLUEpayload[k];
            }
            sprintf(Crc, "%02X", _CRC);
            strcat(BLUEpayload, Crc); //CRC
            strcat(BLUEpayload, "@@");
            if (cilentfd > 0)
            {
                send(cilentfd, BLUEpayload, strlen(BLUEpayload), 0);
                socket_cnt++;
            }
            printf("BLUE upload data: %s\n", BLUEpayload);
            /*
            for (uint16_t k = 0; k < 100; k++)
            {
                printf("%2X %c ,\n", BLUEPayloadSerial[i][k], BLUEPayloadSerial[i][k]);
            }
            */
            bzero(BLUEPayloadSerial[i], sizeof(BLUEPayloadSerial[i]));
        }
        BLUECnt = 0;
    }
    return 0;
}

/*
    线程函数6:其他数据发送
*/
void *OTHERsend(void *ptr)
{
    char OTHERpayload[750], OTHERpayload_h2[40];
    char socketCnt[5];
    char Crc[2];
    std::string fy, hg, ph, kg;
    fy = config[37];
    hg = config[38];
    ph = config[39];
    kg = config[40];
    uint32_t socket_cnt = 1;
    strcpy(OTHERpayload_h2, ",DEVS,SSN");
    strcat(OTHERpayload_h2, SSN.c_str());
    uint64_t sleepTime = abs(atoi(config[36].c_str())); //**
                                                        //   while (true)
    while (true)
    {
        //usleep(1000000);
        sleep(sleepTime);
//        sleep(sleepTime);
//        sleep(sleepTime);
//        sleep(sleepTime);
        //std::cout<<"sleepTime:"<< sleepTime << std::endl;
        //int Maxlength = Maxlength_; //**abs(atoi(config[20].c_str()))
        //Maxlength = (Maxlength < BLUECnt) ? Maxlength : BLUECnt;
        //std::cout << "OTHER sending..." << std::endl;
        //std::cout << "HWcilentfd:" << HWcilentfd << std::endl;
        //std::cout << "ZXcilentfd:" << ZXcilentfd << std::endl;
        std::cout << "cilentfd:" << cilentfd << std::endl;
        bzero(socketCnt, 5);
        sprintf(socketCnt, "%05d", socket_cnt);
        strcpy(OTHERpayload, "##00");
        strcat(OTHERpayload, socketCnt);

        strcat(OTHERpayload, OTHERpayload_h2);

        if (1) //判断内部外部供电
            strcat(OTHERpayload, ",BAT1");
        else
            strcat(OTHERpayload, ",BAT0");
        strcat(OTHERpayload, ",BAP");
        char buff[4];
//        strcat(OTHERpayload, BAP.c_str()); //拼接BAP

        strcat(OTHERpayload, ",TEM");
//        strcat(OTHERpayload, TEM.c_str()); //拼接TEM

        strcat(OTHERpayload, ",LIN1");

        if (ZXcilentfd > 0)
            strcat(OTHERpayload, ",SI11");
        else
            strcat(OTHERpayload, ",SI10");

        if (HWcilentfd > 0)
            strcat(OTHERpayload, ",SI21");
        else
            strcat(OTHERpayload, ",SI20");


        if (1) //判断USB
            strcat(OTHERpayload, ",USB1");
        else
            strcat(OTHERpayload, ",USB2");
        /*XX@@ */
        if ((HW_NTgps || ZX_NTgps) && Serial_Mcu.GGA[47]=='2')
            strcat(OTHERpayload, ",GPS2");
        else
            strcat(OTHERpayload, ",GPS1");

        strcat(OTHERpayload, ",SLN");
        //tiqu SLN
        SLN=(Serial_Mcu.GGA[49]-0x30)*10+(Serial_Mcu.GGA[50]-0x30);
        sprintf(buff, "%d",SLN);
        strcat(OTHERpayload, buff); //拼接SLN

        strcat(OTHERpayload, ",INS");
        strcat(OTHERpayload, kg.c_str()); //INS

        strcat(OTHERpayload, ",PIT"); //PIT ROL YAW
        strcat(OTHERpayload, fy.c_str());
        strcat(OTHERpayload, ",ROL");
        strcat(OTHERpayload, hg.c_str());
        strcat(OTHERpayload, ",YAW");
        strcat(OTHERpayload, ph.c_str());

        char qiehuan[5];
        strcat(OTHERpayload, ",DRS"); //专网切出信号强度门限，默认值-9999
        sprintf(qiehuan, "%05d", DRS);
        strcat(OTHERpayload, qiehuan);
        strcat(OTHERpayload, ",URS"); //专网切回信号强度门限，默认值-9999
        sprintf(qiehuan, "%05d", URS);
        strcat(OTHERpayload, qiehuan);
        strcat(OTHERpayload, ",DCR"); //专网切出信号质量门限，默认值-9999
        sprintf(qiehuan, "%05d", DCR);
        strcat(OTHERpayload, qiehuan);
        strcat(OTHERpayload, ",UCR"); //专网切回信号质量门限，默认值-9999
        sprintf(qiehuan, "%05d", UCR);
        strcat(OTHERpayload, qiehuan);
        strcat(OTHERpayload, ",DCN");
        sprintf(qiehuan, "%05d", DCNCnt); //
        strcat(OTHERpayload, qiehuan);
        strcat(OTHERpayload, ",UCN");
        sprintf(qiehuan, "%05d", UCNCnt); //
        strcat(OTHERpayload, qiehuan);

        char Rsi[2];
        strcat(OTHERpayload, ",RSI-"); //RSI-85
        sprintf(Rsi, "%02d", rssi);    //
        strcat(OTHERpayload, qiehuan);
        strcat(OTHERpayload, ",CIN"); //CIN24
        sprintf(Rsi, "%02d", rssi_);
        strcat(OTHERpayload, qiehuan);

        strcat(OTHERpayload, ",RPR");            //RPR
        strcat(OTHERpayload, ZXipLocal.c_str()); //ZXipLocal
        strcat(OTHERpayload, ",RPU");            //RPU,
        strcat(OTHERpayload, HWipLocal.c_str()); //HWipLocal

        if (whichCilent == 0) //NET2,
            strcat(OTHERpayload, ",NET1");
        else
            strcat(OTHERpayload, ",NET2");

        strcat(OTHERpayload, ",WSN"); //WSN869524020401641
        strcat(OTHERpayload, WSN.c_str());

        strcat(OTHERpayload, ",STM"); //STM20180717143430

        strcat(OTHERpayload, BDdate);
        //printf("shijian:%s\r\n",date.c_str());

        strcat(OTHERpayload, ",");

        char _CRC = '#'; //CRC
        for (uint16_t k = 1; k < strlen(OTHERpayload); k++)
        {
            _CRC ^= OTHERpayload[k];
        }
        sprintf(Crc, "%02X", _CRC);
        strcat(OTHERpayload, Crc); //CRC
        strcat(OTHERpayload, "@@");
        if (cilentfd > 0)
        {
            send(cilentfd, OTHERpayload, strlen(OTHERpayload), 0);
            socket_cnt++;
        }
        std::cout <<"OTHER:"<< OTHERpayload << std::endl;
        //printf("OTHER: %s\r\n", OTHERpayload);
    }
    return 0;
}

/*
    线程函数7:差分定位连接
*/
void *HWnTrip(void *ptr)
{
    int NTcilentfd = -1, NTTimeoutCnt = 0, NTconnectCnt = 0;
    int NTconnectfd = -1;
    int recbyte = 0;
    bool NTconnected = false;
    std::string NTname; //Ntrip name
    struct sockaddr_in NT_s_add;
    std::string NTportnum; //Ntrip port
    std::string NTipaddr;  //Ntrip ipaddr
    std::string adport = config[2];
    std::string::size_type index = adport.find(":");

    //HW Ntrip connecting
    std::cout << "HW Ntrip connecting" << std::endl;
    usleep(500000);

    if (index > 0)
    {
        NTportnum = adport.substr(index + 1, adport.length() - 1 - index);
        NTipaddr = adport.substr(0, index);
    }
    else
    {
        std::cout << "HW_NT_addr is wrong" << std::endl;
        return 0;
    }

    struct sigaction sa;
    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, 0);
    // std::stringstream stream;
    while (true)
    {
        NTcilentfd = socket(AF_INET, SOCK_STREAM, 0);
        if (NTcilentfd == -1)
        {
            printf("HW_NTcilentfd error!\n");
        }
        while (NTconnectfd == -1) //TCP CREATE
        {
            bzero(&NT_s_add, sizeof(struct sockaddr_in));
            NT_s_add.sin_family = AF_INET;
            NT_s_add.sin_addr.s_addr = inet_addr(NTipaddr.c_str()); //inet_addr为数据转换
            NT_s_add.sin_port = htons(atoi(NTportnum.c_str()));
            NTconnectfd = connect(NTcilentfd, (struct sockaddr *)(&NT_s_add), sizeof(struct sockaddr)); //建立一个与端口的连接
            // usleep(100000);
            printf("HW_NTconnectfd %d , connect .. !\n", NTconnectfd);
            if (NTconnectfd != -1)
            {
                std::cout << "HW_NT connected" << std::endl;
                NTconnected = true;
                break;
            }
            else if (NTconnectCnt++ > 3)
            {
                NTconnected = false;
                NTconnectCnt = 0;
                std::cout << "no HW_NT net.. ," << std::endl;
                break;
            }
            usleep(300000);
        }
        while (NTconnected)
        {
            std::cout << "NTconnected GET 0" << std::endl;
            NTname = config[5];
            std::string NTnamesend = "GET /" + NTname + " HTTP/1.0\r\n";
            // Ntrip_hwQ64, Ntrip_zxQ64
            std::string Host = "Host: " + NTipaddr + ":" + NTportnum + "\r\n";
            // "Ntrip-Version: Ntrip/2.0\r\n"
            std::string Radio = "User-Agent: NTRIP GNSSInternetRadio/1.0.0\r\n";
            std::string Connection = "Connection: close\r\n";
            //std::string NTusrpassword = "Authorization: Basic " + Ntrip_hwQ64 + "\r\n"+"\r\n";
            std::string NTusrpassword = "Authorization: Basic dXNlcjpwYXNz\r\n\r\n";

            std::string NTsocket_sendbuf = NTnamesend + Host + "Ntrip-Version: Ntrip/2.0\r\n" + Radio +
                                           Connection + NTusrpassword;
            send(NTcilentfd, NTsocket_sendbuf.c_str(), NTsocket_sendbuf.length(), 0);
            bzero(NTsocket_recvbuf, 4096);
            recbyte = read(NTcilentfd, NTsocket_recvbuf, 4096);
            // if (NTsocket_recvbuf[0] == '2' && NTsocket_recvbuf[1] == '0')
            //     break;
            if (recbyte > 0)
            {
                //std::cout << "\r\n HW_NT recbyte: \r\n" << recbyte << std::endl;
                //std::cout << "\r\n HW_NTsocket_recvbuf: \r\n" << NTsocket_recvbuf << std::endl;
                HW_NTgps = true;
                break;
            }
            usleep(300000);
        }
        while (NTconnected)
        {
            sleep(1);
            /*gga*/
            //std::cout << "NTconnected 0" << std::endl;
            if (Serial_Mcu.GGA_Send)
            {
                send(NTcilentfd, Serial_Mcu.GGA, Serial_Mcu.GGA_DatLen, 0);
                //std::cout << "HW NTrip send: " << Serial_Mcu.GGA << std::endl;
                
                //printf("\r\n weizhi46: %c\r\n",Serial_Mcu.GGA[45]);
                //printf("\r\n weizhi47: %c\r\n",Serial_Mcu.GGA[46]);
                //Ntrip state word
                //printf("\r\n weizhi48: %c\r\n",Serial_Mcu.GGA[47]);
                /*
                for(int i=0;i<100;i++){
                    if(Serial_Mcu.GGA[i]=='E')
                        printf("\r\n weizhi: %d\r\n",i);
                }
                */

            }
            if (whichCilent == 0)
            {
                //std::cout << "whichCilent 0" << std::endl;
                bzero(NTsocket_recvbuf, 4096);
                recbyte = read(NTcilentfd, NTsocket_recvbuf, 4096);
                if (recbyte > 0)
                {
                    char start[3] = {0x0a, 'A', 'N'};
                    // char end[1] = {0x0d};
                    char end = 0x0d;
                    Serial_Mcu.WritePort(start, 3);
                    Serial_Mcu.WritePort(NTsocket_recvbuf, recbyte);
                    Serial_Mcu.WritePort(&end, 1);
                    //printf("NTsocket_recvbuf, %d: ", recbyte);
                    // for(int i = 0;i<recbyte;i++)
                    //     printf("%02X ",NTsocket_recvbuf[i]);
                    // printf("\n");
                }
                else
                {
                    NTTimeoutCnt++;
                    printf("HW_NT Timeout %d!\n", NTTimeoutCnt);
                    if (NTTimeoutCnt > 2)
                    {
                        NTTimeoutCnt = 0;
                        NTconnectfd = -1;
                        NTconnected = false;
                        HW_NTgps = false;
                        close(NTcilentfd);
                    }
                }
            }
        }
        printf("HWNtrip close connect\r\n");
    }
    return 0;
}

/*
    线程函数7:差分定位连接
*/
void *ZXnTrip(void *ptr)
{
    int NTcilentfd = -1, NTTimeoutCnt = 0, NTconnectCnt = 0;
    int NTconnectfd = -1;
    int recbyte = 0;
    bool NTconnected = false;
    std::string NTname;
    // unsigned int TimeoutCnt = 0;
    struct sockaddr_in NT_s_add;
    std::string NTportnum;
    std::string NTipaddr;
    std::string adport = config[6];
    std::string::size_type index = adport.find(":");
    if (index > 0)
    {
        NTportnum = adport.substr(index + 1, adport.length() - 1 - index);
        NTipaddr = adport.substr(0, index);
    }
    else
    {
        std::cout << "ZX_NT_addr is wrong" << std::endl;
        return 0;
    }
    struct sigaction sa;
    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, 0);
    // std::stringstream stream;
    while (true)
    {
        NTcilentfd = socket(AF_INET, SOCK_STREAM, 0);
        if (NTcilentfd == -1)
        {
            printf("ZX_NTcilentfd error!\n");
            close(NTcilentfd);
        }
        while (NTconnectfd == -1)
        {
            bzero(&NT_s_add, sizeof(struct sockaddr_in));
            NT_s_add.sin_family = AF_INET;
            NT_s_add.sin_addr.s_addr = inet_addr(NTipaddr.c_str()); //inet_addr为数据转换
            NT_s_add.sin_port = htons(atoi(NTportnum.c_str()));
            printf("s_addr = %#x ,port : %#x\n", NT_s_add.sin_addr.s_addr, NT_s_add.sin_port);
            NTconnectfd = connect(NTcilentfd, (struct sockaddr *)(&NT_s_add), sizeof(struct sockaddr)); //建立一个与端口的连接
            // usleep(100000);
            printf("ZX_NTconnectfd %d , connect .. !\n", NTconnectfd);
            if (NTconnectfd != -1)
            {
                std::cout << "ZX_NT connected" << std::endl;
                NTconnected = true;
                break;
            }
            else if (NTconnectCnt++ > 3)
            {
                NTconnected = false;
                NTconnectCnt = 0;
                std::cout << "no ZX_NT net.. ," << std::endl;
                break;
            }
            usleep(300000);
        }
        while (NTconnected)
        {
            std::cout << "NTconnected GET 1" << std::endl;
            NTname = config[9];
            std::string NTnamesend = "GET /" + NTname + " HTTP/1.0\r\n";
            // Ntrip_hwQ64, Ntrip_zxQ64
            std::string Host = NTipaddr + ":" + NTportnum + "\r\n";
            // "Ntrip-Vertion: Ntrip/2.0\r\n"
            std::string Radio = "User-Agent: NTRIP GNSSInternetRadio/1.0.0\r\n";
            std::string Connection = "Connection: close\r\n";
            std::string NTusrpassword = "Authorization: Basic " + Ntrip_zxQ64 + "\r\n" + "\r\n";

            std::string NTsocket_sendbuf = NTnamesend + Host + "Ntrip-Vertion: Ntrip/2.0\r\n" + Radio +
                                           Connection + NTusrpassword;

            send(NTcilentfd, NTsocket_sendbuf.c_str(), NTsocket_sendbuf.length(), 0);
            bzero(NTsocket_recvbuf, 4096);
            recbyte = read(NTcilentfd, NTsocket_recvbuf, 4096);
            // if (NTsocket_recvbuf[0] == '2' && NTsocket_recvbuf[1] == '0')
            //     break;
            if (recbyte > 0)
            {
                //std::cout << "ZX_NT recbyte: " << recbyte << std::endl;
                //std::cout << "ZX_NTsocket_recvbuf: " << NTsocket_recvbuf << std::endl;
                ZX_NTgps = true;
                break;
            }
            usleep(300000);
        }
        while (NTconnected)
        {
            sleep(1);
            std::cout << "NTconnected 1" << std::endl;
            if (Serial_Mcu.GGA_Send)
            {
                send(NTcilentfd, Serial_Mcu.GGA, Serial_Mcu.GGA_DatLen, 0);
                //std::cout << "ZX NTrip send: " << Serial_Mcu.GGA << std::endl;
            }
            if (whichCilent == 1)
            {
                //std::cout << "whichCilent 1" << std::endl;
                bzero(NTsocket_recvbuf, 4096);
                recbyte = read(NTcilentfd, NTsocket_recvbuf, 4096);
                if (recbyte > 0)
                {
                    char start[3] = {0x0a, 'A', 'N'};
                    // char end[1] = {0x0d};
                    char end = 0x0d;
                    Serial_Mcu.WritePort(start, 3);
                    Serial_Mcu.WritePort(NTsocket_recvbuf, recbyte);
                    Serial_Mcu.WritePort(&end, 1);
                }
                else
                {
                    NTTimeoutCnt++;
                    printf("Timeout %d!\n", NTTimeoutCnt);
                    if (NTTimeoutCnt > 2)
                    {
                        NTTimeoutCnt = 0;
                        NTconnectfd = -1;
                        NTconnected = false;
                        ZX_NTgps = false;
                        close(NTcilentfd);
                    }
                }
            }
        }
        printf("ZXNtrip close connect\r\n");
    }
    return 0;
}

/*
    线程函数9:单片机心跳线程
*/
void *irq(void *ptr)
{
    system("echo out >" DEV_PATH45 "/direction");

    
    while (true)
    {
        usleep(5000000);
        system("echo 1 >> " DEV_PATH45 "/value");
        usleep(5000000);
        system("echo 0 >> " DEV_PATH45 "/value");
        printf("\r\nLinux heart send!\r\n\r\n");
    }
    return 0;
}

void *getInfo(void *ptr)
{
    int sock;
    struct sockaddr_in sin;
    struct ifreq ifrh, ifrz;

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == -1)
    {
        perror("socket");
    }
    strncpy(ifrh.ifr_name, "usb0", IFNAMSIZ);
    ifrh.ifr_name[IFNAMSIZ - 1] = 0;
    strncpy(ifrz.ifr_name, "eth2", IFNAMSIZ);
    ifrz.ifr_name[IFNAMSIZ - 1] = 0;
    while (true)
    {
        if (ioctl(sock, SIOCGIFADDR, &ifrh) < 0)
        {
            perror("usb0 ioctl");
        }
        else
        {
            memcpy(&sin, &ifrh.ifr_addr, sizeof(sin));
            HWipLocal = inet_ntoa(sin.sin_addr);
            //std::cout << "HWipLocal: " << HWipLocal << std::endl;
        }

        if (ioctl(sock, SIOCGIFADDR, &ifrz) < 0)
        {
            //perror("eth2 ioctl");
        }
        else
        {
            memcpy(&sin, &ifrz.ifr_addr, sizeof(sin));
            ZXipLocal = inet_ntoa(sin.sin_addr);
            std::cout << "ZXipLocal: " << ZXipLocal << std::endl;
        }
        sleep(10);
    }
    return 0;
}

void *ZXAT(void *ptr)
{
    int fd, wr_num, rd_num, status;
    uint8_t reqTime = 0;
    struct termios Opt;
    char send_buf[] = "CSQ?", recv_buf[4096];
    fd = open("/dev/ttyUSB5", O_RDWR);
    while ((fd = open("/dev/ttyUSB5", O_RDWR)) == -1)
    {
        sleep(3);
        printf("can't not open the COM1! \n");
    }
    cfsetispeed(&Opt, B115200);
    cfsetospeed(&Opt, B115200);
    status = tcsetattr(fd, TCSANOW, &Opt);
    tcflush(fd, TCIOFLUSH);
    while (1)
    {
        wr_num = write(fd, send_buf, strlen(send_buf));
        sleep(2);
        rd_num = read(fd, recv_buf, sizeof(recv_buf));
        if (rd_num > 0)
        {
            reqTime = 0;
            std::string strfind = "CSQ:";
            std::string payload = recv_buf;
            std::string::size_type index = payload.find(strfind);
            if (index != std::string::npos)
                rssi = atoi(payload.substr(index + strfind.length(), 2).c_str());
            if (rssi < 0)
                rssi = 0;
            else if (rssi > 99)
                rssi = 99;
            printf("recv_message: %s ,rssi: %d \n", recv_buf, rssi);
        }
        else
        {
            reqTime++;
        }
        if (reqTime > 5)
        {
            reqTime = 6;
            rssi = 0;
        }
        sleep(3);
    }
    return 0;
}

/*******************************************************************************
**函数名称：Main主函数
**函数功能：主循环配置工作流程
**修改日期：2018-08-22
********************************************************************************/
int main(int argc, char **argv)
{
	
    uint8_t confCnt = 0;
    system("ulimit -s 102400");
    std::fstream Conf;

    //打开配置文件：config.cng ，Conf是文件句柄
    Conf.open("/opt/config.conf", std::ios::in); //|std::ios::out|std::ios::binary);
    //成功打开
    if (Conf.is_open())
    {
        //定义读取配置文件的buf
        char buf[50];
        //按行读取，根据结束符判断是否读完
        while (!Conf.eof())
        {
            //获取一行数据到buf
            Conf.getline(buf, 50);
            //将该行数据压如配置文件容器config
            config.push_back(buf);
            //打印当前读取的行号
            printf(" %d: ", confCnt);
            //打印当前行的数据
            std::cout << config[confCnt] << std::endl;
            //行数加1
            confCnt++;
        }
        //读取完毕，关闭文件句柄
        Conf.close();
    }
    //如果打开失败，给出报错信息
    else
    {
        printf("%s", "Open config.cnf file error!");
    }

    //打开版本文件：version.conf，文件句柄Conf
    Conf.open("/opt/version.conf", std::ios::in); //|std::ios::out|std::ios::binary);
    //成功打开
    if (Conf.is_open())
    {
        //定义读取buf
        char buf[50];
        //读取一行数据到buf
        Conf.getline(buf, 50);
        //将该行数据压如config容器
        config.push_back(buf);
        //输出行号
        printf(" %d: ", confCnt);
        //输出版本信息
        std::cout << config[confCnt] << std::endl;
        //行号加1
        confCnt++;
        //关闭文件句柄
        Conf.close();
    }
    //如果打开失败，给出报错信息
    else
    {
        printf("%s", "Open version.conf file error!");
    }

    //打开设备编号文件 SSN.conf
    Conf.open("/opt/SSN.conf", std::ios::in); //|std::ios::out|std::ios::binary);
    //成功打开
    if (Conf.is_open())
    {
        //定义读取buf
        char buf[50];
        //读取一行数据
        Conf.getline(buf, 50);
        //将该行数据压入config容器
        config.push_back(buf);
        //输出行号
        printf(" %d: ", confCnt);
        //输出设备编号
        std::cout << config[confCnt] << std::endl;
        //行号加1
        confCnt++;
        //关闭文件句柄
        Conf.close();
    }
    //如果打开失败，给出报错信息
    else
    {
        printf("%s", "Open SSN.conf file error!");
    }


    //++++++add judge for system initialization state++++++
    if (true)
    {
        //定义华为和中兴账号密码字符串
        std::string confighw = "", configzx = "";
        //华为公网用户名和密码，中间加”：“
        confighw = config[3] + ":" + config[4];
        //中兴专网用户名和密码，中间加”：“
        configzx = config[7] + ":" + config[8];
        
        // confighw.append((config[3].append(":")).append(config[4]));
        // configzx.append((config[7].append(":")).append(config[8]));
        
        //取华为字符串首地址
        char *src_hw = (char *)confighw.c_str();
        //取中兴字符串首地址
        char *src_zx = (char *)configzx.c_str();
        //定义华为和中兴加密后数据首地址
        char *buf_hw, *buf_zx;
        
        //华为Ntrip账号和密码加密
        base64_encode(src_hw, &buf_hw);
        //中兴Ntrip账号和密码加密
        base64_encode(src_zx, &buf_zx);
        //华为Ntrip账号密码
        Ntrip_hwQ64 = buf_hw;
        //中兴Ntrip账号密码
        Ntrip_zxQ64 = buf_zx;
        
        //定义index索引变量
        std::string::size_type index;
        //寻找加密字符串末尾“=”，将字符计数给index
        index = Ntrip_hwQ64.find_last_of("=");
        //如果index不为0
        if (index != std::string::npos)
        		//将index前所有字符复制给变量
            Ntrip_hwQ64 = Ntrip_hwQ64.substr(0, index);
        //寻找加密字符串末尾“=”，将字符计数给index
        index = Ntrip_zxQ64.find_last_of("=");
        //如果index不为0
        if (index != std::string::npos)
        		//将index前所有字符复制给变量
            Ntrip_zxQ64 = Ntrip_zxQ64.substr(0, index);
            
       	//输出华为账号密码加密结果
        printf("base64 encode HW:%s\n", Ntrip_hwQ64.c_str());
        //输出中兴账号密码加密结果
        printf("base64 encode ZX:%s\n", Ntrip_zxQ64.c_str());
        
        //释放变量内存
        free(buf_hw);
        free(buf_zx);
        buf_hw = NULL;
        buf_zx = NULL;

        //读取SSN号
        SSN = config[confCnt - 1];
        //读取VNB号
        VNB = config[16];
        //读取ENB号
        ENB = config[17];
    }
    //加1s延时
    sleep(1);

    //

    if (true)
    {		
    		//定义帧头
//        char start[3] = {0x0a, 'A', 'B'};
        //定义结束符和中间分隔符
//        char end = 0x0d, mid = ':',comma = ',';
        //UART4:115200, A3352<--->OBD
        //打开串口4，波特率115200
        if ((Serial_OBD.OpenPort(4, 115200, '8', '1', 'N')) < 0)
        {
            //打开失败给出提示信息
            printf("open Serial_OBD is failed\n");
            return 0;
        }
        //UART3:19200, A3352<--->DDP   与蜂鸣器合并
        //打开串口3，波特率19200
        if ((Serial_ddp.OpenPort(3, 19200, '8', '1', 'N')) < 0)
        {
            //打开失败给出提示信息
            printf("open Serial_ddp is failed\n");
            return 0;
        }
        //UART1:115200, A3352<--->MCU
        //打开串口1，波特率115200
        if ((Serial_Mcu.OpenPort(1, 115200, '8', '1', 'N')) < 0)
        {
            //打开失败给出提示信息
            printf("open Serial_Mcu is failed\n");
            return 0;
        }
        //UART2:38400,  A3352<---->PC配置串口
        //打开串口2，波特率38400,
        if ((Serial_Com.OpenPort(2, 38400, '8', '1', 'N')) < 0)
        {
            //打开失败给出提示信息
            printf("open Serial_Com is failed\n");
            return 0;
        }
        
        //A3352<--->1 to 4(RS232)
				 if ((Serial_Com.OpenPort(5, 19200, '8', '1', 'N')) < 0)
        {
            //打开失败给出提示信息
            printf("open Serial_Com is failed\n");
            return 0;
        }

/*

        while(!Serial_Mcu.config_received32)
        {
            std::cout<<"requestreceivedfrom32:"<<Serial_Mcu.requestreceivedfrom32<<std::endl;
            std::cout<<"config_received32:"<<Serial_Mcu.config_received32<<std::endl;
            //if(true){
            if(Serial_Mcu.requestreceivedfrom32==true){
                //发送帧头
				Serial_Mcu.WritePort(start, 3);
				//逐行将数据发送给STM32
				for(int i=0;i<43;i++){
					Serial_Mcu.WritePort((char *)config[i].c_str(), config[i].length());
					Serial_Mcu.WritePort(&comma, 1);
				}
				//发送帧尾
				Serial_Mcu.WritePort(&end, 1);
                printf("\r\n\r\n\r\n\r\nConfig five has sended.\r\n\r\n\r\n\r\n");
                usleep(500000);	
            }
            usleep(500000);
            
        }		


*/



				/*原王催代码
        //send commond to 32
        //0x0a 'A' 'B'
        Serial_Mcu.WritePort(start, 3); // blue name keyword
        //config[21]
        Serial_Mcu.WritePort((char *)config[21].c_str(), config[21].length());
        //':'
        Serial_Mcu.WritePort((char *)":", 1);
        //config[22]
        Serial_Mcu.WritePort((char *)config[22].c_str(), config[22].length());
        //0x0d
        Serial_Mcu.WritePort(&end, 1);

        //delay for a while
        usleep(100000);

        if (config[25] != "1") //blue open ?
        {
            start[2] = 'C';
            Serial_Mcu.WritePort(start, 3);
            Serial_Mcu.WritePort(&end, 1);
        }

        char set[] = "You can set config...";
        for (uint16_t i = 0; i < 3; i++)
        {
            Serial_Com.WritePort(set, strlen(set));
            printf("%s\n", set);
            usleep(100000);
        }

        std::string adport = config[0];
        std::string::size_type index = adport.find(":");
        if (index > 0) //ip port
        {
            start[2] = 'I';
            Serial_Mcu.WritePort(start, 3);
            Serial_Mcu.WritePort((char *)adport.substr(0, index).c_str(), index);
            Serial_Mcu.WritePort(&end, 1);
            usleep(100000);
            start[2] = 'P';
            Serial_Mcu.WritePort(start, 3);
            Serial_Mcu.WritePort((char *)adport.substr(index + 1, adport.length() - 1 - index).c_str(), adport.length() - 1 - index);
            Serial_Mcu.WritePort(&end, 1);
            usleep(100000);
        }

        start[2] = 'S';
        Serial_Mcu.WritePort(start, 3);
        Serial_Mcu.WritePort((char *)SSN.c_str(), SSN.length());
        Serial_Mcu.WritePort(&mid, 1);
        Serial_Mcu.WritePort((char *)VNB.c_str(), VNB.length());
        Serial_Mcu.WritePort(&mid, 1);
        Serial_Mcu.WritePort((char *)ENB.c_str(), ENB.length());
        Serial_Mcu.WritePort(&end, 1);
        usleep(100000);*/
    }





    if (true)
    {
        //开启线程函数
        //华为心跳和请求服务器配置信息
        pthread_create(&id->HWsocketTcp, NULL, HWsocketTcp, NULL);
        //中兴心跳和请求服务器配置信息
        pthread_create(&id->ZXsocketTcp, NULL, ZXsocketTcp, NULL);
        //OBD数据上传
        //pthread_create(&id->OBDsend, NULL, OBDsend, NULL);
        //BD数据上传
        pthread_create(&id->BDsend, NULL, BDsend, NULL);
        //蓝牙数据上传
        pthread_create(&id->BLUEsend, NULL, BLUEsend, NULL);
        //其他数据上传
        pthread_create(&id->OTHERsend, NULL, OTHERsend, NULL);
        //华为获取Ntrip差分数据
        pthread_create(&id->HWnTrip, NULL, HWnTrip, NULL);
        //中兴获取Ntrip差分数据
        //pthread_create(&id->ZXnTrip, NULL, ZXnTrip, NULL);
        //获取？？信息
        pthread_create(&id->getInfo, NULL, getInfo, NULL);
        //中兴AT指令
        //pthread_create(&id->ZXAT, NULL, ZXAT, NULL);

        //
        std::string ssid, key;
        //根据配置文件中WIFI开关关键字，开启或关闭WiFi
        if (atoi(config[28].c_str()) == 1)
            wifiopen = true;
        else{
        		wifiopen = false;
        }
        //wifi名称
        ssid = config[26];
        //wifi密码
        key = config[27];
        //如果配置文件中wifi开关为开即1，开启Wifi
        if (wifiopen)
        {
            char ap[200];
            sprintf(ap, "dhd_helper iface wlan0 ssid %s bgnmode bgn chan 3 amode wpawpa2psk emode tkipaes key %s",
                    ssid.c_str(), key.c_str());
            system("echo \"1\" > /proc/sys/net/ipv4/ip_forward");
            system("iptables -t nat -I POSTROUTING -o eth0 -j MASQUERADE");
            system("iptables -A FORWARD -s 192.168.5.1/24 -j ACCEPT");
            system("iptables -A FORWARD -d 192.168.5.1/24 -j ACCEPT");
            system("ifconfig wlan0 up 192.168.5.143");
            system("udhcpd /etc/udhcpd.conf &");
            system(ap);
            //system("dhd_helper iface wlan0 ssid tttb_ap hidden n bgnmode b chan 3 amode open emode none");
            system("dhd_helper iface wlan0 ssid ttt_b bgnmode bgn chan 3 amode wpawpa2psk emode tkipaes key 12345678");
            
            //输出wifi开启提示信息
        		std::cout << "Wifi is open!" << std::endl;
        }
        
        //开启linux心跳，提示单片机正常工作
        pthread_create(&id->irq, NULL, irq, NULL);
        
    }
    //以上为配置过程，到此全部配置完成
    //------------------------------------------------------------------------------------------------------------------------------------
    //************************************************************************************************************************************
    //************************************************************************************************************************************
    //************************************************************************************************************************************
    //************************************************************************************************************************************
    //************************************************************************************************************************************   
    //------------------------------------------------------------------------------------------------------------------------------------
    //以下为数据处理上传过程 
    //start working information
    std::cout << "Linux start working!" << std::endl;



	//将整理好的配置信息整体下发给STM32
	if(true)
	{
		//发送帧头
		Serial_Mcu.WritePort(start, 3);
		//逐行将数据发送给STM32
		for(int i=0;i<43;i++){
			Serial_Mcu.WritePort((char *)config[i].c_str(), config[i].length());
			Serial_Mcu.WritePort(&comma, 1);
		}
		//发送帧尾
		Serial_Mcu.WritePort(&end, 1);
		usleep(500000);	
        printf("\r\n\r\n\r\n\r\nConfig five has sended.\r\n\r\n\r\n\r\n");

	}








    //数据循环处理过程
    while (true)
    {
        //延时1s，保证所有配置都完成
        usleep(1); //利于系统稳定,必不可少,........这里是因为CAN_send会一直被清零

        //-----------DDP信息处理、输出显示----------------
        if (Serial_Mcu.DDP_Send)
        {
            //定义年月日时分秒
            char yy[2], MM[2], dd[2], hh[2], mm[2], ss[2];
            //定义时间字符串
            char time[] = {0};
            //将32发送的调度屏时间十六进制转换成十六进制的字符形式
            //年
            sprintf(yy, "%02X", Serial_Mcu.DDP[23]);
            //拼接到time
            strcpy(time, yy);
            //月
            sprintf(MM, "%02X", Serial_Mcu.DDP[24]);
            //拼接到time
            strcat(time, MM);
            //日
            sprintf(dd, "%02X", Serial_Mcu.DDP[25]);
            //拼接到time
            strcat(time, dd);
            //时
            sprintf(hh, "%02X", Serial_Mcu.DDP[0]);
            //拼接到time
            strcat(time, hh);
            //分
            sprintf(mm, "%02X", Serial_Mcu.DDP[1]);
            //拼接到time
            strcat(time, mm);
            //秒
            sprintf(ss, "%02X", Serial_Mcu.DDP[2]);
            //拼接到time
            strcat(time, ss);
            date = time;
            
            //将STM32发送的调度屏的数据发给DDP
            Serial_ddp.WritePort(Serial_Mcu.DDP, 31);
            
            //输出DDP数据
            std::cout <<"Serial_Mcu.DDP: "<< Serial_Mcu.DDP << std::endl;
            //输出时间
            std::cout << "date: " << date << std::endl;
            while(1);

            //调度屏的数据区清0
            bzero(Serial_Mcu.DDP, sizeof(Serial_Mcu.DDP));
            //更新完DDP信息后不再发送
            Serial_Mcu.DDP_Send = false;
        } 

        //------------------OBD数据处理--------------------
        if (Serial_OBD.CAN_Send && !Socket_is_Send)
        {
            float x;
            char y9[9], y6[6], y5[5], y4[4], y1[1];
            int z;

            if (OBDCnt > 9)
            {
                OBDCnt = 0;
                bzero(OBDPayloadSerial, sizeof(OBDPayloadSerial));
            }

            z = Serial_OBD.OBD[0][0] << 24 | Serial_OBD.OBD[0][1] << 16 | Serial_OBD.OBD[0][2] << 8 | Serial_OBD.OBD[0][3];
            x = float(z) / float(100);
            bzero(y6, 6);
            sprintf(y6, "%.2f,", x);
            strcpy(OBDPayloadSerial[OBDCnt], head9);
            strcat(OBDPayloadSerial[OBDCnt], y6);

            z = Serial_OBD.OBD[1][0] << 24 | Serial_OBD.OBD[1][1] << 16 | Serial_OBD.OBD[1][2] << 8 | Serial_OBD.OBD[1][3];
            x = float(z) / float(100);
            bzero(y9, 9);
            sprintf(y9, "%.2f,", x);
            strcat(OBDPayloadSerial[OBDCnt], head10);
            strcat(OBDPayloadSerial[OBDCnt], y9);

            z = Serial_OBD.OBD[7][0] << 24 | Serial_OBD.OBD[7][1] << 16 | Serial_OBD.OBD[7][2] << 8 | Serial_OBD.OBD[7][3];
            x = float(z) / float(100);
            bzero(y5, 5);
            sprintf(y5, "%.2f,", x);
            strcat(OBDPayloadSerial[OBDCnt], head12);
            strcat(OBDPayloadSerial[OBDCnt], y5);

            z = Serial_OBD.OBD[6][0] << 24 | Serial_OBD.OBD[6][1] << 16 | Serial_OBD.OBD[6][2] << 8 | Serial_OBD.OBD[6][3];
            x = float(z) / float(100);
            bzero(y5, 5);
            sprintf(y5, "%.2f,", x);
            strcat(OBDPayloadSerial[OBDCnt], head13);
            strcat(OBDPayloadSerial[OBDCnt], y5);

            z = Serial_OBD.OBD[5][0] << 24 | Serial_OBD.OBD[5][1] << 16 | Serial_OBD.OBD[5][2] << 8 | Serial_OBD.OBD[5][3];
            x = float(z) / float(100);
            bzero(y4, 4);
            sprintf(y4, "%d,", int(x));
            strcat(OBDPayloadSerial[OBDCnt], head14);
            strcat(OBDPayloadSerial[OBDCnt], y4);

            z = Serial_OBD.OBD[8][0] << 24 | Serial_OBD.OBD[8][1] << 16 | Serial_OBD.OBD[8][2] << 8 | Serial_OBD.OBD[8][3];
            x = float(z) / float(100);
            bzero(y4, 4);
            sprintf(y4, "%d,", static_cast<int>(x));
            strcat(OBDPayloadSerial[OBDCnt], head15);
            strcat(OBDPayloadSerial[OBDCnt], y4);

            z = Serial_OBD.OBD[4][0] << 24 | Serial_OBD.OBD[4][1] << 16 | Serial_OBD.OBD[4][2] << 8 | Serial_OBD.OBD[4][3];
            x = float(z) / float(100);
            bzero(y5, 5);
            sprintf(y5, "%.2f,", x);
            strcat(OBDPayloadSerial[OBDCnt], head17);
            strcat(OBDPayloadSerial[OBDCnt], y5);

            z = Serial_OBD.OBD[3][0] << 24 | Serial_OBD.OBD[3][1] << 16 | Serial_OBD.OBD[3][2] << 8 | Serial_OBD.OBD[3][3];
            x = float(z) / float(100);
            bzero(y4, 4);
            sprintf(y4, "%.1f,", x);
            strcat(OBDPayloadSerial[OBDCnt], head18);
            strcat(OBDPayloadSerial[OBDCnt], y4);

            z = Serial_OBD.OBD[18][0] << 24 | Serial_OBD.OBD[18][1] << 16 | Serial_OBD.OBD[18][2] << 8 | Serial_OBD.OBD[18][3];
            x = float(z) / float(100);
            bzero(y4, 4);
            sprintf(y4, "%d,", static_cast<int>(x));
            strcat(OBDPayloadSerial[OBDCnt], head19);
            strcat(OBDPayloadSerial[OBDCnt], y4);

            z = Serial_OBD.OBD[10][0] << 24 | Serial_OBD.OBD[10][1] << 16 | Serial_OBD.OBD[10][2] << 8 | Serial_OBD.OBD[10][3];
            x = float(z) / float(100);
            bzero(y5, 5);
            sprintf(y5, "%.1f,", x);
            strcat(OBDPayloadSerial[OBDCnt], head21);
            strcat(OBDPayloadSerial[OBDCnt], y5);

            z = Serial_OBD.OBD[12][0] << 24 | Serial_OBD.OBD[12][1] << 16 | Serial_OBD.OBD[12][2] << 8 | Serial_OBD.OBD[12][3];
            x = float(z) / float(100);
            bzero(y4, 4);
            sprintf(y4, "%d,", static_cast<int>(x));
            strcat(OBDPayloadSerial[OBDCnt], head23);
            strcat(OBDPayloadSerial[OBDCnt], y4);

            z = Serial_OBD.OBD[14][0] << 24 | Serial_OBD.OBD[14][1] << 16 | Serial_OBD.OBD[14][2] << 8 | Serial_OBD.OBD[14][3];
            if (z > 1)
                z = 2;
            sprintf(y1, "%d,", z);
            strcat(OBDPayloadSerial[OBDCnt], head24);
            strcat(OBDPayloadSerial[OBDCnt], y1);

            z = Serial_OBD.OBD[15][0] << 24 | Serial_OBD.OBD[15][1] << 16 | Serial_OBD.OBD[15][2] << 8 | Serial_OBD.OBD[15][3];
            if (z > 1)
                z = 2;
            sprintf(y1, "%d,", z);
            strcat(OBDPayloadSerial[OBDCnt], head25);
            strcat(OBDPayloadSerial[OBDCnt], y1);

            z = Serial_OBD.OBD[27][0] << 24 | Serial_OBD.OBD[27][1] << 16 | Serial_OBD.OBD[27][2] << 8 | Serial_OBD.OBD[27][3];
            x = float(z) / float(100);
            bzero(y6, 6);
            sprintf(y6, "%d,", int(x));
            strcat(OBDPayloadSerial[OBDCnt], head29);
            strcat(OBDPayloadSerial[OBDCnt], y6);

            z = Serial_OBD.OBD[26][0] << 24 | Serial_OBD.OBD[26][1] << 16 | Serial_OBD.OBD[26][2] << 8 | Serial_OBD.OBD[26][3];
            x = float(z) / float(100);
            bzero(y5, 5);
            sprintf(y5, "%d,", int(x));
            strcat(OBDPayloadSerial[OBDCnt], head34);
            strcat(OBDPayloadSerial[OBDCnt], y5);

            z = Serial_OBD.OBD[25][0] << 24 | Serial_OBD.OBD[25][1] << 16 | Serial_OBD.OBD[25][2] << 8 | Serial_OBD.OBD[25][3];
            x = float(z) / float(100);
            bzero(y6, 6);
            sprintf(y6, "%.2f,", x);
            strcat(OBDPayloadSerial[OBDCnt], head35);
            strcat(OBDPayloadSerial[OBDCnt], y6);

            //****CRC
            //strcat(OBDPayloadSerial[OBDCnt], head41);
            OBDCnt++;
            Serial_OBD.CAN_Send = 0;
        } 

        
        //-----------------BD数据处理-------------------
        //北斗数据每隔1s获取一条，处理一次
        if (Serial_Mcu.BD_Send && !Socket_is_Send)
        {
            uint64_t zero = 0;
            uint64_t i = 0;
            // char buf;
            //获取北斗数据长度
            uint16_t len = Serial_Mcu.BD_DatLen;
            
            //北斗数据满10条，计数器清0，缓存区清0
            if (BDCnt > 9)
            {
                BDCnt = 0;
                bzero(BDPayloadSerial, sizeof(BDPayloadSerial));
            }
            
            //读取STM32发送的BD数据到缓冲区
            for (i = 0; i < len; i++)
            {
                // buf = Serial_Mcu.BD[i];
                // if (Serial_Mcu.BD[i + zero] == 0x00)
                // {
                //     zero++;
                //     i--;
                // }
                // else
                
                BDPayloadSerial[BDCnt][i] = Serial_Mcu.BD[i + zero];
            }
            //加上字符串结束字符'\0'
            BDPayloadSerial[BDCnt][i] = '\0';
            //printf("\r\n");
            //向屏幕输出北斗数据
            puts(BDPayloadSerial[BDCnt]);
            //printf("\r\n");
             
            BDCnt++;
            //北斗数据接收缓冲区清0
            bzero(Serial_Mcu.BD, sizeof(Serial_Mcu.BD));
            //更新完BD信息后不再发送
            Serial_Mcu.BD_Send = false;
        }
            

            //gengxin   BDdate   
             /*
            for(int i=0;i<15;i++)
            {
                BDdate[i]=BDPayloadSerial[BDCnt][i+52];
            }
            printf("BDdate:%s",BDdate);

           
            for(int i=0;i<100;i++)
            {
                if(BDPayloadSerial[BDCnt][i]=='S'&& BDPayloadSerial[BDCnt][i+1]=='D'&& BDPayloadSerial[BDCnt][i+2]=='T')
                {
                    printf("nianyueriweizhi:\r\n");
                    printf("%d\n",i);
                }
            }*/


        //-----------------Speed数据处理-------------------
        //数据每隔1s获取一条，处理一次
        if (Serial_Mcu.Speed_Send && !Socket_is_Send)
        {
            uint64_t zero = 0;
            uint64_t i = 0;
            // char buf;
            //获取数据长度
            uint16_t len = Serial_Mcu.Speed_DatLen;
            
            //数据满10条，计数器清0，缓存区清0
            if (SpeedCnt > 9)
            {
                SpeedCnt = 0;
                bzero(SpeedPayloadSerial, sizeof(SpeedPayloadSerial));
            }
            
            //读取STM32发送的BD数据到缓冲区
            for (i = 0; i < len; i++)
            {
                // buf = Serial_Mcu.BD[i];
                // if (Serial_Mcu.BD[i + zero] == 0x00)
                // {
                //     zero++;
                //     i--;
                // }
                // else
                
                SpeedPayloadSerial[SpeedCnt][i] = Serial_Mcu.Speed[i + zero];
            }
            //加上字符串结束字符'\0'
            SpeedPayloadSerial[SpeedCnt][i] = '\0';
            //printf("\r\n");
            //向屏幕输出北斗数据
            puts(SpeedPayloadSerial[SpeedCnt]);
            //printf("\r\n");
             
            SpeedCnt++;
            //北斗数据接收缓冲区清0
            bzero(Serial_Mcu.Speed, sizeof(Serial_Mcu.Speed));
            //更新完BD信息后不再发送
            Serial_Mcu.Speed_Send = false;
        }














        //---------------------blue数据处理----------------------
        if (Serial_Mcu.BLUE_Send && !Socket_is_Send)
        {
            //获取蓝牙数据长度
            uint16_t len = Serial_Mcu.BLUE_DatLen;
            
            //蓝牙数据满10条，计数器清0，缓存区清0
            if (BLUECnt > 9)
            {
                BLUECnt = 0;
                bzero(BLUEPayloadSerial, sizeof(BLUEPayloadSerial));
            }
            
            //读取并打印蓝牙数据
            std::cout << "BLUEPayloadSerial: ";
            for (uint16_t i = 0; i < len; i++)
            {
                BLUEPayloadSerial[BLUECnt][i] = Serial_Mcu.BLUE[i];   	
            }
            std::cout << BLUEPayloadSerial[BDCnt]<< std::endl;
            
            //蓝牙数据包计数器加1
            BLUECnt++;
            //蓝牙数据接收缓冲区清0
            bzero(Serial_Mcu.BLUE, sizeof(Serial_Mcu.BLUE));
            //更新完蓝牙信息后不再发送
            Serial_Mcu.BLUE_Send = 0;
        } 
        
        
        
        while(!Serial_Mcu.config_received32)
        {
            std::cout<<"requestreceivedfrom32:"<<Serial_Mcu.requestreceivedfrom32<<std::endl;
            std::cout<<"config_received32:"<<Serial_Mcu.config_received32<<std::endl;
            //if(true){
            if(Serial_Mcu.requestreceivedfrom32==true){
                //发送帧头
				Serial_Mcu.WritePort(start, 3);
				//逐行将数据发送给STM32
				for(int i=0;i<43;i++){
					Serial_Mcu.WritePort((char *)config[i].c_str(), config[i].length());
					Serial_Mcu.WritePort(&comma, 1);
				}
				//发送帧尾
				Serial_Mcu.WritePort(&end, 1);
                printf("\r\n\r\n\r\n\r\nConfig five has sended.\r\n\r\n\r\n\r\n");
                usleep(1000000);	
            }
            usleep(1000000);    
        }	





        //---------------------PC端上位机更新配置文件config-------------------------
        if (Serial_Com.sendConf || Serial_Com.sendSconf)
        {
            char bg[1] = {0x0A}, ed[1] = {0x0D}, doo[1] = {','};
            std::cout << "send config" << std::endl;
            for (int i = 0; i < (confCnt - 1); i++)
            {

                if (config[i].length() >= 1)
                {
                    if (i == 20)
                    {
                        Serial_Com.WritePort(bg, 1);
                        Serial_Com.WritePort((char *)config[confCnt - 1].c_str(), config[confCnt - 1].length());
                        Serial_Com.WritePort(ed, 1);
                        std::cout << config[i] << std::endl;
                        usleep(5000);
                    }
                    else
                    {
                        Serial_Com.WritePort(bg, 1);
                        Serial_Com.WritePort((char *)config[i].c_str(), config[i].length());
                        Serial_Com.WritePort(ed, 1);
                        std::cout << config[i] << std::endl;
                        usleep(5000);
                    }
                }
            }
            struct ifreq ifr;
            struct ifconf ifc;
            char buf[2048];
            //int success = 0;

            int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (sock == -1)
            {
                printf("socket error\n");
            }

            ifc.ifc_len = sizeof(buf);
            ifc.ifc_buf = buf;
            if (ioctl(sock, SIOCGIFCONF, &ifc) == -1)
            {
                printf("ioctl error\n");
            }

            struct ifreq *it = ifc.ifc_req;
            const struct ifreq *const end = it + (ifc.ifc_len / sizeof(struct ifreq));
            char szMac[64];
            int count = 0;
            for (; it != end; ++it)
            {
                strcpy(ifr.ifr_name, it->ifr_name);
                if (ioctl(sock, SIOCGIFFLAGS, &ifr) == 0)
                {
                    if (!(ifr.ifr_flags & IFF_LOOPBACK))
                    { // don't count loopback
                        if (ioctl(sock, SIOCGIFHWADDR, &ifr) == 0)
                        {
                            count++;
                            unsigned char *ptr;
                            ptr = (unsigned char *)&ifr.ifr_ifru.ifru_hwaddr.sa_data[0];
                            snprintf(szMac, 64, "%02X:%02X:%02X:%02X:%02X:%02X", *ptr, *(ptr + 1), *(ptr + 2), *(ptr + 3), *(ptr + 4), *(ptr + 5));
                            //printf("%d,Interface name : %s , Mac address : %s \n", count, ifr.ifr_name, szMac);
                            Serial_Com.WritePort(bg, 1);
                            Serial_Com.WritePort(ifr.ifr_name, strlen(ifr.ifr_name));
                            Serial_Com.WritePort(doo, 1);
                            Serial_Com.WritePort(szMac, strlen(szMac));
                            Serial_Com.WritePort(ed, 1);
                            usleep(5000);
                        }
                    }OBDCnt = 0;
                }
                else
                {
                    printf("get mac info error\n");
                }
            }
            Serial_Com.sendConf = false;
            Serial_Com.sendSconf = false;
        } //if sendConf true


				//--------------------ADC温度和电量信息处理---------------------
        if (Serial_Mcu.ADC_Send)
        {
            //定义位置索引变量
            std::string::size_type index;
            //读取ADC数据
            std::string adcBuf = Serial_Mcu.ADC;
            //查找','所在位置
            index = adcBuf.find(',');
            //获取电量
            BAP = adcBuf.substr(0, index);
            //获取温度
            TEM = adcBuf.substr(index + 1);
            //获取完成继续接收STM32发送数据
            Serial_Mcu.ADC_Send = false;
        }
        
        
        
        

        //-----------------华为和中兴上传通道切换---------------------
        if (true)
        {
						//默认使用中兴专网模块进行数据上传
            if (ZXcilentfd > 0 && ZXconnected)
            {
                //上传数据句柄为中兴联网句柄
                cilentfd = ZXcilentfd;
                
                if (!whichCilent)
                		//切出中兴次数计数器
                    UCNCnt++;
                whichCilent = 1;
            }
            //当中兴联网失败时切换到华为模块进行数据上传
            else if (HWcilentfd > 0 && HWconnected) 
            {
                //上传数据句柄为中兴联网句柄
                cilentfd = HWcilentfd;
                if (whichCilent)
                		//切入华为次数计数器
                    DCNCnt++;
                whichCilent = 0;
            }
            //都连不上网络时上传句柄为-1
            else
            {
                cilentfd = -1;
            }
        }
    } 


		//系统关闭时，关闭串口
    Serial_OBD.ClosePort();
    Serial_ddp.ClosePort();
    Serial_Mcu.ClosePort();
    Serial_Com.ClosePort();
    //给出系统退出信息
    std::cout << "Goodbye !" << std::endl;
    return 0;

} //main()
