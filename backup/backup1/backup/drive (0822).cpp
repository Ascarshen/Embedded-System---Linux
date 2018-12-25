/*
 * drive.cpp
 *
 *  Created on: Mar 19, 2018
 *      Author: Wangcui
 *      Lidar - Q600
 */
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

//报警音:*************************************************************
char Voice_begin[2] = {0x24, 0xa2};
char Voicelength[1];
char Voice_end[1] = {0x0a};
char YUEJIE[] = {0xC4, 0xFA, 0xD2, 0xD1, 0xD4, 0xBD, 0xBD, 0xE7};                                                //"您已越界"
char JINRU[] = {0xC4, 0xFA, 0xD2, 0xD1, 0xBD, 0xF8, 0xC8, 0xEB, 0xCE, 0xA5, 0xB9, 0xE6, 0xC7, 0xF8, 0xD3, 0xF2}; //"您已进入违规区域"
//前方有停止线，请减速
char STOP[] = {0xC7, 0xB0, 0xB7, 0xBD, 0xD3, 0xD0, 0xCD, 0xA3, 0xD6, 0xB9, 0xCF, 0xDF, 0xA3, 0xAC, 0xC7, 0xEB, 0xBC, 0xF5, 0xCB, 0xD9};
//违规进入电子围栏区域
char WEILAN[] = {0xCE, 0xA5, 0xB9, 0xE6, 0xBD, 0xF8, 0xC8, 0xEB, 0xB5, 0xE7, 0xD7, 0xD3, 0xCE, 0xA7, 0xC0, 0xB8, 0xC7, 0xF8, 0xD3, 0xF2};
char CHAOSU[] = {0xC4, 0xFA, 0xD2, 0xD1, 0xB3, 0xAC, 0xCB, 0xD9};                        //"您已超速"
char YURE[] = {0xB3, 0xB5, 0xC1, 0xBE, 0xD4, 0xA4, 0xC8, 0xC8};                          //"车辆预热"
char PILAO[] = {0xC4, 0xFA, 0xD2, 0xD1, 0xC6, 0xA3, 0xC0, 0xCD, 0xBC, 0xDD, 0xCA, 0xBB}; //"您已疲劳驾驶"
char CHAOSHI[] = {0xB3, 0xAC, 0xCA, 0xB1, 0xCD, 0xA3, 0xB3, 0xB5};                       //"超时停车"

//OBD发送ID********************************************************
int OBDCnt = 0;
static char OBDPayloadSerial[10][320];
static char OBDpayload[300], OBDpayload_h2[40];
//   head1[2] = {'#','#'};//采用这种形式
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

//蓝牙
int BLUECnt = 0;
static char BLUEPayloadSerial[10][750];
static char BLUEpayload[750], BLUEpayload_h2[40];
std::string date = "180000000000";
static char BDdate[15]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,'\0'};
//北斗
int BDCnt = 0;
static char BDPayloadSerial[10][600];
char BDpayload[600];

//其他

std::string BAP = "0.0", TEM = "0.0";
bool ZX_NTgps = false, HW_NTgps = false;

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
        
        
            //HW huoqupeizhi
            char socket_configure[50];
            //char Crc1[2];
            //set the first five characters 0
            bzero(socketCnt, 5);
            //splice the socket_cnt into string 
            sprintf(socketCnt, "%05d", socket_cnt);
            //splice the frame head 
            strcpy(socket_configure, "##00");
            //splice the socketCnt
            strcat(socket_configure, socketCnt);
            //splice the the identification field
            strcat(socket_configure, ",GGDC,SSN");
            //splice the SSN code
            strcat(socket_configure, SSN.c_str());
            //splice the comma separator
            //strcat(socket_configure, ",");
            strcat(socket_configure, ",TYPGGDC");

            strcat(socket_configure, ",VNB");
            //splice the SSN code
            strcat(socket_configure, VNB.c_str());
            //splice the comma separator

            //strcat(socket_configure, ",ENB");
            //splice the SSN code
            //strcat(socket_configure, ENB.c_str());
            //splice the comma separator
            //strcat(socket_configure, ",");
            //strcat(socket_configure, ",SDT");
            //strcat(socket_configure, BDdate);
            strcat(socket_configure, ",");

            char socket_configure_CRC = '#'; //CRC
            for (uint16_t k = 1; k < strlen(socket_configure); k++)
            {
                socket_configure_CRC ^= socket_configure[k];
            }
            sprintf(Crc, "%02X", socket_configure_CRC);
            strcat(socket_configure, Crc); //CRC
            strcat(socket_configure, "@@");

            //zero clearing the receive buffer
            bzero(socket_recvbuf, 1024);
            //send the configure request information
            send(HWcilentfd, socket_configure, strlen(socket_configure), 0);
            socket_cnt++;
            //output information
            printf("HW configure request sended\n");
            printf("HW configure request:%s\n", socket_configure);
            //receive the return message
            recbyte = read(HWcilentfd, socket_recvbuf, 1024);
            if (recbyte == -1)
            {
                printf("read service data fail !\n");
            }
            else if (socket_recvbuf[0] == '#' && socket_recvbuf[2] == '9')
            { 
                printf("HW configure request information received\n");
                std::cout << "HW configure recvbuf: " << socket_recvbuf << std::endl;
            } 
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
        strcat(OTHERpayload, BAP.c_str()); //拼接BAP

        strcat(OTHERpayload, ",TEM");
        strcat(OTHERpayload, TEM.c_str()); //拼接TEM

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

/*
    Main函数
*/
int main(int argc, char **argv)
{

    uint8_t confCnt = 0;
    system("ulimit -s 102400");
    std::fstream Conf;

    //Open config.cng file
    Conf.open("/opt/config.conf", std::ios::in); //|std::ios::out|std::ios::binary);
    //Open successfull
    if (Conf.is_open())
    {
        char buf[50];
        while (!Conf.eof())
        {
            Conf.getline(buf, 50);
            config.push_back(buf);
            printf("%d: ", confCnt);
            //std::cout << config[confCnt] << std::endl;
            confCnt++;
        }
        Conf.close();
    }
    else
    {
        printf("%s", "Open config.cnf file error!");
    }

    //Open version.conf file
    Conf.open("/opt/version.conf", std::ios::in); //|std::ios::out|std::ios::binary);
    //Open successfull
    if (Conf.is_open())
    {
        char buf[50];
        Conf.getline(buf, 50);
        config.push_back(buf);
        std::cout << config[confCnt] << std::endl;
        confCnt++;
        Conf.close();
    }
    //Open failure
    else
    {
        printf("%s", "Open version.conf file error!");
    }

    //Open SSN.conf file
    Conf.open("/opt/SSN.conf", std::ios::in); //|std::ios::out|std::ios::binary);
    //Open successfull
    if (Conf.is_open())
    {
        char buf[50];
        Conf.getline(buf, 50);
        config.push_back(buf);
        std::cout << config[confCnt] << std::endl;
        confCnt++;
        Conf.close();
    }
    //Open failure
    else
    {
        printf("%s", "Open SSN.conf file error!");
    }

    //++++++add judge for system initialization state++++++
    if (true)
    {
        std::string confighw = "", configzx = "";
        confighw = config[3] + ":" + config[4];
        configzx = config[7] + ":" + config[8];
        // confighw.append((config[3].append(":")).append(config[4]));
        // configzx.append((config[7].append(":")).append(config[8]));
        char *src_hw = (char *)confighw.c_str();
        char *src_zx = (char *)configzx.c_str();
        char *buf_hw, *buf_zx;
        base64_encode(src_hw, &buf_hw);
        base64_encode(src_zx, &buf_zx);
        Ntrip_hwQ64 = buf_hw;
        Ntrip_zxQ64 = buf_zx;
        std::string::size_type index;
        index = Ntrip_hwQ64.find_last_of("=");
        if (index != std::string::npos)
            Ntrip_hwQ64 = Ntrip_hwQ64.substr(0, index);
        index = Ntrip_zxQ64.find_last_of("=");
        if (index != std::string::npos)
            Ntrip_zxQ64 = Ntrip_zxQ64.substr(0, index);
        printf("base64 encode HW:%s\n", Ntrip_hwQ64.c_str());
        printf("base64 encode ZX:%s\n", Ntrip_zxQ64.c_str());
        free(buf_hw);
        free(buf_zx);
        buf_hw = NULL;
        buf_zx = NULL;

        //SSN init
        SSN = config[confCnt - 1];
        //VNB init
        VNB = config[16];
        //ENB init
        ENB = config[17];
    }
    //delay for a while
    sleep(1);

    //

    if (true)
    {
        char start[3] = {0x0a, 'A', 'B'};
        char end = 0x0d, mid = ':';
        //UART4:115200, A3352<--->OBD
        if ((Serial_OBD.OpenPort(4, 115200, '8', '1', 'N')) < 0)
        {
            printf("open Serial_OBD is failed\n");
            return 0;
        }
        //UART3:19200, A3352<--->DDP
        if ((Serial_ddp.OpenPort(3, 19200, '8', '1', 'N')) < 0)
        {
            printf("open Serial_ddp is failed\n");
            return 0;
        }
        //UART1:115200, A3352<--->MCU
        if ((Serial_Mcu.OpenPort(1, 115200, '8', '1', 'N')) < 0)
        {
            printf("open Serial_Mcu is failed\n");
            return 0;
        }
        //UART2:38400, A3352<--->1 to 4(RS232)
        if ((Serial_Com.OpenPort(2, 38400, '8', '1', 'N')) < 0)
        {
            printf("open Serial_Com is failed\n");
            return 0;
        }

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

        if (config[25] != "1") //**blue open ?
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
        if (index > 0) //**ip port
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
        usleep(100000);
    }

    if (true)
    {
        pthread_create(&id->HWsocketTcp, NULL, HWsocketTcp, NULL);
        //pthread_create(&id->ZXsocketTcp, NULL, ZXsocketTcp, NULL);
        //pthread_create(&id->OBDsend, NULL, OBDsend, NULL);
        pthread_create(&id->BDsend, NULL, BDsend, NULL);
        //pthread_create(&id->BLUEsend, NULL, BLUEsend, NULL);
        //pthread_create(&id->OTHERsend, NULL, OTHERsend, NULL);
        pthread_create(&id->HWnTrip, NULL, HWnTrip, NULL);
        //pthread_create(&id->ZXnTrip, NULL, ZXnTrip, NULL);
        pthread_create(&id->getInfo, NULL, getInfo, NULL);
        //pthread_create(&id->ZXAT, NULL, ZXAT, NULL);

        std::string ssid, key;
        if (atoi(config[28].c_str()) == 1)
            wifiopen = true;
        ssid = config[26];
        key = config[27];
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
        }
        pthread_create(&id->irq, NULL, irq, NULL);
    }
    //start working information
    std::cout << "start working!" << std::endl;

    //information processing
    while (true)
    {
        //delay for a while
        usleep(1); //利于系统稳定,必不可少,........这里是因为CAN_send会一直被清零

        /*DDP*/
        if (Serial_Mcu.DDP_Send)
        {
            char yy[2], MM[2], dd[2], hh[2], mm[2], ss[2];
            char time[] = {0};
            sprintf(yy, "%02X", Serial_Mcu.DDP[23]);
            strcpy(time, yy);
            sprintf(MM, "%02X", Serial_Mcu.DDP[24]);
            strcat(time, MM);
            sprintf(dd, "%02X", Serial_Mcu.DDP[25]);
            strcat(time, dd);
            sprintf(hh, "%02X", Serial_Mcu.DDP[0]);
            strcat(time, hh);
            sprintf(mm, "%02X", Serial_Mcu.DDP[1]);
            strcat(time, mm);
            sprintf(ss, "%02X", Serial_Mcu.DDP[2]);
            strcat(time, ss);
            date = time;
            //send information to DDP
            Serial_ddp.WritePort(Serial_Mcu.DDP, 31);
            // std::cout <<"Serial_Mcu.DDP: "<< Serial_Mcu.DDP << std::endl;
            // std::cout << "date: " << date << std::endl;

            //
            bzero(Serial_Mcu.DDP, sizeof(Serial_Mcu.DDP));
            //send one time
            Serial_Mcu.DDP_Send = false;
        } //if DDP_Send true

        /*OBD*/
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
        } //if OBD_Send true

        /*BD*/
        //std::cout << " Serial_Mcu.BD_Send , Socket_is_Send " << Serial_Mcu.BD_Send << Socket_is_Send << std::endl;
        if (Serial_Mcu.BD_Send && !Socket_is_Send)
        {
            uint64_t zero = 0;
            uint64_t i = 0;
            // char buf;
            uint16_t len = Serial_Mcu.BD_DatLen;
            if (BDCnt > 9)
            {
                BDCnt = 0;
                bzero(BDPayloadSerial, sizeof(BDPayloadSerial));
            }
            
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
            BDPayloadSerial[BDCnt][i] = '\0';
            //printf("\r\n");
            puts(BDPayloadSerial[BDCnt]);
            //printf("\r\n");

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
             
            BDCnt++;
            bzero(Serial_Mcu.BD, sizeof(Serial_Mcu.BD));
            Serial_Mcu.BD_Send = false;
        } //if BD_Send true

        /*blue*/
        if (Serial_Mcu.BLUE_Send && !Socket_is_Send)
        {
            uint16_t len = Serial_Mcu.BLUE_DatLen;
            if (BLUECnt > 9)
            {
                BLUECnt = 0;
                bzero(BLUEPayloadSerial, sizeof(BLUEPayloadSerial));
            }
            //std::cout << "BLUEPayloadSerial: ";
            for (uint16_t i = 0; i < len; i++)
            {
                BLUEPayloadSerial[BLUECnt][i] = Serial_Mcu.BLUE[i];
               //std::cout << BLUEPayloadSerial[BDCnt][i];
            }
            //std::cout << std::endl;
            BLUECnt++;
            bzero(Serial_Mcu.BLUE, sizeof(Serial_Mcu.BLUE));
            Serial_Mcu.BLUE_Send = 0;
        } //if BLUE_Send true

        /*config*/
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

        if (Serial_Mcu.ADC_Send)
        {
            std::string::size_type index;
            std::string adcBuf = Serial_Mcu.ADC;
            index = adcBuf.find(',');
            BAP = adcBuf.substr(0, index);
            TEM = adcBuf.substr(index + 1);
            Serial_Mcu.ADC_Send = false;
        }

        /*HW ZX fd change*/
        if (true)
        {

            if (ZXcilentfd > 0 && ZXconnected)
            {
                cilentfd = ZXcilentfd;
                if (!whichCilent)
                    UCNCnt++;
                whichCilent = 1;
            }
            else if (HWcilentfd > 0 && HWconnected) //**************************
            {
                cilentfd = HWcilentfd;
                if (whichCilent)
                    DCNCnt++;
                whichCilent = 0;
            }
            else
            {
                cilentfd = -1;
            }
        }
    } //while true

    Serial_OBD.ClosePort();
    Serial_ddp.ClosePort();
    Serial_Mcu.ClosePort();
    Serial_Com.ClosePort();
    std::cout << "Goodbye !" << std::endl;
    return 0;

} //main()
