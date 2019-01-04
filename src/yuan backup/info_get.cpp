#include <stdio.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <syslog.h>
#include <errno.h>
#include "include/serial.h"
#include <pthread.h>
#include <string>
#include <vector>
#include <iostream>
#include <stdint.h>
#include <sstream>
#include "include/toollib.h"
#include "include/network_card.h"
#include "include/msg_queue.h"
#define MAX_RECV_SIZE     1024
#define CMD_AT_OK         "OK"
/*- AT CMD */
#define CMD_AT_SIG_STRENGTH     "AT+CINR\r" 
#define CMD_AT_SIG_QUALITY         "AT+CSQ\r" 
#define CMD_AT_SIG_GETLINE          "AT^NDISDUP=1,1\r" 
#define CMD_AT_SIG_ENABLE           "AT+CFUN=1\r" 
#define CMD_AT_SIG_ENABLE_CG    "AT+CGACT=1,1\r" 
#define CMD_AT_SIG_ENABLE_ZG    "AT+ZGACT=1,1\r" 


using namespace std;


typedef struct 
{
int fd_huawei;
int fd_zhongx;
}sop;

/*serial ops */
sop ser ;


static int special_network_ip(string str_ip)
{
	string net_name="eth2"; 
	if (get_ip(net_name,str_ip)<0)
	{
		return -1;	
	}	
	if(str_ip=="")
		return -1;		
	return 0;
}




static void splitString(const std::string& s, std::vector<std::string>& v, const std::string& c)
{
	v.clear();
	std::string::size_type pos1, pos2;
	pos2 = s.find(c);
	pos1 = 0;
	while(std::string::npos != pos2)
	{
		v.push_back(s.substr(pos1, pos2-pos1));
		pos1 = pos2 + c.size();
		pos2 = s.find(c, pos1);
	}
	if(pos1 != s.length())
		v.push_back(s.substr(pos1));
}


static char*  __check_at_exec_ok(const char * result)
{
	return (char *)strstr(result,CMD_AT_OK);
}


static int __at_send_or_recv(int serial_fd, const char * send_buf,char *recv_buf)
{
	int len,i=0;
	tcflush(serial_fd,TCIOFLUSH);
	len = UART0_Send(serial_fd,(char*)send_buf,strlen(send_buf));  
	if (len<0)
	{
		printf(" send data failed!\n"); 
		return -1;			
	}
	sleep(1);
	len = UART0_Recv(serial_fd, recv_buf,MAX_RECV_SIZE-1);  
	if(len > 0 && len< MAX_RECV_SIZE)  
	{  
		recv_buf[len] = '\0';  
	//	printf("receive data is %s\n",recv_buf);    
	}  
	else  
	{  
		printf("cannot receive data\n"); 
		return -1;			
	}

	if (!__check_at_exec_ok(recv_buf))
	{
		printf("check is error \n");
		return -1;
	}
	return len;	
}


static int __special_network_strength(float *result)
{
	char recv_buf[MAX_RECV_SIZE]={0};
	char list_no[255]={0};
	float m[6+2]={0};
	std::vector<std::string> l;
	string req= CMD_AT_SIG_STRENGTH;
	if (__at_send_or_recv(ser.fd_zhongx,req.c_str(),recv_buf)<0)
	{
		printf("信号强度获取失败\n");
		return -1;
	}
	req= "+ZSINR:";
	string ops(recv_buf);
	uint32_t pose_start=ops.find(req)+req.size();
	uint32_t pose_end=ops.find_last_of("\n");
	memcpy(list_no,(char*)(ops.c_str()+pose_start),pose_end-pose_start);
	string  tmp(list_no);
	splitString(tmp,l,",");
	if (l.size()!=8)
	{
		return -1;
	}
	string str_to_fl;
	string spl=".";
	for (uint32_t i=0;i<l.size();i++)
	{
		if (i%2)
		{   str_to_fl.append(l[i-1]);
			str_to_fl.append(spl);
			str_to_fl.append(l[i]);
			m[i-1]=(float)atof(str_to_fl.c_str());
			str_to_fl.clear();
		}
	}

	m[5]=__MAX(m[0],m[1]);
	m[6]=__MAX(m[2],m[3]);
	*result = m[5]+m[6];
	return 0;
}


static int  __special_network_quality(float *result)
{
	char recv_buf[MAX_RECV_SIZE]={0};
	string req= CMD_AT_SIG_QUALITY;
	char list_no[255]={0};
	float m;
	float ret;
	std::vector<std::string> l;
	if (__at_send_or_recv(ser.fd_zhongx,req.c_str(),recv_buf)<0)
	{
		printf("信号质量获取失败\n");
		return -1;
	}
	req= "+CSQ: "; /*note  space*/
	string ops(recv_buf);	
	uint32_t pose_start=ops.find(req)+req.size();
	string str_ops_end(ops.c_str()+pose_start);
	uint32_t pose_end=pose_start +str_ops_end.find_first_of("\n");
	memcpy(list_no,(char*)(ops.c_str()+pose_start),pose_end-pose_start);
	string  tmp(list_no);
	splitString(tmp,l,",");
	__string_to_float(l[0],&m);
	m = m - 100;
	if(m == 99) {
		ret= 0xff;
	} else {
		ret = -(m - 141);
	}	
	*result = ret;
	return 0;
}


static int  network_online()
{
    char recv_buf[MAX_RECV_SIZE]={0};
	string req;
	uint8_t status[2]={0};
	uint32_t loop_count=10;
    while(loop_count--)
	{
		req= CMD_AT_SIG_GETLINE;
	  if (__at_send_or_recv(ser.fd_huawei,req.c_str(),recv_buf)<0)
	  { sleep(1);
		continue;
	  }
	    status[0]=1;
		system("udhcpc -i usb0");
		break;     		
	}
	
	if(!status[0])
	{
		if(!system("udhcpc -i usb0"))
			status[0]=1;
	}
	else
		printf("公网网卡上线失败！\n");
	
	loop_count=10;
	 while(loop_count--)
	 {
		 req=CMD_AT_SIG_ENABLE;
		  if (__at_send_or_recv(ser.fd_zhongx,req.c_str(),recv_buf)<0)
		  {
			  sleep(1);
			  continue;
	      }
		 req= CMD_AT_SIG_ENABLE_CG;
		   if (__at_send_or_recv(ser.fd_zhongx,req.c_str(),recv_buf)<0)
			{
			  sleep(1);
			  continue;
	      }
		 req= CMD_AT_SIG_ENABLE_ZG;
		   if (__at_send_or_recv(ser.fd_zhongx,req.c_str(),recv_buf)<0)
			 {
			  sleep(1);
			  continue;
	      }
	    status[1]=1;	 
		system("udhcpc -i eth2");
		break;
		 
	 }
	if (!(status[1] || status[0]))
	{
		return -1;
	}
	return 0;
}



/** 
 * @brief  Initialize the network configuration and get the gateway
 * @param void
 *
 * @return <0==failed   0==ok
 *     
 */

static int init_all_serial()
{
	int err;
	ser.fd_huawei =UART0_Open(ser.fd_huawei,(char*)"/dev/ttyUSB0");

	if (ser.fd_huawei<0)
	{
		printf("serial init failed ,check serial port is used\n");
		return  err;
	}
	err = UART0_Init(ser.fd_huawei,115200,0,8,1,'N');  
	if (err<0)
	{
		printf("fd_huawei serial init failed  in set\n");
		return  err;

	}
	ser.fd_zhongx =UART0_Open(ser.fd_zhongx,(char *)"/dev/ttyUSB5");

	if (ser.fd_zhongx<0)
	{
		printf("fd_zhongx serial init failed ,check serial port is used\n");
		return  err;
	}
	err = UART0_Init(ser.fd_zhongx,115200,0,8,1,'N');  

	if (err<0)
	{
		printf("fd_zhongx serial init failed  in set\n");
		return  err;
	}
return 0;
}



/** 
 * @brief  Deal and get network card info 
 * @param 
 *
 * @return void
 *     
 */
static int sigdaq(struct _msgbuf *message)
{
	message->mtype = SIGDAQ;
//again:
   
	if (__special_network_strength(&message->si)<0)
	{
		//goto again;
	}
	if (__special_network_quality(&message->sq)<0)
	{
		//goto again;
	}
	if (special_network_ip(message->ip)<0)
	{
		memset(message->ip,0,sizeof(message->ip));
	}
	return 0;
	
}

/** 
 * @brief Init net_card
 * @param 
 *
 * @return <0==failed   0==ok
 *     
 */
int init_net_card()
{
	key_t key;
    int msgid = 0;
    struct _msgbuf message;
	if (init_all_serial()<0)
	{
		printf("串口初始化失败\n");
	   goto END;
	}
	
	if (network_online()<0)
	{
		printf("所有网卡上线失败\n");
		goto EXIT;
	}
	
	
	if((key = ftok("/tmp", 'z')) < 0)
	{
		perror("ftok");
		goto EXIT;
	}
	if((msgid = msgget(key, IPC_CREAT|0666)) < 0)
	{
		perror("msgget");
		goto EXIT;
	}
	
	while(1)
	{
		memset(&message,0,sizeof(struct _msgbuf));
		sigdaq(&message);
		if(msgsnd(msgid, &message, sizeof(struct _msgbuf)- sizeof(long),IPC_NOWAIT)<0)
		{
			sleep(1);
		}
		sleep(2);
	}

	msgctl(msgid, IPC_RMID, NULL);
	return 0;
EXIT:
    UART0_Close(ser.fd_huawei);
	UART0_Close(ser.fd_zhongx);	
END:
	return -1;
}

#if 0
int main (void)
{
	
	init_net_card();
	
	return 0;
	
#endif