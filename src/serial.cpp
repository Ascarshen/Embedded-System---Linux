#include "include/Serial.h"

#define MAXLEN 200		//定义最大长度
#define SerialPort "/dev/ttyO%d"			//定义串口
//***************************CSerial******************************************
//构造函数
CSerial::CSerial()
{
	this->m_fd = 0;
	this->m_DatLen = 0;
	this->DatBuf[1023] = '\n';
	this->m_ExitThreadFlag = 0;
	this->epfd = epoll_create(6);
	this->OK = false;								//能否处理
	this->CAN_Send = false; 				//OBD能否发送
	this->MCU_Send = false;
	this->OBD_Send = false;
	this->BD_Send = false;
	this->Speed_Send = false;
	this->BLUE_Send = false;
	this->DDP_Send = false;
	this->GGA_Send = false;
	this->confCnt = false;
	this->sendSconf = false;
	this->ADC_Send = false;
	this->requestreceivedfrom32 = false;
	this->config_received32 = false;
	this->ADC = "0.0,0.0";
	memset(DatBuf, 0, 1024);
}

//析构函数
CSerial::~CSerial()
{
	//关闭串口
	this->ClosePort();
}



//************************************************************************
//******************************配置串口**********************************
int CSerial::OpenPort(int PortNo, int baudrate, char databits, char stopbits, char parity)
{
	bool result = false;
	char pathname[20];

	sprintf(pathname, SerialPort, PortNo); //这里设置的路径

	this->m_fd = open(pathname, O_RDWR | O_NOCTTY | O_NDELAY);

	if (this->m_fd < 0)
	{
		printf("Can‘t open serial port\n");
		return result;
	}
	struct termios newtio;

	bzero(&newtio, sizeof(newtio));

	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;

	//设置波特率
	switch (baudrate)
	{
	case 2400:
		cfsetispeed(&newtio, B2400);
		cfsetospeed(&newtio, B2400);
		break;
	case 4800:
		cfsetispeed(&newtio, B4800);
		cfsetospeed(&newtio, B4800);
		break;
	case 9600:
		cfsetispeed(&newtio, B9600);
		cfsetospeed(&newtio, B9600);
		break;
	case 19200:
		cfsetispeed(&newtio, B19200);
		cfsetospeed(&newtio, B19200);
		break;
	case 38400:
		cfsetispeed(&newtio, B38400);
		cfsetospeed(&newtio, B38400);
		break;
	case 115200:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
		break;
	default:
		cfsetispeed(&newtio, B115200);
		cfsetospeed(&newtio, B115200);
	}

	//设置数据位，只支持7，8
	switch (databits)
	{
	case '7':
		newtio.c_cflag |= CS7;
		break;
	case '8':
		newtio.c_cflag |= CS8;
		break;
	default:
		printf("Unsupported Data_bits\n");
		return result;
	}

	//设置校验位
	switch (parity)
	{
	default:
	case 'N':
	case 'n':
	{
		newtio.c_cflag &= ~PARENB;
		newtio.c_iflag &= ~INPCK;
	}
	break;
	case 'o':
	case 'O':
	{
		newtio.c_cflag |= (PARODD | PARENB);
		newtio.c_iflag |= INPCK;
	}
	break;
	case 'e':
	case 'E':
	{
		newtio.c_cflag |= PARENB;
		newtio.c_cflag &= ~PARODD;
		newtio.c_iflag |= INPCK;
	}
	break;

	case 's':
	case 'S':
	{
		newtio.c_cflag &= ~PARENB;
		newtio.c_cflag &= ~CSTOPB;
	}
	break;
	}
	//设置停止位，值为1 or 2
	printf("stopbits:%c\n", stopbits);
	switch (stopbits)
	{
	case '1':
	{
		newtio.c_cflag &= ~CSTOPB;
		break;
	}
	case '2':
	{
		newtio.c_cflag |= CSTOPB;
		break;
	}
	default:
		printf("Unsupported stopbits.\n");
		return result;
	}

	//设置最少字符和等待时间，对于接收字符和等待时间没有特别的要求时，可设为0：
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;

	//刷清输入和输出队列
	tcflush(0, TCIOFLUSH);

	//激活配置，TCSANOW表示更改后立即生效。
	if ((tcsetattr(this->m_fd, TCSANOW, &newtio)) != 0)
	{ //判断是否激活成功。
		printf("Com set error\n");
		return result;
	}

	/*创建接受线程。*/
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	// 设置线程绑定属性
	int res = pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
	// 设置线程分离属性
	res += pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	//创建线程
	pthread_create(&pid, &attr, ReceiveThreadFunc, (void *)this);

	result = true;
	return result;
}




//************************************************************************
//******************************接收线程**********************************
void *CSerial::ReceiveThreadFunc(void *lparam)
{
	CSerial *serial_port = (CSerial *)lparam;
	serial_port->event.data.fd = serial_port->m_fd;
	serial_port->event.events = EPOLLET | EPOLLIN;

	if (epoll_ctl(serial_port->epfd, EPOLL_CTL_ADD, serial_port->m_fd, &serial_port->event) != 0)
	{
		//将读事件添加到epoll的事件队列中
		printf("set epoll error!\n");
		return NULL;
	}

	int i = 0, waiteNum = 0;
	//printf("Strat Receive thread!\n");
	while (true)
	{
		waiteNum = epoll_wait(serial_port->epfd, serial_port->events, 6, 10);
		//printf("waiteNum:%d\n", waiteNum);
		for (i = 0; i < waiteNum; i++)
		{
			if (serial_port->events[i].events & EPOLLIN)
			{
				//判断是否有数据进入
				//接受数据
				//printf("Receive!\n");
				serial_port->ReadPort(serial_port->DatBuf, 1024); //这里修改MAXLEN接收长度
				//数据处理
				serial_port->PackagePro(serial_port->DatBuf, serial_port->m_DatLen); //这里修改MAXLEN,处理的数据长度
			}
		}
	}
	return NULL;
}

/*由于linux的串口很可能一包最大16，如果要取定长度的数据，需要拼接,（注释部分仅参考！要用可能需要稍作修改）

int CSerial::ReadPort( char* Buf, int ReadLen )
{
	if(this->m_fd > 0)
	{
		int len = 0;
		int rdlen = 0;
		while (true) {
			rdlen = read(this->m_fd, Buf + len, ReadLen);
			printf("rdlen:%d\n", rdlen);
			len += rdlen;
			if (len == ReadLen) {
				this->WritePort(Buf, len);
				return len;
			}
		}
	}
}
*/



//************************************************************************
//******************************读端口数据**********************************
int CSerial::ReadPort(char *Buf, int ReadLen)
{
	if (this->m_fd > 0)
	{
		int rdlen = 0, len = 0;
		bzero(Buf, 1024);
		while (true)
		{
			rdlen = read(this->m_fd, Buf + len, ReadLen);
			len += rdlen;
			char buf0 = Buf[0];
			char buf1 = Buf[1];
			usleep(1); //延迟主语稳定收16字节数
			//std::cout<<"r:"<<rdlen<<"  "<<len <<"  "<<Buf[0]<<"  "<<Buf[1]<<"  "<<Buf[2]<<std::endl;
			//for(int i = 0;i<rdlen;i++)
			//printf("%c:%02X %d\n",Buf[i+len-rdlen],Buf[i+len-rdlen],i);
			
			
			/*OBD*/
			if (buf0 == 0x55)
			{
				if (rdlen > 0 && rdlen < 16 && len > 16 && len < 186)
				{ //针对不同的协议长度做修改,不适用于完全=16的情况
					if (Buf[4] == 0x00 && Buf[182] == 0x00 && Buf[183] == 0x65)
					{ //汽车OBD
						// printf("rdlen:%d\n", rdlen);
						this->m_DatLen = len;
						this->OK = true;
						this->OBD_Send = true;
						return len;
					}
					else
					{
						bzero(Buf, 1024);
						return 0;
					}
				}
				else if (len < 16 || len > 186)
				{
					bzero(Buf, 1024);
					return 0;
				}
			}


			
			/*MCU*/
			else if (buf0 == 0x0A && buf1 == 'M')
			{
				if (Buf[len - 1] == 0x0D)
				{ //针对不同的协议长度做修改,不适用于完全=16的情况
					//Buf[len-1]==0x0D,满足MCU或配置软件输入条件
					this->m_DatLen = len;
					this->OK = true;
					this->MCU_Send = true;
					return len;
				}
				else if (len > 800 || len < 16)
				{ //要注意 是否判断le<16
					bzero(Buf, 1024);
					return 0;
				}
			}


			else if (buf0 == 0x0A && buf1 == 'C')
			{
				// std::cout << "com coming" << std::endl;
				if (Buf[len - 1] == 0x0D)
				{ //针对不同的协议长度做修改,不适用于完全=16的情况
					//Buf[len-1]==0x0D,满足MCU或配置软件输入条件
					// for (int i = 0; i < len; i++)
					// printf("%02X", Buf[i]);
					this->m_DatLen = len;
					this->OK = true;
					this->COM_Send = true;
					return len;
				}
				else if (len > 800)
				{ //要注意 是否判断le<16
					bzero(Buf, 1024);
					return 0;
				}
			}
			else
			{ //要注意 是否判断le<16
				bzero(Buf, 1024);
				return 0;
			}
		}
	}
	else
		return 0;
}




//************************************************************************
//******************************读取数据**********************************
//读取n个字节到Buf
int CSerial::Read(char *Buf, int ReadLen)
{
	//串口句柄
	if (this->m_fd > 0)
	{
		int rdlen = 0;
		rdlen = read(this->m_fd, Buf, ReadLen);
		//打印显示接收到的数据
		if (rdlen > 0)
		{
			for (int j = 0; j < rdlen; j++)
			{
				printf("%x", Buf[j]);
			}
			printf("\n");
			/*//按字符串读同等于上述
				printf("rd_static = %d  buf = %s\n",rd_static,buf);
				memset(buf, 0, strlen(buf));*/
			return rdlen;
		}
		else //read failed
			return 0;
	}
	else
	{
		printf("no port\n");
		return 0;
	}
}



//************************************************************************
//******************************输出数据**********************************
//将Buf的n个字节输出
int CSerial::WritePort(char *Buf, int WriteLen)
{
	int wlen = write(this->m_fd, Buf, WriteLen);
	return wlen;
}



// void CSerial::PackagePro(char* Buf, int DataLen )
// {
// 	if(this->OK){
// 		int i = 0;
// 		printf("I am ReadDataProc!I have get %d chars.\n", DataLen);
// 		printf("RecvData is :\t");
// 		for(i = 0; i < DataLen; i++)
// 			printf("%02x\t", Buf[i]);
// 		printf("\n");
// 		bzero(Buf, 1024);
// 		this->OK = false;
// 	}
// }



//************************************************************************
//******************************数据处理**********************************
void CSerial::PackagePro(char *Buf, int DataLen)
{
	//--------------------------OBD发送-----------------------------
	if (this->OK && !this->CAN_Send && this->OBD_Send)
	{
		int i = 13;
		for (int j = 0; j < 28; j++)
		{
			this->OBD[j][0] = Buf[i];
			this->OBD[j][1] = Buf[i + 1];
			this->OBD[j][2] = Buf[i + 2];
			this->OBD[j][3] = Buf[i + 3];
			i += 6;
			// printf("%02x  %02x  %02x  %02x\n",this->OBD[j][0],this->OBD[j][1],this->OBD[j][2],this->OBD[j][3]);
		}

		bzero(Buf, 1024);
		this->OK = false;
		this->OBD_Send = false;
		this->CAN_Send = true;
	}
	//--------------------------MCU发送-----------------------------
	else if (this->OK && this->MCU_Send)
	{
		uint16_t i;
		
		for (i = 0; i < this->m_DatLen; i++)
		{
		 	 	//std::cout << BD[i - 3];
		 		//printf("%c",Buf[i]);
		 		//printf("%c:%02X ",Buf[i],Buf[i]);
		}
		printf("\n");
				
		//检测帧头
		if (Buf[1] == 'M')
		{
			//检测帧头MD，北斗数据
			if (Buf[2] == 'D' && !this->BD_Send)
			{ 
				//获取到北斗数据给出提示信息
				//std::cout << "get bd: " << std::endl;
				//开始接收北斗数据
				for (i = 3; i < this->m_DatLen - 1; i++)
				{
					this->BD[i - 3] = Buf[i];
					//每接收一个字节，打印一下
					//std::cout << BD[i - 3] << std::endl;
				}
				//长度重新赋值
				this->BD_DatLen = this->m_DatLen - 4;
				//北斗上传使能
				this->BD_Send = true;
				//缓存数组清零
				bzero(Buf, 1024);
			}
			else if(Buf[2] == 'S' && !this->Speed_Send)
			{
				//获取到北斗数据给出提示信息
				//std::cout << "get speed: " << std::endl;
				//开始接收北斗数据
				for (i = 3; i < this->m_DatLen - 1; i++)
				{
					this->Speed[i - 3] = Buf[i];
					//每接收一个字节，打印一下
					//std::cout << Speed[i - 3];
				}
				//长度重新赋值
				this->Speed_DatLen = this->m_DatLen - 4;
				//北斗上传使能
				this->Speed_Send = true;
				//缓存数组清零
				bzero(Buf, 1024);

			}

			//检测帧头ME，蓝牙数据
			else if (Buf[2] == 'E' && !this->BLUE_Send)
			{ 
				//获取到蓝牙数据给出提示信息
				//std::cout << "get blue: " << std::endl;
				//开始接收蓝牙数据	
				for (i = 3; i < this->m_DatLen - 1; i++)
				{
					this->BLUE[i - 3] = Buf[i];
					//每接收一个字节，打印一下
					//std::cout << BLUE[i - 3];
				}
				this->BLUE_DatLen = this->m_DatLen - 4;
				this->BLUE_Send = true;
			}
			//检测帧头MP，调度屏数据
			else if (Buf[2] == 'P' && !this->DDP_Send)
			{ 
				//获取到调度屏数据给出提示信息
				//std::cout << "get ddp: " << std::endl;
				//开始接收调度屏数据
				for (i = 3; i < this->m_DatLen - 1; i++)
				{
					this->DDP[i - 3] = Buf[i];
					//每接收一个字节，打印一下
					//std::cout << DDP[i - 3];
				}
				this->DDP_Send = true;
			}
			//检测帧头MN，GGA数据
			else if (Buf[2] == 'N')
			{ 
				//获取到GGA数据给出提示信息
				//GGA禁止发送
				this->GGA_Send = false;
				//GGA数组清零
				memset(this->GGA, 0, sizeof(this->GGA));
				//接收GGA数据
				for (i = 3; i < this->m_DatLen - 1; i++)
				{
					this->GGA[i - 3] = Buf[i];
					//std::cout <<"GGA:"<< GGA[i - 3]<<std::endl;
				}
				this->GGA_DatLen = this->m_DatLen - 4;
				//打印GGA语句
				//std::cout << "\r\nget GGA: " << this->GGA_DatLen << std::endl;
				//使能发送
				this->GGA_Send = true;
			}
			//检测帧头MA，ADC数据
			else if (Buf[2] == 'A')
			{
				this->ADC = Buf;
				//std::cout << "get ADC: " << this->ADC << std::endl;
				this->ADC_Send = true;
			}

			else if(Buf[2] == 'C')
			{
				//给出提示信息
				std::cout << "\r\n\r\nGET CONFIG REQUEST: \r\n\r\n" << std::endl;
				//SHOU DAO REQUEST
//				this->config_received32=false;
				this->requestreceivedfrom32=true;
				
				//std::cout <<this->requestreceivedfrom32 << std::endl;

			}
			else if(Buf[2] == 'R')
			{
				//给出提示信息
				std::cout << "\r\n\r\n32 has received config data.\r\n\r\n " << std::endl;
				this->config_received32=true;
				this->requestreceivedfrom32=false;

			}
			else
			{
				//std::cout << "recv char 2 error" << std::endl;
				//缓存数组清零
				bzero(Buf, 1024);
				return;
			}
		}



		else
		{
			//如果接收到的第一个字符不是‘M’,给出帧头错误提示
			//std::cout << "recv char 1 error" << std::endl;
			//缓存数组清零
			bzero(Buf, 1024);
			return;
		}
		//缓存数组清零
		bzero(Buf, 1024);
		//系统状态异常
		this->OK = false;
		//STM32发送异常
		this->MCU_Send = false;
	}
	





	//电脑端上位机发送的配置信息
	else if (this->OK == true && this->COM_Send == true)
	{
		if (Buf[2] == 'P')
		{
			char buf[50];
			//缓存数组清零
			bzero(buf, sizeof(buf));
			//接收PC发送的配置信息
			for (int i = 3; i < this->m_DatLen - 1; i++)
				buf[i - 3] = Buf[i];
			
			conf.push_back(buf);
			//打印配置文件
			std::cout << "conf<"<<confCnt<<">: " << conf[confCnt] << std::endl;
			this->confCnt++;
			this->OK = false;
			this->COM_Send = false;
			if (this->confCnt >= 118)
			{
				std::fstream Conf;
				std::cout << "break and save" << std::endl;
				Conf.open("/opt/config.conf", std::ios::out); //|std::ios::out|std::ios::binary);
				if (Conf.is_open())
				{
					for (int i = 0; i < this->confCnt; i++)
					{
						Conf << conf[i] << std::endl;
					}
				}
				Conf.close(); //****
				confCnt = 0;
				this->conf.clear();
			}
		}
		else if (Buf[2] == 'C')
		{
			std::cout << "read config sendConf1<"<<this->sendConf<<">\n";
			this->sendConf = true; //***
			std::cout << "read config sendConf2<"<<this->sendConf<<">\n";
		}
		else if (Buf[2] == 'S')
		{
			std::cout << "read config sendSconf<"<<this->sendSconf<<">\n";
			this->sendSconf = true;
			std::cout << "read config sendSconf<"<<this->sendSconf<<">\n";
		}
		else if (Buf[2] == 'R')
		{
			std::cout << "reboot" << std::endl;
			system("reboot");
		}
	}
}











//************************************************************************
//******************************关闭串口**********************************
void CSerial::ClosePort()
{
	//串口句柄大于0表明串口正常
	if (this->m_fd > 0)
	{
		//根据串口句柄关闭串口
		close(this->m_fd);
	}
}
