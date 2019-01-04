#ifndef ___MSG_QUEUE_H_
#define ___MSG_QUEUE_H_

#define SIGDAQ 	8888

/*-Msg queue struct*/
struct _msgbuf
{
	long mtype;
	char ip[16]; //ip地址
	float si; //信号强度
	float sq; //信号质量
};

#endif
