#ifndef ___MSG_QUEUE_H_
#define ___MSG_QUEUE_H_

#define SIGDAQ 	8888

/*-Msg queue struct*/
struct _msgbuf
{
	long mtype;
	char ip[16]; //ip��ַ
	float si; //�ź�ǿ��
	float sq; //�ź�����
};

#endif
