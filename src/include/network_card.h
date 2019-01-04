#ifndef __NETWORK_CARD_
#define __NETWORK_CARD_
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <malloc.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/types.h>  
#include <netinet/in.h>  
#include <arpa/inet.h>  
#include <net/if.h>
#define BUF_SIZE 1024
#include <net/route.h>
#define PATH_ROUTE "/proc/net/route"
using namespace std;
int get_ip(string net_name, string &strip);
int init_net_card();


#endif 
