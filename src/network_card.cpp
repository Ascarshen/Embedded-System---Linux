
#include "include/network_card.h"
#include <unistd.h>
 int get_ip(string net_name, string &strip)
{
    int sock_fd;
    struct ifconf conf;
    struct ifreq *ifr;
    char buff[BUF_SIZE] = {0};
    int num;
    int i;
 
    sock_fd = socket(PF_INET, SOCK_DGRAM, 0);
    if ( sock_fd < 0 )     
		return -1;
 
    conf.ifc_len = BUF_SIZE;
    conf.ifc_buf = buff;
 
    if ( ioctl(sock_fd, SIOCGIFCONF, &conf) < 0 )
    {
        close(sock_fd);
        return -1;
    }
 
    num = conf.ifc_len / sizeof(struct ifreq);
    ifr = conf.ifc_req;
 
    for(i = 0; i < num; i++)
    {
        struct sockaddr_in *sin = (struct sockaddr_in *)(&ifr->ifr_addr);
 
        if ( ioctl(sock_fd, SIOCGIFFLAGS, ifr) < 0 )
        {
                close(sock_fd);
                return -1;
        }
 
        if ( (ifr->ifr_flags & IFF_UP) && strcmp(net_name.c_str(),ifr->ifr_name) == 0 )
        {
                strip = inet_ntoa(sin->sin_addr);
                close(sock_fd);
 
                return 0;
        }
		
        ifr++;
    }
	
    close(sock_fd);
 
    return -1;
}
 
 



#if  0
/*
it is only test 
*/

int main (void)
{
	
 char pGateway[32]={0};
 get_gateway("eth1", pGateway);
  printf("%s",pGateway);	
}

#endif 
 