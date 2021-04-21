using namespace std;

#include <stdio.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h>

#include "common.h"
#include "channel.h"
  
int main() { 
    int sck; 
    uint8_t buffer[BRBUFSIZE];
    struct sockaddr_in srv, cli; 
      
    // Creating socket file descriptor 
    if ( (sck = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) { 
        perror("socket creation failed"); 
        exit(EXIT_FAILURE); 
    } 
      
    memset(&srv, 0, sizeof(srv)); 
    memset(&cli, 0, sizeof(cli));

    srv.sin_family = AF_INET; // IPv4 
    srv.sin_addr.s_addr = INADDR_ANY; 
    srv.sin_port = htons(PORT); 
      
    // Bind the socket with the server address 
    if ( bind(sck, (const struct sockaddr *)&srv, sizeof(srv)) < 0 )  { 
        perror("bind failed"); 
        exit(EXIT_FAILURE); 
    } 
      
    ssize_t n;
    socklen_t len; 
    len = sizeof(cli);

    Channel ch(sck);

    while (1) {
        n = recvfrom(sck, buffer, BRBUFSIZE-1,  MSG_WAITALL, (struct sockaddr *)&cli, &len);
        buffer[n] = '\0';
        ch.recvBcst(buffer, n, &cli);
        //printf("recv : addr %08X, port %d, msg %s\n", cli.sin_addr.s_addr, cli.sin_port, buffer);
        //printf("sent : addr %08X, port %d\n", cli.sin_addr.s_addr, cli.sin_port);
        //sendto(sck, "good", strlen("good"), MSG_CONFIRM, (const struct sockaddr *)&cli, len);
    }

    return 0; 
}
