#include "UnoTransport.hpp"
#include <stdio.h>
#include <arpa/inet.h>
#include <assert.h>
#include <unistd.h>
#include <cstring>
#include <inttypes.h>

#define PORT        13002
#define IP_ADDR     "127.0.0.1"
#define TIMEOUT     8000
#define COUNT       10

int main(void)
{
    UnoTransportClient client(false);

    struct sockaddr_in server;
    assert (inet_pton(AF_INET, IP_ADDR, &server.sin_addr) != 0);
    server.sin_family       = AF_INET;
    server.sin_port         = htons (PORT);
    printf("client selfport = %" PRIu16 "\n", client._dbg_get_self_port());

    char msg[100] = {0,};
    int ret;
    if(client.connect (server, TIMEOUT) == 0) {
        for(int i=0; i<COUNT; i++) {
            memset (msg, 0, sizeof(msg));
            sprintf(msg, "Hello from client, count=%d, port=%" PRIu16, i, client._dbg_get_port());    
        
        

            // ret = client.send_msg_reliable (msg, strlen(msg)+1);
            // printf ("send ret = %d\n", ret);

            memset (msg, 0, sizeof(msg));
            ret = client.recv_msg (msg, sizeof(msg)-1, TIMEOUT);
            printf("recv ret = %d msg = %s\n", ret, msg);

            if(ret == UNO_TRANSPORT_CONN_CLOSE_RECVD) {
                printf("connection closed by server\n");
                break;
            }
            if(ret == UNO_TRANSPORT_ERR) {
                break;
            }

            sleep(1);
        }
    }

    return 0;
}