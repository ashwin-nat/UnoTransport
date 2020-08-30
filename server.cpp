#include "UnoTransport.hpp"
#include <cstring>
#include <stdio.h>
#include <inttypes.h>
#include <assert.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstdlib>
#include <ctime>

#define PORT        13002
#define BUFFER_SIZE 5000

void print_buffer (uint8_t *buffer, int size)
{
    if(size <= 0) {
        printf("err\n");
        perror("recvfrom");
        return;
    }
    int i;
    for(i=0; i<size; i++) {
        printf("%02" PRIX8 " ", buffer[i]);
    }
    printf("\n");
}

bool should_close (void)
{
    // int random = rand() % 100;
    // if(random < 10) {
    //     return true;
    // }
    // else {
    //     return false;
    // }
    return false;
}

int main (void)
{
    srand(time(NULL));
    UnoTransportServer server(PORT);

    struct sockaddr_in src;
    printf("server pid = %ld\n", (long)getpid());
    bool is_spl = false;

    uint8_t buffer[5000] = {0,};
    int bytes;
    while(1) {
        memset (&src, 0, sizeof(src));
        //bytes = server.recv_msg (buffer, sizeof(buffer), UNO_TRANSPORT_TIMEOUT_INFINITE);
        //bytes = server.listen (src);
        bytes = server.recv_msg (buffer, sizeof(buffer), &src, is_spl);
        if(is_spl) {
            printf("spl msg: ");
        }
        else {
            printf("regular msg: ");
        }
        printf("bytes = %d msg = %s\n", bytes, buffer);
        // print_buffer (buffer, bytes);
        //sendto (server.fd, buffer, bytes, 0, (struct sockaddr*)&src, len);

        if(should_close()) {
            printf("sending explicit close conn\n");
            server.close_connection (src);
        }
        else {
            server.send_msg_reliable (buffer, bytes, src);
        }

        memset (buffer, 0, bytes);
    }

    return 0;
}