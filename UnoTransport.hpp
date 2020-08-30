/**
 * @file UnoTransport.cpp
 * @author Ashwin Natarajan
 * @brief This file contains the declarations of the classes and methods needed for this protocol implementation
 * @version 0.1
 * @date 2020-08-30
 * 
 * @copyright MIT License

Copyright (c) 2020 Ashwin N

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
 * 
 */

#ifndef __UNO_TRANSPORT_HPP__
#define __UNO_TRANSPORT_HPP__

#include <stdint.h>         //for fixed width integers
#include <list>             //for std::list
#include <netinet/in.h>     //for struct sockaddr_in
#include <cstring>          //for memset, memcmp, memcpy, etc
#include <time.h>           //for struct timespec and clock_gettime()
/**************************************************************************************************************************/
//constants definitions
#define UNO_TRANSPORT_TIMEOUT_INFINITE  -1
#define UNO_TRANSPORT_HDR_LEN           (\
                                        UNO_TRANSPORT_HDR_SEQ_LEN + \
                                        UNO_TRANSPORT_HDR_RC_LEN + \
                                        UNO_TRANSPORT_HDR_PROTO_VER_LEN + \
                                        UNO_TRANSPORT_HDR_CMD_LEN + \
                                        UNO_TRANSPORT_HDR_MSG_LEN_LEN \
                                        )
#define UNO_TRANSPORT_MTU               (512-(UNO_TRANSPORT_HDR_LEN))
#define UNO_TRANSPORT_RECO_BUFF_SIZE    ((UNO_TRANSPORT_MTU))   //the recommended buffer size
#define UNO_TRANSPORT_KEEPALIVE_INT     5000                    //the default keepalive interval
#define UNO_TRANSPORT_DFL_MAX_CONN      10                      //default max connections

#define UNO_TRANSPORT_REL_MAX_RETRIES   4
#define UNO_TRANSPORT_REL_RETRY_INT_MS  20

//status codes
#define UNO_TRANSPORT_CONN_REFUSED      -2
#define UNO_TRANSPORT_CLIENT_NOT_CONN   -2
#define UNO_TRANSPORT_CONN_CLOSE_RECVD  -3
#define UNO_TRANSPORT_ERR               (-1)
/**************************************************************************************************************************/
struct uno_hdr;
enum class _close_reason {
    CLOSE_REASON_REQUESTED_BY_CLIENT,
    CLOSE_REASON_REQUESTED_BY_SERVER,
    CLOSE_REASON_TIMEDOUT,
    
    CLOSE_REASON_UNKNOWN,
};
/**
 * @brief The below is the structure that is used to represent a connection. All members are public for the sake of debugging,
            should be changed
 * 
 */
struct UnoConnection {
    public:
        int fd;                         //the fd that is corresponding to the speicied connection
        uint16_t self_port;             //holds the randomly generated port for this connection. redundant, should be removed
        uint16_t client_seq;            //the sequence id for this client
        struct sockaddr_in cli_addr;    //the IPv4 addr of the client
        bool connected;                 //a flag to tell if the client is connected or not
        struct timespec last_msg_time;  //stores that last received message time
        _close_reason close_reason;     //stores the reason that the connection is being closed, is basically an argument to the destructor
        
        /**
         * @brief Construct a new Uno Connection object
         * 
         * @param fd The fd of the already created socket
         * @param cli reference to the struct socakddr_in holding the client's IPv4 addr
         * @param hdr reference to the structure containing the deserialised header
         * @param ka_dur the keep alive interval
         * @param curr_time const reference to the struct containing the curr time, this value is used
                    as the initial value for last_msg_time
         */
        UnoConnection (int fd, const struct sockaddr_in &cli, uno_hdr &hdr, uint16_t ka_dur, const struct timespec &curr_time);
        /**
         * @brief Construct a new Uno Connection object by copying the given one
         * 
         * @param cpy 
         */
        UnoConnection (const UnoConnection &cpy);
        /**
         * @brief Destroy the Uno Connection object. Sends connection close req to the client if the 
                    client is still connected
         * 
         */
        ~UnoConnection (void);
        /**
         * @brief Overloaded operator for move
         * 
         * @param t 
         * @return UnoConnection& 
         */
        UnoConnection & operator = (const UnoConnection &t) {
            this->fd = t.fd;
            this->self_port = t.self_port;
            memcpy (&(this->cli_addr), &(t.cli_addr), sizeof(this->cli_addr));

            return *this;
        }
        /**
         * @brief Sends a connection response to the device sending the connection request
         * 
         * @param hdr info in the header struct
         * @param ka_dur the keepalive duration
         * @return int retuns number of bytes sent on success, UNO_TRANSPORT_ERR on failure
         */
        int send_connect_rsp (uno_hdr &hdr, uint16_t ka_dur);
        /**
         * @brief Set the close reason variable
         * 
         * @param reason - the given reason for connection closure
         */
        void set_close_reason (_close_reason reason);
};
/**************************************************************************************************************************/
/**
 * @brief This is the class that is used to represent a client.
 * 
 */
class UnoTransportClient {
    //there are few fields here that can be removed, but it works for now
    private:
        int fd;                     //the fd of the socket
        uint16_t seq;               //the sequence number of the client
        uint16_t server_seq;        //the sequence number of the server
        bool connected;             //a flag to store if the client is still connected to the server
        struct sockaddr_in addr;    //stores the IPv4 address of the new endpoint created by the server
        bool can_broadcast;         //stores the option of whether this client can broadcast
        uint16_t keepalive_dur;     //the keepalive interval as specified by the server
        uint16_t self_port;         //the randomly generated port number of this client

    public:
        /**
         * @brief Construct a new Uno Transport Client object. Creates a socket at a random port without broadcast permissions
         * 
         */
        UnoTransportClient (void);
        /**
         * @brief Construct a new Uno Transport Client object with the specified broadcast permissions
         * 
         * @param broadcast 
         */
        UnoTransportClient (bool broadcast);
        /**
         * @brief Destroy the Uno Transport Client object. Send connection close message if still connected
         * 
         */
        ~UnoTransportClient (void);

        /**
         * @brief get the port number of the connection (on the server side) (TO BE REMOVED)
         * 
         * @return uint16_t the port number value
         */
        uint16_t _dbg_get_port (void);
        /**
         * @brief get the port number of the client side socket (TO BE REMOVED)
         * 
         * @return uint16_t the port number
         */
        uint16_t _dbg_get_self_port (void);
        /**
         * @brief Connect to the specified server
         * 
         * @param dst - reference to the struct sockaddr_in instance containing the server's IPv4 address
         * @param to_ms - the timeout in milliseconds
         * @return int - 0 if successful, UNO_TRANSPORT_ERR if failed
         */
        int connect (const struct sockaddr_in &dst, int to_ms);
        /**
         * @brief Send a mesage to the server that this client is connected to
         * 
         * @param msg - void pointer to the mesage
         * @param len - length of the message
         * @return int - returns UNO_TRANSPORT_ERR on failure, or number of bytes sent on success
         */
        int send_msg (const void *msg, size_t len);
        /**
         * @brief Sends the given message to the server, expects an ACK from the server.
         * 
         * @param msg - void pointer to the message
         * @param len - length of the message
         * @return int - returns UNO_TRANSPORT_ERR on no ACK or general failure, or number of bytes sent on success
         */
        int send_msg_reliable (const void *msg, size_t len);
        /**
         * @brief - Sends a special (connectionless) message to the server at the specified address
         * 
         * @param msg - const void pointer to the message
         * @param len - length of the message
         * @param dst - const reference to the struct containing the destination address
         * @return int - returns number of bytes sent, if successful, else, UNO_TRANSPORT_ERR on failure
         */
        int send_msg_spl (const void *msg, size_t len, const struct sockaddr_in &dst);
        /**
         * @brief Receives a message from the connected server into the specified server
         * 
         * @param buffer - The buffer where the message is to be written
         * @param buffer_size - The size of this buffer 
         * @param to_ms - The timeout in milliseconds
         * @return int - The number of bytes received
         */
        int recv_msg (void *buffer, size_t buffer_size, int to_ms);
        /**
         * @brief - Sends a keepalive message to the server
         * 
         * @return int - returns 0 on success, or UNO_TRANSPORT_ERR on failure, UNO_TRANSPORT_CLIENT_NOT_CONN if not connected
         */
        int send_keepalive (void);
        
};
/**
 * @brief The class that holds the server information
 * 
 */
class UnoTransportServer {
    private:
        int fd;                                     //the fd of the service side socket
        uint16_t seq;                               //the sequence number
        uint16_t keepalive_dur;                     //the keepalive duration (not configurable as of now)
        uint8_t max_conn;                           //the max connections that this server can support
        std::list<UnoConnection> client_list;       //std::list containing active connections

    public:
        /**
         * @brief Construct a new Uno Transport Server object. Does not create a socket
         * 
         */
        UnoTransportServer(void);
        /**
         * @brief Construct a new Uno Transport Server object. Creates a socket with the specified port
         * 
         * @param port - the port number of this server socket
         */
        UnoTransportServer(uint16_t port);
        /**
         * @brief Construct a new Uno Transport Server object. Creates a socket with the specified port
         * 
         * @param port - the port number of this server socket
         * @param max_connections - the max number of connections supported by this socket
         */
        UnoTransportServer(uint16_t port, uint8_t max_connections);
        /**
         * @brief Destroy the Uno Transport Server object. Sends connection close message to all connected clients
                    and closes the server side socket
         * 
         */
        ~UnoTransportServer(void);
        /**
         * @brief Create a socket object at the specified port
         * 
         * @param port - the port number of this server socket
         * @return int - the fd number
         */
        int create_socket (uint16_t port);

        /**
         * @brief Receives the incoming message in the specified buffer
         * 
         * @param buffer - the buffer to store the incoming msg
         * @param buffer_size - the size of this buffer
         * @param src - optional pointer to the struct sockaddr_in object where the source address is to be filled in
         * @param is_spl - reference to the bool variable that will be set if the incoming msg is a spl msg
         * @return int - number of bytes received
         */
        int recv_msg (void *buffer, size_t buffer_size, struct sockaddr_in *src, bool &is_spl);
        /**
         * @brief Sends the given message to the specified client, if connected
         * 
         * @param msg - void pointer to the message
         * @param len - the length of the message
         * @param dst - const reference to the structure containing the destination
         * @return int - returns number of bytes sent on success, UNO_TRANSPORT_CLIENT_NOT_CONN if the client is not found
                        in the client table, UNO_TRANSPORT_ERR on any general purpose failures
         */
        int send_msg (const void *msg, size_t len, const struct sockaddr_in &dst);
        /**
         * @brief Sends the given message to the specified client, if connected. Expects an ACK from the recipient
         * 
         * @param msg  - void pointer to the message
         * @param len - the length of the message
         * @param dst - const reference to the structure containing the destination
         * @return int - returns number of bytes sent on success, UNO_TRANSPORT_CLIENT_NOT_CONN if the client is not found
                        in the client table, UNO_TRANSPORT_ERR on lack of ACK or any general purpose failures
         */
        int send_msg_reliable (const void *msg, size_t len, const struct sockaddr_in &dst);
        /**
         * @brief Sends the connection close requet to the specified client
         * 
         * @param dst - reference to the structure containing the destination
         * @return int - returns 0 on success, UNO_TRANSPORT_CLIENT_NOT_CONN if the client is not found
                        in the client table, UNO_TRANSPORT_ERR on any general purpose failures
         */
        int close_connection (struct sockaddr_in &dst);
        
};

#endif  //__UNO_TRANSPORT_HPP__