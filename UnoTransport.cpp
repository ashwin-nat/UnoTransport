/**
 * @file UnoTransport.cpp
 * @author Ashwin Natarajan
 * @brief This file contains the implementation of the protocol and methods specified in UnoTransport.hpp
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
#include "UnoTransport.hpp"
#include <unistd.h>             //for close, socket, etc
#include <fcntl.h>              //for fcntl to set sockets as non blocking
#include <poll.h>               //for poll()
#include <assert.h>             //for asserts
#include <arpa/inet.h>          //for IPv4 related stuff
#include <vector>               //std::vector
#include <stdio.h>              //for fprintf, perror, etc
#include <sys/timerfd.h>        //for timerfd
#ifdef  UNO_TRANSPORT_DEBUG
#include <inttypes.h>           //for format specifiers to fixed width integers
#endif

/**************************************************************************************************************************/
//debug specific macros
#ifdef  UNO_TRANSPORT_DEBUG
#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"
#define UNO_DBG_PREFIX     ANSI_COLOR_RED "<====================>"
#define UNO_DBG_SUFFIX     "<====================>" ANSI_COLOR_RESET "\n"
#endif

#ifndef UNO_TRANSPORT_DEBUG
#define NDEBUG      //remove asserts if the debug macro is not set
#endif
/**************************************************************************************************************************/
//general purpose macros
#define UNO_CLEAR_BIT(x,n)              ((x) & ~(1UL << n))
#define UNO_SET_BIT(x,n)                ((x) | (1UL << n) )
#define UNO_IS_BIT_SET(x,n)             ((x) & (1UL <<n))
#define UNO_SEC_TO_NS(x)                ((x) * 1000 * 1000 * 1000)
#define UNO_MS_TO_NS(x)                 ((x) * 1000 * 1000)
#define UNO_MS_TO_SEC(x)                ((x) / 1000)
#define UNO_NS_TO_SEC(x)                ((x) / (1000 * 1000 * 1000))
/**************************************************************************************************************************/
//the version of this protocol
#define UNO_TRANSPORT_PROTO_VER         1
/**************************************************************************************************************************/
//header fields lengths
#define UNO_TRANSPORT_HDR_SEQ_LEN       2
#define UNO_TRANSPORT_HDR_RC_LEN        1
#define UNO_TRANSPORT_HDR_PROTO_VER_LEN 1
#define UNO_TRANSPORT_HDR_CMD_LEN       1
#define UNO_TRANSPORT_HDR_MSG_LEN_LEN   2
/**************************************************************************************************************************/
//header fields positions
#define UNO_TRANSPORT_HDR_SEQ_POS       0
#define UNO_TRANSPORT_HDR_RC_POS        2
#define UNO_TRANSPORT_HDR_PROTO_VER_POS 3
#define UNO_TRANSPORT_HDR_CMD_POS       4
#define UNO_TRANSPORT_HDR_MSG_LEN_POS   5
/**************************************************************************************************************************/
//general purpose header related macros
#define UNO_SEQ_ID_RESET                0
#define UNO_TRANSPORT_CTRL_MSG_MAX_SIZE (10)
/**************************************************************************************************************************/
//the below are the bits that are supported in the control flag, the number is their position
#define UNO_TRANSPORT_CMD_CONTROL_MSG   (0)
#define UNO_TRANSPORT_CMD_RELIABLE_MSG  (1)
#define UNO_TRANSPORT_CMD_ACK_MSG       (2)
#define UNO_TRANSPORT_CMD_SPL_MSG       (3) //a special msg is one that will be passed to the app layer without any header
                                            //or connection validation, should be used for discovery, control msg flag must
                                            //not be set

//In a control command, extra info will be placed on the data section of the message.
//  the first byte will be the command ID, followed by data specific to that command.
//  commands must be very simple
/**
 * @brief The command ID for connection request. This command is used for both opening and 
            closing connections There will be one byte of additional data
                byte 1          = mode
                            0   = conn open req
                            1   = conn close req by client
                            2   = conn close req by server (explicit, server wants the client gone)
                            3   = conn close req by server (timeout)
 * 
 */
#define UNO_TRANSPORT_CTRL_CONN_REQ                             (1)
#define UNO_TRANSPORT_CTRL_CONN_REQ_LEN                         (2)
#define UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_OPEN               (0)
#define UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_CLOSE_CLIENT_REQ   (1)
#define UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_CLOSE_SERVER_EXPL  (2)
#define UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_CLOSE_SERVER_TO    (3)
/**
 * @brief The command ID for connection response. Additional data for connection response (if successful)
            There will be no additional data (msg len = 0) if the server is full
            bytes 1,2       = port number (in nwk byte order)
            bytes 3,4       = keepalive interval in ms (in nwk byte order)
 * 
 */
#define UNO_TRANSPORT_CTRL_CONN_RSP                             (2)
#define UNO_TRANSPORT_CTRL_CONN_RSP_LEN                         (5)
/**
 * @brief The command ID for error status. Additional data:
            byte            = status code, depends on command, full list to be populated here
 * 
 */
#define UNO_TRANSPORT_CTRL_ERR_RSP                              (3)
#define UNO_TRANSPORT_CTRL_ERR_RSP_LEN                          (2)

/**
 * @brief The command ID for connection keep alive. There is no additioanl data for this
 * 
 */
#define UNO_TRANSPORT_CTRL_KEEPALIVE                            (4)
#define UNO_TRANSPORT_CTRL_KEEPALIVE_LEN                        (1)

/**
 * @brief The command ID for the connection closed command. This will only be sent by the server
            There will be 1 byte of additional data
            byte 1 = reason
                        0 = close requested by client
                        1 = timedout
                        2 = explicit close notified by server (the caller doesn't like this client)
 * 
 */
#define UNO_TRANSPORT_CTRL_CONN_CLOSED  (5)
#define UNO_TRANSPORT_CTRL_CONN_CLOSED_LEN (2)
#define UNO_TRANSPORT_CTRL_CONN_CLOSED_OPT_REQ      (0)
#define UNO_TRANSPORT_CTRL_CONN_CLOSED_OPT_TIMEDOUT (1)
#define UNO_TRANSPORT_CTRL_CONN_CLOSED_OPT_EXPL     (2)


/**************************************************************************************************************************/
/*  HEADER FORMAT - drawn using the awesome website http://asciiflow.com/

    +--------------------------------+----------+-------------------------------------+
    | seq id   |retry |proto.| cmd   | msg len  |     data                            |
    |          |count |Ver   |       |          |                                     |
    +------------------------+-------+----------+-------------------------------------+
    0          2      3      4       5          6

    NOTE: all multibyte fields will be in nwk byte order
*/
/**************************************************************************************************************************/
/**
 * @brief This struct will hold the header in an easily accessible form. also supports 
            serialisation and deserialisation
 * 
 */
struct uno_hdr {
    public:
        uint16_t    seq_id;     //sequence id
        uint8_t     rc;         //retry count
        uint8_t     proto_ver;  //protocol version
        uint8_t     cmd;        //command flags
        uint16_t    msg_len;    //message length

        /**
         * @brief Construct a new uno hdr object with everything set to 0
         * 
         */
        uno_hdr (void) {
            seq_id  = 0;
            rc      = 0;
            proto_ver   = UNO_TRANSPORT_PROTO_VER;
            cmd     = 0;
            msg_len = 0;
        }
        /**
         * @brief Construct a new uno hdr object by deserialising the given flag buffer containing the header
         * 
         * @param hdr pointer to the buffer containing the header
         */
        uno_hdr (const uint8_t *hdr) {
           deserialise (hdr);
        }
        /**
         * @brief Construct a new uno hdr object using the given paramters
         * 
         * @param seq_id 
         * @param cmd 
         * @param msg_len 
         */
        uno_hdr (uint16_t seq_id, uint8_t cmd, uint16_t msg_len) {
            this->seq_id    = seq_id;
            this->cmd       = cmd;
            this->msg_len   = msg_len;
            
            this->rc        = 0;
            this->proto_ver = UNO_TRANSPORT_PROTO_VER;
        }

        /**
         * @brief Construct a new uno hdr object. copy the values of x into this
         * 
         * @param x the object to copy from
         */
        uno_hdr (const uno_hdr &x) {
            seq_id      = x.seq_id;
            rc          = x.rc;
            proto_ver   = x.proto_ver;
            cmd         = x.cmd;
            msg_len     = x.msg_len;
        }

        /**
         * @brief serialise this struct into the given buffer
         * 
         * @param hdr the buffer to which the data is to be serialised, which is assumed 
                            to be not nullptr and large enough
         */
        void serialise (uint8_t *hdr) {
            //2 bytes seq_id in nwk byte order
            hdr[UNO_TRANSPORT_HDR_SEQ_POS+0]    = (this->seq_id >> 8) & 0xFF;
            hdr[UNO_TRANSPORT_HDR_SEQ_POS+1]    = (this->seq_id >> 0) & 0xFF;

            //1 byte reboot count
            hdr[UNO_TRANSPORT_HDR_RC_POS]       = this->rc;

            //1 byte proto ver
            hdr[UNO_TRANSPORT_HDR_PROTO_VER_POS]= UNO_TRANSPORT_PROTO_VER;

            //1 byte cmd info
            hdr[UNO_TRANSPORT_HDR_CMD_POS]      = this->cmd;

            //2 bytes msg len
            hdr[UNO_TRANSPORT_HDR_MSG_LEN_POS+0]= (this->msg_len >> 8) & 0xFF;
            hdr[UNO_TRANSPORT_HDR_MSG_LEN_POS+1]= (this->msg_len >> 0) & 0xFF;
        }

        /**
         * @brief deserialise from the given buffer into this hdr struct
         * 
         * @param hdr the buffer containing the header, which is assumed to be not nullptr and
                        large enough
         */
        void deserialise (const uint8_t *hdr) {
            //2 byte seq in nwk byte order
            seq_id  = 0;
            seq_id |= hdr[UNO_TRANSPORT_HDR_SEQ_POS+0] << 8;
            seq_id |= hdr[UNO_TRANSPORT_HDR_SEQ_POS+1] << 0;

            //1 byte reboot count
            rc      = hdr[UNO_TRANSPORT_HDR_RC_POS];

            //1 byte proto ver
            proto_ver= hdr[UNO_TRANSPORT_HDR_PROTO_VER_POS];

            //1 byte cmd info
            cmd      = hdr[UNO_TRANSPORT_HDR_CMD_POS];

            //2 byte msg len - in nwk byte order
            msg_len  = 0;
            msg_len |= hdr[UNO_TRANSPORT_HDR_MSG_LEN_POS+0] << 8;
            msg_len |= hdr[UNO_TRANSPORT_HDR_MSG_LEN_POS+1] << 0;
        }

        /**
         * @brief checks if this header is valid or out of order or duplicate
         * 
         * @param exp_seq the expected sequence number
         * @return ret  returns true if valid
         */
        bool is_valid (uint16_t exp_seq) {
            return true;
        }
};

/**
 * @brief This struct is the wrapper for the timer that will be used in this code. Currently, it
            uses the linux timerfd
 * 
 */
struct uno_timer {
    public:
        int fd;
        bool is_armed;
        struct itimerspec last_config;

        /**
         * @brief Construct a new uno timer object. Creates the fd and initialises the fields
         * 
         */
        uno_timer (void) {
            //create the fd
            fd = timerfd_create (CLOCK_MONOTONIC, 0);
            if(fd == UNO_TRANSPORT_ERR) {
                //raise exception
                ;
            }
            is_armed = false;
            memset (&last_config, 0, sizeof(last_config));
        }

        /**
         * @brief Destroy the uno timer object. Closes the fd and clears the fields
         * 
         */
        ~uno_timer (void) {
            is_armed = false;
            memset (&last_config, 0, sizeof(last_config));
            close(fd);
            fd = UNO_TRANSPORT_ERR;
        }

        /**
         * @brief - sets the timer of this object
         * 
         * @param timer_val - the const struct timespec structure containing the timer delay value
         * @return int - returns 0 on success, UNO_TRANSPORT_ERR on failure
         */
        int arm_non_recurring (const struct timespec &timer_val) {
            assert (timer_val.tv_sec>0 && timer_val.tv_nsec>0);
            struct itimerspec val   = {{0,},};
            val.it_value.tv_sec     = timer_val.tv_sec;
            val.it_value.tv_nsec    = timer_val.tv_nsec;

            //arm the timer with the given value
            int ret = timerfd_settime (fd, 0, &val, NULL);
            if(ret == UNO_TRANSPORT_ERR) {
                perror("timerfd_settime");
            }
            else {
                //update the fields of the struct
                is_armed = true;
                memcpy (&last_config, &val, sizeof(val));
            }
            return ret;
        }

        /**
         * @brief - clears the timer of this object
         * 
         * @return int - returns 0 on success, UNO_TRANSPORT_ERR on failure
         */
        int disarm (void) {
            //just returned if not armed
            if(is_armed == false) {
                return 0;
            }

            struct itimerspec val   = {{0,},};

            //disarm the timer
            int ret = timerfd_settime (fd, 0, &val, NULL);
            if(ret == UNO_TRANSPORT_ERR) {
                perror("timerfd_settime");
            }
            else {
                //update the fields of the struct
                is_armed = false;
                memcpy (&last_config, &val, sizeof(val));
            }
            return ret;
        }
};

/**
 * @brief the enum class of the msg types in the cmd field of the header
 * 
 */
enum class _uno_msg_type {
    UNO_MSG_CTRL,
    UNO_MSG_ACK,
    UNO_MSG_DATA,
    UNO_MSG_SPL,
    
    UNO_MSG_UNKNOWN,
};
/**************************************************************************************************************************/
//static function declarations - descriptions will be placed above the definitions
static int _uno_create_client_fd (int broadcast);
static int _uno_create_server_fd (uint16_t port);
static ssize_t _uno_send (int fd, uint8_t *hdr, const void *msg, size_t len, const struct sockaddr_in *dst, uint16_t *seq);
static ssize_t _uno_timed_recv (int fd, uint8_t *hdr, void *buffer, size_t buffer_size, int flags, struct sockaddr_in *src, int to_ms);
static ssize_t _uno_recv (int fd, uint8_t *hdr, void *buffer, size_t buffer_size, int flags, struct sockaddr_in *src);
static _uno_msg_type _uno_get_msg_type (uint8_t cmd);
static void _create_new_connection (int fd, std::list<UnoConnection> &client_list, const struct sockaddr_in &src, 
                    uno_hdr &hdr, uint16_t ka_dur, uint8_t max_conn, const struct timespec &curr_time);
static int _uno_get_port_from_fd (int fd);
static int _uno_process_server_msg (int fd, uno_hdr &hdr, uint8_t *buffer, ssize_t bytes, 
                        std::list<UnoConnection> &client_list, uint16_t ka_dur, const struct sockaddr_in &src, 
                        uint8_t max_conn, const struct timespec &curr_time, bool &is_spl);
static int _uno_process_client_msg (int fd, uno_hdr &hdr, uint8_t *buffer, ssize_t bytes, std::list<UnoConnection> &client_list,
                        const struct sockaddr_in &src);
static int _uno_send_server_full_rsp (int fd, const struct sockaddr_in &src, uno_hdr &hdr);
static int _uno_send_conn_close (int fd, const struct sockaddr_in &dst, uint16_t *seq);
static void _uno_close_connection_requested (int fd, std::list<UnoConnection> &client_list);
static void _uno_close_connection_requested_iter (std::list<UnoConnection>::iterator iter, std::list<UnoConnection> &client_list);
static std::list<UnoConnection>::iterator _find_client_in_list (std::list<UnoConnection> &client_list, 
                const struct sockaddr_in &dst);
static std::list<UnoConnection>::iterator _find_client_in_list (std::list<UnoConnection> &client_list, int fd);
static int _uno_client_process_recv_msg (uno_hdr &hdr, uint8_t *buffer, ssize_t bytes, uint16_t &server_seq);
static int _uno_send_reliable (int fd, uint8_t *hdr_ser, const void *msg, size_t len, 
                                const struct sockaddr_in &dst, uint16_t *seq, uint8_t max_retries, uint16_t retry_interval);
static bool _uno_is_reliable_msg (uint8_t cmd_flags);
static int _uno_send_ack (int fd, const struct sockaddr_in &dst, const uno_hdr &hdr);
static bool _is_seq_valid (int fd, uint16_t inc_seq, uint16_t &saved_seq);
static int _uno_send_conn_close_expl (int fd, const struct sockaddr_in &dst, uint16_t *seq);
static int _uno_send_conn_close_to (int fd, const struct sockaddr_in &dst, uint16_t *seq);
static std::list<UnoConnection>::iterator _uno_find_most_stale_connection (std::list<UnoConnection> &client_list);
static void _uno_compute_timer_value (const struct timespec &last_msg_time, uint16_t keepalive_dur, 
                                        const struct timespec &curr_time, struct timespec &_timer_val);
/**************************************************************************************************************************/
//code specific to UnoConnection
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
UnoConnection :: UnoConnection (int fd, const struct sockaddr_in &cli, uno_hdr &hdr, uint16_t ka_dur, 
                const struct timespec &curr_time)
{
    this->fd = _uno_create_client_fd (false);
    if(this->fd == UNO_TRANSPORT_ERR) {
        ;
        //throw exception here
    }

    //get the self addr info from the fd
    int temp = _uno_get_port_from_fd (this->fd);
    if(temp == UNO_TRANSPORT_ERR) {
        //throw exception here
    }
    self_port   = (uint16_t) temp;    
    //update in structures
    memcpy (&cli_addr, &cli, sizeof(cli_addr));

    //send the new connection info to the client using the argument fd
    this->connected = true;
    this->send_connect_rsp (hdr, ka_dur);
    this->client_seq = 0;
    this->close_reason = _close_reason::CLOSE_REASON_UNKNOWN;
    
    //set the last msg time as the provided curr time
    memcpy (&(this->last_msg_time), &curr_time, sizeof(curr_time));
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf (stderr, UNO_DBG_PREFIX "created new connection fd=%d port=%" PRIu16 
                        " tv_sec=%ld tv_nsec=%ld" UNO_DBG_SUFFIX, this->fd, self_port, 
                        this->last_msg_time.tv_sec, this->last_msg_time.tv_nsec);
    #endif
}

/**
* @brief Construct a new Uno Connection object by copying the given one
* 
* @param the object from which this one is to be copied from 
*/
UnoConnection :: UnoConnection (const UnoConnection &cpy)
{
    this->fd = cpy.fd;
    this->self_port = cpy.self_port;
    memcpy (&(this->cli_addr), &(cpy.cli_addr), sizeof(cpy.cli_addr));
    this->client_seq = 0;
    memset(&(this->last_msg_time), 0, sizeof(this->last_msg_time));
    this->connected = cpy.connected;
    this->close_reason = cpy.close_reason;
}

/**
* @brief Destroy the Uno Connection object. Sends connection close req to the client if the 
        client is still connected
* 
*/

UnoConnection :: ~UnoConnection (void)
{
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf(stderr, UNO_DBG_PREFIX "in UnoConnection destructor for fd=%d: connected=%d" UNO_DBG_SUFFIX, fd, connected);
    #endif
    if(this->connected) {
        assert (this->close_reason != _close_reason::CLOSE_REASON_UNKNOWN);
        switch(this->close_reason) {
            //no need to send anything if close was requested by client
            case _close_reason::CLOSE_REASON_REQUESTED_BY_CLIENT:
                break;
            //send explicit connection closure message
            case _close_reason::CLOSE_REASON_REQUESTED_BY_SERVER:
                _uno_send_conn_close_expl (this->fd, this->cli_addr, nullptr);
                break;
            //send connection timedout message
            case _close_reason::CLOSE_REASON_TIMEDOUT:
                _uno_send_conn_close_to (this->fd, this->cli_addr, nullptr);
                break;
            case _close_reason::CLOSE_REASON_UNKNOWN:
                assert (0);
                break;
        }
        this->connected = false;
        this->close_reason = _close_reason::CLOSE_REASON_UNKNOWN;
    }
    close (this->fd);
    this->fd = UNO_TRANSPORT_ERR;
    this->connected = false;
    memset (&(this->self_port), 0, sizeof(this->self_port));
    memset (&(this->cli_addr),  0, sizeof(this->cli_addr));
    this->client_seq = 0;
    memset(&(this->last_msg_time), 0, sizeof(this->last_msg_time));
}
/**
* @brief Sends a connection response to the device sending the connection request
* 
* @param hdr info in the header struct
* @param ka_dur the keepalive duration
* @return int retuns number of bytes sent on success, UNO_TRANSPORT_ERR on failure
*/

int UnoConnection :: send_connect_rsp (uno_hdr &hdr, uint16_t ka_dur)
{
    uint8_t data[UNO_TRANSPORT_CTRL_CONN_RSP_LEN] = {0,};
    //cmd id
    data[0] = UNO_TRANSPORT_CTRL_CONN_RSP;
    //port number - in nwk byte order
    data[1] = (self_port >> 8) & 0xFF;
    data[2] = (self_port >> 0) & 0xFF;
    //keep alive - in nwk byte order
    data[3] = (ka_dur >> 8) & 0xFF;
    data[4] = (ka_dur >> 0) & 0xFF;

    hdr.msg_len = sizeof(data);
    uint8_t hdr_ser[UNO_TRANSPORT_HDR_LEN];
    hdr.serialise(hdr_ser);

    return _uno_send (fd, hdr_ser, data, sizeof(data), &cli_addr, NULL);
}

/**
 * @brief Set the close reason variable
 * 
 * @param reason 
 */
void UnoConnection :: set_close_reason (_close_reason reason)
{
    this->close_reason = reason;
}

/**************************************************************************************************************************/
//code specific to UnoTransportClient
/**
 * @brief The default constructor for the client class, creates a UDP socket at a random port
 * 
 * @return UnoTransportClient 
 */
UnoTransportClient :: UnoTransportClient (void)
{
    fd = _uno_create_client_fd (0);
    if(fd == UNO_TRANSPORT_ERR) {
        //raise exception here
    }
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf (stderr, UNO_DBG_PREFIX "created client with fd = %d broadcast = 0" UNO_DBG_SUFFIX, fd);
    #endif
    seq = 0;
    memset (&addr, 0, sizeof(addr));
    can_broadcast = false;
    connected = false;
    self_port = (uint16_t) _uno_get_port_from_fd (fd);
}

/**
 * @brief Constructor with capability to create a broadcast socket.
 * 
 * @param broadcast set this to true if you want the socket to be able to broadcast
 * @return UnoTransportClient N/A
 */
UnoTransportClient :: UnoTransportClient (bool broadcast)
{
    //create a socket with broadcast permissions
    int opt = (broadcast==true) ? (1) : (0);
    fd = _uno_create_client_fd (opt);
    if(fd == -1) {
        //raise exception here
    }
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf (stderr, UNO_DBG_PREFIX "created client with fd = %d broadcast = %d" UNO_DBG_SUFFIX, fd, broadcast);
    #endif
    //initialise other fields
    seq = 0;
    memset (&addr, 0, sizeof(addr));
    can_broadcast = broadcast;
    connected = false;
    self_port = (uint16_t) _uno_get_port_from_fd (fd);
}

/**
 * @brief Destructor for the client class, will close the socket and reset all other fields
 * 
 * @return UnoTransportClient N/A
 */
UnoTransportClient :: ~UnoTransportClient (void)
{
    //if connected, disconnect
    if(connected) {
        #ifdef UNO_TRANSPORT_DEBUG
            fprintf (stderr, UNO_DBG_PREFIX "closing connection in client destructor. fd=%d conn_port=%" PRIu16 UNO_DBG_SUFFIX,
                            fd, ntohs(addr.sin_port));
        #endif
        _uno_send_conn_close (fd, addr, &seq);
    }
    //close the socket
    if(fd != UNO_TRANSPORT_ERR) {
        close (fd);
        fd = UNO_TRANSPORT_ERR;
    }
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf (stderr, UNO_DBG_PREFIX "destroyed client" UNO_DBG_SUFFIX);
    #endif
    //clear other fields
    seq = 0;
    memset (&addr, 0, sizeof(addr));
    can_broadcast = false;
    connected = false;
    self_port = 0;
}

/**
* @brief get the port number of the connection (on the server side) (TO BE REMOVED)
* 
* @return uint16_t the port number value
*/
uint16_t UnoTransportClient :: _dbg_get_port (void)
{
    return ntohs(this->addr.sin_port);
}

/**
* @brief get the port number of the client side socket (TO BE REMOVED)
* 
* @return uint16_t the port number
*/
uint16_t UnoTransportClient :: _dbg_get_self_port (void) 
{
    return self_port;
}

/**
* @brief Connect to the specified server
* 
* @param dst - reference to the struct sockaddr_in instance containing the server's IPv4 address
* @param to_ms - the timeout in milliseconds
* @return int - 0 if successful, UNO_TRANSPORT_ERR if failed
*/
int UnoTransportClient :: connect (const struct sockaddr_in &dst, int to_ms)
{
    struct sockaddr_in src;
    uint8_t hdr_ser[UNO_TRANSPORT_HDR_LEN] = {0,};
    uint8_t cmd_flags = 0;

    //this is the data section for the connection request command
    uint8_t data[UNO_TRANSPORT_CTRL_CONN_REQ_LEN];
    data[0] = UNO_TRANSPORT_CTRL_CONN_REQ;  //command id
    data[1] = UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_OPEN;    //command mode
    
    //set the control msg bit in the cmd field of the hdr and serialise the hdr
    cmd_flags = UNO_SET_BIT(cmd_flags, UNO_TRANSPORT_CMD_CONTROL_MSG);
    uno_hdr hdr (seq, cmd_flags, sizeof(data));

    //send this msg out
    hdr.serialise (hdr_ser);
    int ret = _uno_send (this->fd, hdr_ser, data, sizeof(data), &dst, &seq);
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf(stderr, UNO_DBG_PREFIX "sent connection req %d bytes to fd=%d port %hu" UNO_DBG_SUFFIX, ret, 
                            this->fd, ntohs(dst.sin_port));
    #endif

    //todo - check source address, and continue waiting if interrupted by other message
    if(ret != UNO_TRANSPORT_ERR) {
        memset (&src, 0, sizeof(src));
        memset (hdr_ser, 0, sizeof(hdr_ser));
        uint8_t data[UNO_TRANSPORT_CTRL_MSG_MAX_SIZE] = {0,};
        ret = _uno_timed_recv (this->fd, hdr_ser, data, sizeof(data), MSG_WAITALL, &src, to_ms);
        hdr.deserialise (hdr_ser);

        //process connection response

        if(_uno_get_msg_type(hdr.cmd) == _uno_msg_type::UNO_MSG_CTRL && 
                data[0] == UNO_TRANSPORT_CTRL_CONN_RSP) {
            //if msg len is 1, then the server is full/connection refused
            if(hdr.msg_len == 1) {
                ret = UNO_TRANSPORT_CONN_REFUSED;
                #ifdef UNO_TRANSPORT_DEBUG
                    fprintf(stderr, UNO_DBG_PREFIX "received conn refused" UNO_DBG_SUFFIX);
                #endif
            }
            else {
                //save the connection details
                memcpy (&(this->addr), &src, sizeof(src));

                //2 byte port in nwk byte order, save port in nwk byte order
                memcpy (&(this->addr.sin_port), data+1, 2);

                //2 byte keepalive duration in nwk byte order
                this->keepalive_dur  = 0;
                this->keepalive_dur |= (data[3] << 8);
                this->keepalive_dur |= (data[4] << 0);
                #ifdef UNO_TRANSPORT_DEBUG
                    fprintf(stderr, UNO_DBG_PREFIX "received connection rsp %d bytes fd=%d port=%" PRIu16 
                                    " ka=%" PRIu16 UNO_DBG_SUFFIX, ret, this->fd, ntohs(this->addr.sin_port), this->keepalive_dur);
                #endif
                ret = 0;
                connected = true;
            }
        }

    }

    return ret;
}

/**
* @brief Send a mesage to the server that this client is connected to
* 
* @param msg - void pointer to the mesage
* @param len - length of the message
* @return int - returns UNO_TRANSPORT_ERR on failure, or number of bytes sent on success
*/
int UnoTransportClient :: send_msg (const void *msg, size_t len)
{
    assert (msg!=nullptr);
    assert (len<=UNO_TRANSPORT_MTU);
    if(msg==nullptr || len > UNO_TRANSPORT_MTU) {
        return UNO_TRANSPORT_ERR;
    }

    if(this->connected==false) {
        return UNO_TRANSPORT_ERR;
    }

    uint8_t hdr_ser[UNO_TRANSPORT_HDR_LEN] = {0,};
    // uint8_t cmd_flags = 0;
    uno_hdr hdr (seq, 0, len);
    hdr.serialise (hdr_ser);

    int ret = _uno_send (this->fd, hdr_ser, msg, len, &addr, &seq);
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf(stderr, UNO_DBG_PREFIX "send message of len=%d through fd=%d: ret=%d" UNO_DBG_SUFFIX, len, this->fd, ret);
    #endif
    return ret;
}

/**
* @brief Sends the given message to the server, expects an ACK from the server.
* 
* @param msg - void pointer to the message
* @param len - length of the message
* @return int - returns 
                    -> positive number of bytes on successful reliable send
                    -> 0 if the message was sent successfully, but no ACK was received
                    -> UNO_TRANSPORT_ERR on general failure
*/
int UnoTransportClient :: send_msg_reliable (const void *msg, size_t len)
{
    assert (msg!=nullptr);
    assert (len<=UNO_TRANSPORT_MTU);
    if(msg==nullptr || len > UNO_TRANSPORT_MTU) {
        return UNO_TRANSPORT_ERR;
    }

    if(this->connected==false) {
        return UNO_TRANSPORT_ERR;
    }

    //need to set the reliable msg bit in the cmd section of the header
    uint8_t hdr_ser[UNO_TRANSPORT_HDR_LEN] = {0,};
    uint8_t cmd_flags = 0;
    cmd_flags = UNO_SET_BIT (cmd_flags, UNO_TRANSPORT_CMD_RELIABLE_MSG);
    uno_hdr hdr (seq, cmd_flags, len);
    hdr.serialise (hdr_ser);
    int ret;

    //send out the message
    ret = _uno_send_reliable (this->fd, hdr_ser, msg, len, addr, &seq, UNO_TRANSPORT_REL_MAX_RETRIES,
                            UNO_TRANSPORT_REL_RETRY_INT_MS);
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf(stderr, UNO_DBG_PREFIX "sen reliable message of len=%d through fd=%d: ret=%d" UNO_DBG_SUFFIX, len, this->fd, ret);
    #endif
    if(ret == UNO_TRANSPORT_CONN_CLOSE_RECVD) {
        //close the socket and clear all fields
        close(this->fd);
        memset (&(this->addr), 0, sizeof(this->addr));
        this->seq = 0;
        this->server_seq = 0;
        this->connected = 0;
        this->can_broadcast = 0;
        this->self_port = 0;
    }
    return ret;
}

/**
* @brief Receives a message from the connected server into the specified server
* 
* @param buffer - The buffer where the message is to be written
* @param buffer_size - The size of this buffer 
* @param to_ms - The timeout in milliseconds
* @return int - The number of bytes received
*/
int UnoTransportClient :: recv_msg (void *buffer, size_t buffer_size, int to_ms)
{
    if(buffer == nullptr || buffer_size == 0) {
        return UNO_TRANSPORT_ERR;
    }
    if(connected == false) {
        return UNO_TRANSPORT_CLIENT_NOT_CONN;
    }
    
    uint8_t hdr_ser[UNO_TRANSPORT_HDR_LEN];
    bool exit_condition = false;
    ssize_t bytes;
    int ret = UNO_TRANSPORT_ERR;
    uno_hdr hdr;

    //loop until we get a message intended for the caller
    while(!exit_condition) {
        memset (hdr_ser, 0, sizeof(hdr_ser));
        /*
        TODO - reduce to_ms after every iteration of the loop, the whole function should not take more
                than to_ms milliseconds. This current version will keep waiting to_ms milliseconds on every
                iteration
        */
        bytes = _uno_timed_recv (this->fd, hdr_ser, buffer, buffer_size, MSG_WAITALL, NULL, to_ms);

        //timedout
        if(bytes == 0) {
            ret = 0;
            break;
        }
        
        //every message must be atleast UNO_TRANSPORT_HDR bytes long
        if(bytes < UNO_TRANSPORT_HDR_LEN) {
            //drop the messsage, continue
            continue;
        }

        //check the validity of the sequence number
        if(_is_seq_valid (this->fd, hdr.seq_id, server_seq) == false) {
            //drop the packet;
            continue;
        }

        //send ack if reliable message
        hdr.deserialise (hdr_ser);
        if(_uno_is_reliable_msg(hdr.cmd)) {
            _uno_send_ack (this->fd, addr, hdr);
        }

        //process the message, now that we know it has a valid fmt
        bytes = _uno_client_process_recv_msg (hdr, static_cast<uint8_t*>(buffer), bytes, server_seq);
        if(bytes == UNO_TRANSPORT_CONN_CLOSE_RECVD) {
            #ifdef UNO_TRANSPORT_DEBUG
                fprintf(stderr, UNO_DBG_PREFIX "received conn close req from server" UNO_DBG_SUFFIX);
            #endif
            ret = (int)bytes;
            exit_condition = true;

            //close the socket and clear all fields
            close(this->fd);
            memset (&(this->addr), 0, sizeof(this->addr));
            this->seq = 0;
            this->server_seq = 0;
            this->connected = 0;
            this->can_broadcast = 0;
            this->self_port = 0;
        }
        //if unknown message, ignore and continue waiting
        else if (bytes == UNO_TRANSPORT_ERR) {
            continue;
        }
        //for positive bytes, this is a data message that is to be delivered to the caller
        else if (bytes > 0) {
            ret = (int) (bytes - UNO_TRANSPORT_HDR_LEN);
        }
        //ack messages should not be caught in this function, drop them
        else {
            // assert (false && "this should never happen");
            continue;
        }
        exit_condition = true;
    }
    return ret;

}

/**
 * @brief - Sends a keepalive message to the server
 * 
 * @return int - returns 0 on success, UNO_TRANSPORT_ERR on failure, UNO_TRANSPORT_CLIENT_NOT_CONN if not connected
 */
int UnoTransportClient :: send_keepalive (void)
{
    if(connected == false) {
        return UNO_TRANSPORT_CLIENT_NOT_CONN;
    }

    uint8_t hdr_ser[UNO_TRANSPORT_HDR_LEN] = {0,};
    uint8_t cmd_flags = 0;

    //this is the data section for the connection request command
    uint8_t data[UNO_TRANSPORT_CTRL_KEEPALIVE_LEN];
    data[0] = UNO_TRANSPORT_CTRL_KEEPALIVE;  //command id
    
    //set the control msg bit in the cmd field of the hdr and serialise the hdr
    cmd_flags = UNO_SET_BIT(cmd_flags, UNO_TRANSPORT_CMD_CONTROL_MSG);
    uno_hdr hdr (seq, cmd_flags, sizeof(data));

    //send this msg out
    hdr.serialise (hdr_ser);
    int ret = _uno_send (this->fd, hdr_ser, data, sizeof(data), &addr, &seq);
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf(stderr, UNO_DBG_PREFIX "sent keepalive ret=%d, fd=%d port %hu" UNO_DBG_SUFFIX, ret, 
                            this->fd, ntohs(addr.sin_port));
    #endif
    
    return (ret==UNO_TRANSPORT_ERR) ? (UNO_TRANSPORT_ERR) : (0);
}
/**************************************************************************************************************************/
//code specific to the UnoTransportServer class
/**
* @brief Construct a new Uno Transport Server object. Does not create a socket
* 
*/
UnoTransportServer :: UnoTransportServer(void)
{
    //intialise all fields, don't create socket without port specified
    fd = UNO_TRANSPORT_ERR;
    seq = 0;
    keepalive_dur = UNO_TRANSPORT_KEEPALIVE_INT;
    max_conn = UNO_TRANSPORT_DFL_MAX_CONN;
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf (stderr, UNO_DBG_PREFIX "created server WITHOUT socket" UNO_DBG_SUFFIX);
    #endif
    // client_list.reserve (max_conn);
}

/**
* @brief Construct a new Uno Transport Server object. Creates a socket with the specified port
* 
* @param port - the port number of this server socket
*/
UnoTransportServer :: UnoTransportServer(uint16_t port)
{
    keepalive_dur = UNO_TRANSPORT_KEEPALIVE_INT;
    max_conn = UNO_TRANSPORT_DFL_MAX_CONN;
    create_socket (port);
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf (stderr, UNO_DBG_PREFIX "created server at port=%hu" UNO_DBG_SUFFIX, port);
    #endif
    seq = 0;
}

/**
* @brief Construct a new Uno Transport Server object. Creates a socket with the specified port
* 
* @param port - the port number of this server socket
* @param max_connections - the max number of connections supported by this socket
*/
UnoTransportServer :: UnoTransportServer (uint16_t port, uint8_t max_connections)
{
    keepalive_dur = UNO_TRANSPORT_KEEPALIVE_INT;
    max_conn = max_connections;
    create_socket (port);
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf (stderr, UNO_DBG_PREFIX "created server at port=%hu" UNO_DBG_SUFFIX, port);
    #endif
    seq = 0;
}

/**
* @brief Destroy the Uno Transport Server object. Sends connection close message to all connected clients
        and closes the server side socket
* 
*/
UnoTransportServer :: ~UnoTransportServer(void)
{
    //close the socket
    if(fd != UNO_TRANSPORT_ERR) {
        close (fd);
        fd = UNO_TRANSPORT_ERR;
    }
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf (stderr, UNO_DBG_PREFIX "destroyed server" UNO_DBG_SUFFIX);
    #endif
    //clear other fields
    seq = 0;
}

/**
* @brief Create a socket object at the specified port
* 
* @param port - the port number of this server socket
* @return int - the fd number
*/
int UnoTransportServer :: create_socket (uint16_t port)
{
    fd = _uno_create_server_fd (port);
    if(fd == UNO_TRANSPORT_ERR) {
        return fd;
    }
    seq = 0;
    return 0;
}

/**
* @brief Receives the incoming message in the specified buffer. Does not bother the caller with the protocol specific
            packets. Handles connection requests, acks, etc
* 
* @param buffer - the buffer to store the incoming msg
* @param buffer_size - the size of this buffer
* @param src - optional pointer to the struct sockaddr_in object where the source address is to be filled in
* @param is_spl - reference to the bool variable that will be set if the incoming message is a spl message
* @return int - number of bytes received
*/
int UnoTransportServer :: recv_msg (void *buffer, size_t buffer_size, struct sockaddr_in *src_arg, bool &is_spl)
{
    uint8_t hdr_ser[UNO_TRANSPORT_HDR_LEN] = {0,};
    struct sockaddr_in src = {0,};
    std::vector<struct pollfd> pfds;
    ssize_t bytes;
    bool exit_condition=false;
    int success_count=0;
    int ret;
    uno_hdr hdr;
    struct timespec curr_time;
    uno_timer timer;
    static std::list<UnoConnection>::iterator _most_stale;
    struct timespec _timer_val;

    while(!exit_condition) {
        timer.disarm();
        //clear all data structures
        pfds.clear();   //this can probably be optimised
        memset (&src, 0, sizeof(src));
        memset (hdr_ser, 0, sizeof(hdr_ser));

        //build the vector for poll
        //first fd is always the server fd
        pfds.push_back ({.fd = this->fd, .events = POLLIN});

        //note down curr time - we use clock monotonic here because we're not bothered about the actual time
        clock_gettime (CLOCK_MONOTONIC, &curr_time);
        //find the connection that is going to timeout first
        _most_stale = _uno_find_most_stale_connection (client_list);
        if(_most_stale != client_list.end()) {
            //determine the time remaining till timeout
            memset (&_timer_val, 0, sizeof(_timer_val));
            _uno_compute_timer_value (_most_stale->last_msg_time, keepalive_dur, curr_time, _timer_val);
            timer.arm_non_recurring (_timer_val);
        }
        //second fd will be the timer fd - it does nothing if it is not armed, so no issue
        pfds.push_back ({.fd = timer.fd, .events = POLLIN});

        //add the other fd's after this
        for(auto iter=this->client_list.begin(); iter!=this->client_list.end(); ++iter) {
            pfds.push_back ({.fd = iter->fd, .events = POLLIN});
        }
        #ifdef UNO_TRANSPORT_DEBUG
            fprintf (stderr, UNO_DBG_PREFIX "pollfd structure has %d fd's" UNO_DBG_SUFFIX, pfds.size());
        #endif

        //poll on this set of fd's
        ret = poll (pfds.data(), pfds.size(), UNO_TRANSPORT_TIMEOUT_INFINITE);
        if(ret == UNO_TRANSPORT_ERR) {
            perror("poll");
            exit_condition = true;
        }
        else if (ret == 0) {
            //this must never happen since we're polling for an indefinite time period
            assert (false && "This must never happen");
        }
        else {
            //note down curr time - to minimise number of syscalls
            clock_gettime (CLOCK_MONOTONIC, &curr_time);
            //read from all fd's with data
            success_count = 0;
            int index=0;
            for(auto iter=pfds.begin(); iter!=pfds.end(), success_count!=ret; index++, iter++) {
                //ding ding ding, we have a winner!!!
                if((iter->revents) & POLLIN) {

                    //check if this fd is the timer fd
                    if(iter->fd == timer.fd) {
                        assert (_most_stale != this->client_list.end());
                        //then close the most stale connection
                        _most_stale->set_close_reason (_close_reason::CLOSE_REASON_TIMEDOUT);
                        _uno_close_connection_requested_iter (_most_stale, client_list);
                        break;
                    }

                    //receive data from this fd
                    bytes = _uno_recv (iter->fd, hdr_ser, buffer, buffer_size, MSG_WAITALL, &src);
                    success_count++;

                    //the message must atleast by UNO_TRANSPORT_HDR_LEN bytes
                    if(bytes < UNO_TRANSPORT_HDR_LEN) {
                        //invalid msg, drop the packet
                        continue;
                    }

                    //deserialise the header
                    hdr.deserialise (hdr_ser);
                    //check if reliable msg                    
                    if(_uno_is_reliable_msg(hdr.cmd)) {
                        _uno_send_ack (this->fd, src, hdr);
                    }

                    //check if fd that's ready is the server fd
                    if(iter==pfds.begin()) {
                        bytes = _uno_process_server_msg (iter->fd, hdr, static_cast<uint8_t*>(buffer), bytes, 
                                        this->client_list, this->keepalive_dur, src, max_conn, curr_time, is_spl);
                    }

                    //else it must be one of the client conneciton fd's
                    else {
                        //find the client in the list
                        auto _this_client = _find_client_in_list (this->client_list, iter->fd);
                        //update the last message time
                        memcpy (&(_this_client->last_msg_time), &curr_time, sizeof(curr_time));
                        //check the validity of the sequence number
                        if(_is_seq_valid (iter->fd, hdr.seq_id, _this_client->client_seq) == false) {
                            //drop the packet;
                            continue;
                        }
                        bytes = _uno_process_client_msg (iter->fd, hdr, static_cast<uint8_t*>(buffer), bytes, 
                                        this->client_list, src);
                    }
                    //since our process function returns positive values if the messages are data messages
                    if(bytes > 0) { 
                        ret = (int) bytes;
                        exit_condition = true;
                        if(src_arg) { memcpy (src_arg, &src, sizeof(src)); }
                        break;
                    }
                }   //end of if((iter->revents) & POLLIN)
            }       //end of for loop
        }           //end of else block
    }               //end of while(1) loop
    return ret;
}

/**
* @brief Sends the given message to the specified client, if connected
* 
* @param msg - void pointer to the message
* @param len - the length of the message
* @param dst - const reference to the structure containing the destination
* @return int - returns number of bytes sent on success, UNO_TRANSPORT_CLIENT_NOT_CONN if the client is not found
            in the client table, UNO_TRANSPORT_ERR on any general purpose failures
*/
int UnoTransportServer :: send_msg (const void *msg, size_t len, const struct sockaddr_in &dst)
{
    if(msg==nullptr) {
        return UNO_TRANSPORT_ERR;
    }
    if(len == 0 ) {
        return len;
    }

    //check if we can find this destination address in the list
    auto iter = _find_client_in_list (client_list, dst);
    if(iter == client_list.end()) {
        return UNO_TRANSPORT_CLIENT_NOT_CONN;
    }

    //since this is a data message, no bits are to be set in the cmd_info section of the header
    uint8_t hdr_ser[UNO_TRANSPORT_HDR_LEN] = {0,};
    // uint8_t cmd_flags = 0;
    uno_hdr hdr (seq, 0, len);
    hdr.serialise (hdr_ser);

    //send this header and the message
    int ret = _uno_send (iter->fd, hdr_ser, msg, len, &dst, &seq);
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf(stderr, UNO_DBG_PREFIX "send message of len=%d through fd=%d: ret=%d" UNO_DBG_SUFFIX, len, iter->fd, ret);
    #endif
    return ret;
}

/**
* @brief Sends the given message to the given client (if connected), expects an ACK from the recipient.
* 
* @param msg - void pointer to the message
* @param len - length of the message
* @return int - returns 
                    -> positive number of bytes on successful reliable send
                    -> 0 if the message was sent successfully, but no ACK was received
                    -> UNO_TRANSPORT_ERR on general failure
*/
int UnoTransportServer :: send_msg_reliable (const void *msg, size_t len, const struct sockaddr_in &dst)
{
    if(msg==nullptr) {
        return UNO_TRANSPORT_ERR;
    }
    if(len == 0 ) {
        return len;
    }

    //check if we can find this destination address in the list
    auto iter = _find_client_in_list (client_list, dst);
    if(iter == client_list.end()) {
        return UNO_TRANSPORT_CLIENT_NOT_CONN;
    }

    //need to set the reliable msg bit in the cmd section of the header
    uint8_t hdr_ser[UNO_TRANSPORT_HDR_LEN] = {0,};
    uint8_t cmd_flags = 0;
    cmd_flags = UNO_SET_BIT (cmd_flags, UNO_TRANSPORT_CMD_RELIABLE_MSG);
    uno_hdr hdr (seq, cmd_flags, len);
    hdr.serialise (hdr_ser);

    //send this header and the message
    int ret = _uno_send_reliable (iter->fd, hdr_ser, msg, len, dst, &seq, UNO_TRANSPORT_REL_MAX_RETRIES,
                            UNO_TRANSPORT_REL_RETRY_INT_MS);
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf(stderr, UNO_DBG_PREFIX "send reliable message of len=%d through fd=%d: ret=%d" UNO_DBG_SUFFIX, len, iter->fd, ret);
    #endif
    if(ret == UNO_TRANSPORT_CONN_CLOSE_RECVD) {
        iter->set_close_reason (_close_reason::CLOSE_REASON_REQUESTED_BY_CLIENT);
        _uno_close_connection_requested_iter (iter, client_list);
    }
    return ret;
}

/**
* @brief Sends the connection close requet to the specified client
* 
* @param dst - reference to the structure containing the destination
* @return int - returns 0 on success, UNO_TRANSPORT_CLIENT_NOT_CONN if the client is not found
            in the client table, UNO_TRANSPORT_ERR on any general purpose failures
*/
int UnoTransportServer ::close_connection (struct sockaddr_in &dst)
{
    //first check if we can find this client in the client list
    auto iter = _find_client_in_list (client_list, dst);
    if(iter == client_list.end()) {
        return UNO_TRANSPORT_CLIENT_NOT_CONN;
    }

    int ret = _uno_send_conn_close_expl (iter->fd, dst, &seq);
    //close socket and cleanup
    iter->set_close_reason (_close_reason::CLOSE_REASON_REQUESTED_BY_SERVER);
    _uno_close_connection_requested_iter (iter, client_list);
    return ret;
}

/**************************************************************************************************************************/
            /*    STATIC FUNCTION DEFINITIONS      */
/**************************************************************************************************************************/
/**
 * @brief This function creates a client specific socket
 * 
 * @param opt - the option value for SO_BROADCAST
 * @return int - the fd value of the socket, returns UNO_TRANSPORT_ERR on failure
 */
static int _uno_create_client_fd (int opt)
{
    struct sockaddr_in addr = {0,};
    //create the socket
    int fd = socket (AF_INET, SOCK_DGRAM, 0);
    if(fd == -1) {
        perror("socket");
        return fd;
    }

    //set the socket as non blocking since we can use this to discard data
    if(fcntl(fd, F_SETFL, O_NONBLOCK) == UNO_TRANSPORT_ERR) {
        perror("fcntl");
        goto cleanup;
    }
    
    //set SO_BROADCAST
    if(setsockopt(fd, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt)) == UNO_TRANSPORT_ERR) {
        perror("setsockopt");
        goto cleanup;
    }

    //bind to address - leave port as 0, the OS will select a random port
    addr.sin_family         = AF_INET;
    addr.sin_addr.s_addr    = htonl (INADDR_ANY);
    if(bind(fd, (const struct sockaddr*)&addr, sizeof(addr)) == UNO_TRANSPORT_ERR) {
        perror("bind");
        goto cleanup;
    }

    return fd;

    //if error, close the socket to avoid fd leaks
    cleanup:
    close(fd);
    return UNO_TRANSPORT_ERR;
}

/**
 * @brief This function creates a server specific socket
 * 
 * @param port - the port to which this socket is to be bound to
 * @return int - the fd of the socket, returns UNO_TRANSPORT_ERR on failure
 */
static int _uno_create_server_fd (uint16_t port)
{
    struct sockaddr_in addr = {0,};
    //create the socket
    int fd = socket (AF_INET, SOCK_DGRAM, 0);
    if(fd == UNO_TRANSPORT_ERR) {
        perror("socket");
        return fd;
    }
    
    //set the socket as non blocking since we can use this to discard data
    if(fcntl(fd, F_SETFL, O_NONBLOCK) == UNO_TRANSPORT_ERR) {
        perror("fcntl");
        goto cleanup;
    }
    
    //bind the socket
    addr.sin_family         = AF_INET;
    addr.sin_addr.s_addr    = htonl (INADDR_ANY);
    addr.sin_port           = htons (port);

    if(bind(fd, (struct sockaddr*)&addr, sizeof(addr)) == UNO_TRANSPORT_ERR) {
        perror("bind");
        goto cleanup;
    }
    return fd;
 
    //close the fd before returning if failure
    cleanup:
    close(fd);
    return UNO_TRANSPORT_ERR;
}

/**
 * @brief THe local function for sending out a message with header and buffef, supports null buffer
 * 
 * @param fd - The fd through which this has to be sent
 * @param hdr - uint8_t pointer to the structure containing the fixed length header
 * @param msg - the const void pointer to the buffer containing the message
 * @param len - the length of the message
 * @param dst - pointer to the struct containing the destination
 * @param seq - optional pointer to the uint16_t variable tracking the sequence id, will be incremented on successful send
 * @return ssize_t - number of bytes sent on success, UNO_TRANSPORT_ERR on failure
 */
static ssize_t _uno_send (int fd, uint8_t *hdr, const void *msg, size_t len, const struct sockaddr_in *dst, uint16_t *seq)
{
    //initilase the io vector with the hdr and then the message
    //unfortunately, we need a const_cast here because of the way the sendmsg() API is.
    size_t iov_len = (msg==NULL) ? (1) : (2);
    //prepare the array of vectors, 1st one being the header, followed by the message
    struct iovec iov[] = {  {   .iov_base   = hdr,
                                .iov_len    = UNO_TRANSPORT_HDR_LEN},
                            {   .iov_base   = const_cast<void*>(msg),
                                .iov_len    = len},
    };
    //add these vectors to the message
    struct msghdr msg_hdr = {   .msg_name   = (void*)dst,
                                .msg_namelen= sizeof(*dst),
                                .msg_iov    = iov,
                                .msg_iovlen = iov_len,
    };
    ssize_t ret = sendmsg (fd, &msg_hdr, 0);
    if(ret == UNO_TRANSPORT_ERR) {perror("sendmsg");}
    else {
        //increment the sequence number, if required
        if(seq) (*seq)++;
    }
    return ret;
}

/**
 * @brief Waits for the specified number of milliseconds to receive the message. uses poll() for timeout.
            uses _uno_recv() for the actual socket recv
 * 
 * @param fd - the fd through which the message is to be received
 * @param hdr - pointer to the buffer where the header is to be filled.
 * @param buffer - the buffer where the message is to be stored
 * @param buffer_size - the size of the provided buffer
 * @param flags - the syscall specific flags that are to be passed to _uno_recv()
 * @param src - the optional struct sockaddr_in where the source address is to be stored
 * @param to_ms - the timeout in milliseconds
 * @return ssize_t - returns the number of bytes received if successful, 0 if timedout, UNO_TRANSPORT_ERR if failed
 */
static ssize_t _uno_timed_recv (int fd, uint8_t *hdr, void *buffer, size_t buffer_size, int flags, struct sockaddr_in *src, int to_ms)
{
    int ret;
    if(to_ms == 0) {
        //since it is a non blocking fd, if there is no data, the call will exit immediately
        ret = _uno_recv (fd, hdr, buffer, buffer_size, flags, src);
    }
    else {
        //poll on the specified fd
        struct pollfd pfd = {.fd = fd, .events = POLLIN};
        ret = poll (&pfd, 1, to_ms);
        if(ret == UNO_TRANSPORT_ERR) {
            //this must never happen
            perror("poll");
            return ret;
        }
        //data is ready to be read
        else if (ret>0) {         
            ret = _uno_recv (fd, hdr, buffer, buffer_size, flags, src);
        }
    }
    return ret;
}

/**
 * @brief Receives the message into the given header buffer and message buffer provided using recvmsg()
 * 
 * @param fd - the fd through which the data is to be received
 * @param hdr - uint8_t pointer to the buffer where the fixed length header is to be received
 * @param buffer - the buffer where the mesasge is to be received
 * @param buffer_size - the size of the message buffer
 * @param flags - flags to be passed to recvmsg()
 * @param src - optional pointer to the struct sockaddr_in where the source address is to be filled
 * @return ssize_t - returns number of bytes received if successful, UNO_TRANSPORT_ERR if failed
 */
static ssize_t _uno_recv (int fd, uint8_t *hdr, void *buffer, size_t buffer_size, int flags, struct sockaddr_in *src)
{
    assert (fd > 2);    //since fd's 0,1,2 are reserved
    struct iovec iov[] = {{.iov_base = hdr, .iov_len = UNO_TRANSPORT_HDR_LEN}, {.iov_base = buffer, .iov_len = buffer_size}};
    struct msghdr msg = {       .msg_name   = (void*)src,
                                .msg_namelen= sizeof(*src),
                                .msg_iov    = iov,
                                .msg_iovlen = 2,

    };

    ssize_t ret = recvmsg (fd, &msg, flags);
    if(ret == UNO_TRANSPORT_ERR) {
        perror("recvmsg");
    }
    return ret;
}

/**
 * @brief Returns the enum class value of _uno_msg_type by examining the cmd field of the header
 * 
 * @param cmd The uint8_t value of the cmd_flags
 * @return _uno_msg_type - the type of message that this is
 */
static _uno_msg_type _uno_get_msg_type (uint8_t cmd)
{
    _uno_msg_type ret = _uno_msg_type :: UNO_MSG_UNKNOWN;

    //check ACK bit
    if(UNO_IS_BIT_SET(cmd, UNO_TRANSPORT_CMD_ACK_MSG)) {
        ret = _uno_msg_type :: UNO_MSG_ACK;
    }
    //check control bit
    else if(UNO_IS_BIT_SET(cmd, UNO_TRANSPORT_CMD_CONTROL_MSG)) {
        //ensure that spl bit is not set
        if(!(UNO_IS_BIT_SET(cmd, UNO_TRANSPORT_CMD_SPL_MSG))) {
            ret = _uno_msg_type :: UNO_MSG_CTRL;
        }
    }
    //check spl bit
    else if (UNO_IS_BIT_SET(cmd, UNO_TRANSPORT_CMD_SPL_MSG)) {
        //ensure that control bit is not set
        if(UNO_IS_BIT_SET(cmd, UNO_TRANSPORT_CMD_CONTROL_MSG)) {
            ret = _uno_msg_type :: UNO_MSG_SPL;
        }
    }
    else {
        ret = _uno_msg_type :: UNO_MSG_DATA;
    }

    assert (ret != _uno_msg_type::UNO_MSG_UNKNOWN);
    return ret;
}

/**
 * @brief Creates a new connection in the client list, or sends a server full message
 * 
 * @param fd - The fd of the server
 * @param client_list - reference to the client list
 * @param src - reference to the sockaddr_in struct containing the source address
 * @param hdr - reference to the structure containing the header
 * @param ka_dur - the keepalive duration that is to be sent to the client
 * @param max_conn - the max connections supported
 * @param curr_time - const reference to the struct timespec containing this message's time
 */
static void _create_new_connection (int fd, std::list<UnoConnection> &client_list, const struct sockaddr_in &src, 
                    uno_hdr &hdr, uint16_t ka_dur, uint8_t max_conn, const struct timespec &curr_time)
{   
    bool found = false;
    int index = 0;

    //check if client already exists in the list - if so reuse the connection
    for(auto iter=client_list.begin(); iter!=client_list.end(); ++iter, index++) {
        if(memcmp(&src, &(*iter), sizeof(src)) == 0) {
            found = true;
            break;
        }
    }

    if(found) {
        //TODO
        //1. create a new connection and destroy the old one?
        //2. reuse the old connection
    }
    else {
        //add the client if the server has space
        if(client_list.size() < max_conn) {
            //the constructor will send the conection response
            client_list.emplace_back (fd, src, hdr, ka_dur, curr_time);
        }
        else {
            //send a server full response
            #ifdef UNO_TRANSPORT_DEBUG
                fprintf (stderr, UNO_DBG_PREFIX "sending conn refused msg" PRIu8 UNO_DBG_SUFFIX);
            #endif
            _uno_send_server_full_rsp (fd, src, hdr);
        }
    }
}

/**
 * @brief returns the port number of the given fd, in host byte order
 * 
 * @param fd - the fd whose port needs to be determined
 * @return int - the port number if successful, UNO_TRANSPORT_ERR if failed
 */
static int _uno_get_port_from_fd (int fd)
{
    struct sockaddr_in addr = {0,};
    socklen_t len = sizeof(addr);
    int ret = UNO_TRANSPORT_ERR;

    if(getsockname(fd, (struct sockaddr*)&addr, &len) == UNO_TRANSPORT_ERR) {
        perror("getsockname");
    }
    else {
        ret = (int)ntohs(addr.sin_port);
    }
    return ret;
}

/**
 * @brief processes incoming messages addressed to the server (not connections)
 * 
 * @param fd - the fd of the server
 * @param hdr - rerference to the struct containing the header
 * @param buffer - the buffer containing the message
 * @param bytes - the length of the message (as returned by _uno_recv())
 * @param client_list - reference to the client list of this server
 * @param ka_dur - the keepalive duration of this server
 * @param src - const reference to the struct containing the source address
 * @param max_conn - max connections supported by this server
 * @param is_spl - reference to the variable that will be set if the incoming msg is a spl message
 * @return int - returns 
                    -> a positive number is a data message and is intended for the caller (spl message)
                    -> 0 if it is a control message intended for the server, and NOT the caller
                    -> UNO_TRANSPORT_ERR if failed
 */
static int _uno_process_server_msg (int fd, uno_hdr &hdr, uint8_t *buffer, ssize_t bytes, 
                        std::list<UnoConnection> &client_list, uint16_t ka_dur, const struct sockaddr_in &src, 
                        uint8_t max_conn, const struct timespec &curr_time, bool &is_spl)
{
    int ret = 0;
    is_spl = false;

    switch(_uno_get_msg_type (hdr.cmd)) {
    case _uno_msg_type :: UNO_MSG_CTRL: {
        #ifdef UNO_TRANSPORT_DEBUG
            fprintf (stderr, UNO_DBG_PREFIX "received control message. cmd_id = %" PRIu8 UNO_DBG_SUFFIX, buffer[0]);
        #endif
        //check command id
        switch(buffer[0]) {
        //incoming connection request, setup the new socket and sent connection rsp
        case UNO_TRANSPORT_CTRL_CONN_REQ: {
            //if
            switch(buffer[1]) {
            //process connection open request
            case UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_OPEN:
                _create_new_connection (fd, client_list, src, hdr, ka_dur, max_conn, curr_time);
                ret = 0;
                break;
            //close connection request must not be coming to the server fd
            case UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_CLOSE_CLIENT_REQ:
            case UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_CLOSE_SERVER_EXPL:
            case UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_CLOSE_SERVER_TO:
            default:
                ret = UNO_TRANSPORT_ERR;
                break;
            
            }
            break;
        }
        //should not be receiving conn response here, drop this packet
        case UNO_TRANSPORT_CTRL_CONN_RSP:
        //unknown control command, drop this packet
        default:
            ret = UNO_TRANSPORT_ERR;
            break;
        }
    }
    case _uno_msg_type :: UNO_MSG_ACK:
        break;
    case _uno_msg_type :: UNO_MSG_DATA:
        break;
    //spl msg is a message from an unconnected client that is intended for the caller
    case _uno_msg_type :: UNO_MSG_SPL:
        is_spl = true;
        ret = hdr.msg_len;
        break;
    case _uno_msg_type :: UNO_MSG_UNKNOWN:
        break;
    }
    //clear the buffer since it is the caller's buffer, not our own
    //bury the evidence
    if(ret <= 0) {
        memset (buffer, 0, bytes);
    }
    return ret;
}

/**
 * @brief processes incoming messages addressed to the connection (not the server)
 * 
 * @param fd - the fd of the connection
 * @param hdr - reference to the struct containing the header
 * @param buffer - the buffer containing the message
 * @param bytes - the length of the message
 * @param client_list - reference to the list of clients of this server
 * @param src - const reference to the struct containing the source address
 * @return int - returns
                    -> a positive number is a data message and is intended for the caller
                    -> 0 if it is a control message intended for the server/connection, and NOT the caller
                    -> UNO_TRANSPORT_ERR if failed
 */
static int _uno_process_client_msg (int fd, uno_hdr &hdr, uint8_t *buffer, ssize_t bytes, std::list<UnoConnection> &client_list,
                    const struct sockaddr_in &src)
{
    int ret = 0;

    switch(_uno_get_msg_type (hdr.cmd)) {
    case _uno_msg_type :: UNO_MSG_CTRL: {
        #ifdef UNO_TRANSPORT_DEBUG
                fprintf (stderr, UNO_DBG_PREFIX "received control message. cmd_id = %" PRIu8 UNO_DBG_SUFFIX, buffer[0]);
        #endif
        //check command id
        switch(buffer[0]) {
        //incoming connection request, setup the new socket and sent connection rsp
        case UNO_TRANSPORT_CTRL_CONN_REQ: {
            //check the options
            switch(buffer[1]) {
            //drop connection request since this is a connection fd, not the server fd
            case UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_OPEN:
                ret = UNO_TRANSPORT_ERR;
                break;
            //process the close connection request from the client
            case UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_CLOSE_CLIENT_REQ:
                #ifdef UNO_TRANSPORT_DEBUG
                        fprintf (stderr, UNO_DBG_PREFIX "received close connection for fd=%d" UNO_DBG_SUFFIX, fd);
                #endif
                _uno_close_connection_requested (fd, client_list);
                ret = 0;
                break;
            //drop the below connection requests since these modes are reserved from server
            //to client
            case UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_CLOSE_SERVER_TO:
            case UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_CLOSE_SERVER_EXPL:
            default:
                ret = UNO_TRANSPORT_ERR;
                break;
            
            }
            break;
        }        
        //keepalive message, don't do anything, just update the last msg time
        case UNO_TRANSPORT_CTRL_KEEPALIVE:
            ret = 0;
            break;
        //should not be receiving conn response here, drop this packet
        case UNO_TRANSPORT_CTRL_CONN_RSP:
            ret = UNO_TRANSPORT_ERR;
            break;
        //unknown control command, drop this packet
        default:
            ret = UNO_TRANSPORT_ERR;
            break;
        }
    }
    case _uno_msg_type :: UNO_MSG_ACK:
        break;
    //we need to pass the data message to the caller
    case _uno_msg_type :: UNO_MSG_DATA:
        ret = (int) hdr.msg_len;
        break;
    //spl messages are connectionless messages that are only supported in the server fd
    case _uno_msg_type :: UNO_MSG_SPL:
        ret = UNO_TRANSPORT_ERR;
        break;
    case _uno_msg_type :: UNO_MSG_UNKNOWN:
        break;
    }
    //clear the buffer since it is the caller's buffer, not our own
    if(ret <= 0) {
        memset (buffer, 0, bytes);
    }
    return ret;
}

/**
 * @brief Send a connection response message to the client saying that the server is full
 * 
 * @param fd - fd through which the message is to be sent
 * @param src - const reference to the sockaddr_in struct containing the source address
 * @param hdr - reference to the header structure
 * @return int - returns number of bytes sent if successful, UNO_TRANSPORT_ERR if not successful
 */
static int _uno_send_server_full_rsp (int fd, const struct sockaddr_in &src, uno_hdr &hdr)
{
    hdr.msg_len = 1;
    uint8_t hdr_ser[UNO_TRANSPORT_HDR_LEN];
    hdr.serialise(hdr_ser);
    uint8_t data = UNO_TRANSPORT_CTRL_CONN_RSP;

    return _uno_send (fd, hdr_ser, &data, 1, &src, NULL);
}

/**
 * @brief Send a connection close message from the client. Called in destructor
 * 
 * @param fd - The fd through which the message is to be sent
 * @param dst - const reference to the struct containing the destination address
 * @param seq - optional pointer to the uint16_t variable tracking the sequence number
 * @return int 
 */
static int _uno_send_conn_close (int fd, const struct sockaddr_in &dst, uint16_t *seq)
{
    uint16_t seq_id = (seq==nullptr) ? (0) : (*seq);
    uint8_t hdr_ser[UNO_TRANSPORT_HDR_LEN] = {0,};
    uint8_t cmd_flags = 0;

    //this is the data section for the connection request command
    uint8_t data[UNO_TRANSPORT_CTRL_CONN_REQ_LEN];
    data[0] = UNO_TRANSPORT_CTRL_CONN_REQ;  //command id
    data[1] = UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_CLOSE_CLIENT_REQ;    //command mode
    
    //set the control msg bit in the cmd field of the hdr and serialise the hdr
    cmd_flags = UNO_SET_BIT(cmd_flags, UNO_TRANSPORT_CMD_CONTROL_MSG);
    cmd_flags = UNO_SET_BIT(cmd_flags, UNO_TRANSPORT_CMD_RELIABLE_MSG);
    uno_hdr hdr (seq_id, cmd_flags, sizeof(data));

    //send this msg out
    hdr.serialise (hdr_ser);
    int ret = _uno_send_reliable (fd, hdr_ser, data, sizeof(data), dst, seq, 
                    UNO_TRANSPORT_REL_MAX_RETRIES, UNO_TRANSPORT_REL_RETRY_INT_MS);
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf (stderr, UNO_DBG_PREFIX "send conn close cmd fd=%d, ret=%d" UNO_DBG_SUFFIX, fd, ret);
    #endif

    return ret;
}

/**
 * @brief closes the connection of the given fd when a close request is received
 * 
 * @param fd - the fd of the connection to be closexd
 * @param client_list - reference to the client list
 */
static void _uno_close_connection_requested (int fd, std::list<UnoConnection> &client_list)
{
    for(auto iter=client_list.begin(); iter!=client_list.end(); ++iter) {
        if(fd == iter->fd) {
            //found the client in the list
            iter->set_close_reason (_close_reason :: CLOSE_REASON_REQUESTED_BY_CLIENT);
            _uno_close_connection_requested_iter (iter, client_list);
            return;
        }
    }
}

/**
 * @brief closes the connection at the given iterator when a close request is received
 * 
 * @param iter iterator of the node in the client list
 * @param client_list reference to the client list
 */
static void _uno_close_connection_requested_iter (std::list<UnoConnection>::iterator iter, std::list<UnoConnection> &client_list)
{
#ifdef  UNO_TRANSPORT_DEBUG
    fprintf(stderr, UNO_DBG_PREFIX "closing conn of fd=%d" UNO_DBG_SUFFIX, iter->fd);
#endif
    // iter->connected = false;
    client_list.erase (iter);
}

/**
 * @brief searches the list by IPv4 addr and returns the iterator of the node where the required client info is stored
 * 
 * @param client_list reference to the client list 
 * @param dst reference to the struct containing the IPv4 address
 * @return std::list<UnoConnection>::iterator - position of the client if found, client_list.end() if not found
 */
static std::list<UnoConnection>::iterator _find_client_in_list (std::list<UnoConnection> &client_list, 
                const struct sockaddr_in &dst)
{
    auto ret = client_list.end();
    for(auto iter=client_list.begin(); iter!=client_list.end(); ++iter) {
        if(memcmp(&dst, &(iter->cli_addr), sizeof(dst)) == 0) {
            ret = iter;
            break;
        }
    }
    return ret;
}

/**
 * @brief searches the list by fd and returns the iterator of the node where the required client's info is stored
 * 
 * @param client_list - reference to the client list
 * @param fd - fd of the client to be found
 * @return std::list<UnoConnection>::iterator - position of the client if found, client_list.end() if not found
 */
static std::list<UnoConnection>::iterator _find_client_in_list (std::list<UnoConnection> &client_list, int fd)
{
    auto ret = client_list.end();
    for(auto iter=client_list.begin(); iter!=client_list.end(); ++iter) {
        if(iter->fd == fd) {
            ret = iter;
            break;
        }
    }
    return ret;
}

/**
 * @brief processes the received message on the client side
 * 
 * @param hdr - reference to the struct containing the header
 * @param buffer - pointer to the buffer containing the message
 * @param bytes - number of bytes of the total message (including header, arithmetic will be handled here)
 * @param server_seq - reference to the variable tracking the sequence numbers used by the server
 * @return int - returns
                    -> positive number (number of bytes) if the message is a data message and should be passed to 
                            the caller
                    -> 0 if the message is a control message and is not relevant to the caller
                    -> UNO_TRANSPORT_CONN_CLOSE_RECVD if the server has closed the connection
                    -> UNO_TRANSPORT_ERR for any other failure case
                    
 */
static int _uno_client_process_recv_msg (uno_hdr &hdr, uint8_t *buffer, ssize_t bytes, uint16_t &server_seq)
{
    int ret = 0;

    switch(_uno_get_msg_type (hdr.cmd)) {
    case _uno_msg_type :: UNO_MSG_CTRL: {
#ifdef  UNO_TRANSPORT_DEBUG
        fprintf (stderr, UNO_DBG_PREFIX "received control message. cmd_id = %" PRIu8 UNO_DBG_SUFFIX, buffer[0]);
#endif
        //check command id
        switch(buffer[0]) {
        //incoming connection request, setup the new socket and sent connection rsp
        case UNO_TRANSPORT_CTRL_CONN_REQ: {
            //if
            switch(buffer[1]) {
            //drop connection request since this is a client, not a server (we don't take connections here)
            case UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_CLOSE_CLIENT_REQ:
            case UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_OPEN:
                ret = UNO_TRANSPORT_ERR;
                break;
            //process the close connection request - the server wants to close the connection
            case UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_CLOSE_SERVER_TO:
            case UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_CLOSE_SERVER_EXPL:
                ret = UNO_TRANSPORT_CONN_CLOSE_RECVD;
                break;
            default:
                ret = UNO_TRANSPORT_ERR;
                break;            
            }
            break;
        }
        //should not be receiving conn response here, drop this packet
        case UNO_TRANSPORT_CTRL_CONN_RSP:
        //unknown control command, drop this packet
        default:
            ret = UNO_TRANSPORT_ERR;
            break;
        }
    }
    case _uno_msg_type :: UNO_MSG_ACK:
        //ACK packet, we coo'
        break;

    //we need to pass the data message to the caller
    case _uno_msg_type :: UNO_MSG_DATA:
        ret = (int) hdr.msg_len;
        break;
    //spl messages are connectionless messages that are only supported in the server fd
    case _uno_msg_type :: UNO_MSG_SPL:
        ret = UNO_TRANSPORT_ERR;
        break;
    case _uno_msg_type :: UNO_MSG_UNKNOWN:
        ret = UNO_TRANSPORT_ERR;
        break;
    }
    //clear the buffer since it is the caller's buffer, not our own
    if(ret <= 0) {
        memset (buffer, 0, bytes);
    }
    return ret;
}

/**
 * @brief function that performs reliable send according to this protocol
 * 
 * @param fd - the fd through which the message is to be sent and the ACK is to be received
 * @param hdr_ser - pointer to the buffer containing the fixed length header
 * @param msg - pointer to the buffer containing the message
 * @param len - length of the message
 * @param dst - reference to the struct containing the IPv4 address of the destination
 * @param seq - optional pointer to the variable that is being used to track the sequence number of the sender
 * @param max_retries - the max retries if ACK is not received
 * @param retry_interval - the time to wait before every retry
 * @return int - returns 
                    -> positive number of bytes on successful reliable send
                    -> 0 if the message was sent successfully, but no ACK was received
                    -> UNO_TRANSPORT_CONN_CLOSE_RECVD if conn close msg is delivered
                    -> UNO_TRANSPORT_ERR on general failure
 */
static int _uno_send_reliable (int fd, uint8_t *hdr_ser, const void *msg, size_t len, 
                                const struct sockaddr_in &dst, uint16_t *seq, uint8_t max_retries, uint16_t retry_interval)
{
    //make sure that the reliable msg bit is set
    assert (_uno_is_reliable_msg(hdr_ser[UNO_TRANSPORT_HDR_CMD_POS])==true);
    int ret = 0;
    uint8_t hdr[UNO_TRANSPORT_HDR_LEN];
    struct sockaddr_in src;
    int temp;
    uint8_t data[2];

    //loop and perform all attempts
    for(int i=0; i<max_retries; i++) {
        memset (hdr, 0, sizeof(hdr));
        memset (&src, 0, sizeof(src));
        //send out the message
       temp = _uno_send (fd, hdr_ser, msg, len, &dst, NULL);
       if(temp == UNO_TRANSPORT_ERR) {
           ret = temp;
           break;
       }

       //recv the message
       temp = _uno_timed_recv (fd, hdr, data, sizeof(data), MSG_WAITALL, &src, retry_interval);
       if(temp == UNO_TRANSPORT_ERR) {
           break;
       }

       //if we received a response, check the response
       if(temp > 0) {
           //check if it is an ack response
           if(_uno_get_msg_type(hdr[UNO_TRANSPORT_HDR_CMD_POS]) == _uno_msg_type::UNO_MSG_ACK) {
               //TODO - validate seq id and msg len
               //successful ack, set ret as temp
               ret = temp;
               #ifdef UNO_TRANSPORT_DEBUG
                 fprintf (stderr, UNO_DBG_PREFIX "received ack msg for seq=%" PRIu16 UNO_DBG_SUFFIX,(*seq)-1);                 
               #endif
               break;
           }
           //else check if this is a connection close msg
           if(_uno_get_msg_type(hdr[UNO_TRANSPORT_HDR_CMD_POS]) == _uno_msg_type::UNO_MSG_CTRL) {
               if(data[0] == UNO_TRANSPORT_CTRL_CONN_CLOSED) {
                   ret = UNO_TRANSPORT_CONN_CLOSE_RECVD;
                   break;
               }
           }
       }

       //else increment retry count and try again
       hdr_ser[UNO_TRANSPORT_HDR_RC_POS]++;
    }   //end of for loop

    //update the sequence number if the send was successful (regardless of ACK)
    if(ret != UNO_TRANSPORT_ERR && (seq!=nullptr)) {
        (*seq)++;
    }
    return ret;
}

/**
 * @brief Checks if the given cmd_flags indicates that this message is a reliable msg
 * 
 * @param cmd_flags The 8 bit value containing the cmd_flags
 * @return - self explanatory
 */
static bool _uno_is_reliable_msg (uint8_t cmd_flags)
{
    if(UNO_IS_BIT_SET(cmd_flags,UNO_TRANSPORT_CMD_RELIABLE_MSG)) return true;
    else return false;
}

/**
 * @brief Sends the ACK to the specified node for the given header
 * 
 * @param fd - the fd through which the data is to be sent
 * @param dst - const reference to the struct containing the destination IPv4 address
 * @param hdr - reference to the struct containing the header
 * @return int - number of bytes sent on success, UNO_TRANSPORT_ERR on failure
 */
static int _uno_send_ack (int fd, const struct sockaddr_in &dst, const uno_hdr &hdr)
{
    //ack is the same header, but with msg len as 0 and ack bit as true
    //and with no data
    uno_hdr ack = hdr;
    ack.msg_len = 0;

    //make sure that the ACK bit is set
    uint8_t hdr_ser[UNO_TRANSPORT_HDR_LEN];
    ack.cmd = UNO_SET_BIT(ack.cmd, UNO_TRANSPORT_CMD_ACK_MSG);
    ack.serialise (hdr_ser);

    //send out the message
    int ret = _uno_send (fd, hdr_ser, NULL, 0, &dst, NULL);
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf (stderr, UNO_DBG_PREFIX "received reliable msg at fd=%d of len=%" PRIu16 ", sending ack ret= %d" UNO_DBG_SUFFIX, 
                                    fd, hdr.msg_len, ret );
    #endif
    return ret;
}

/**
 * @brief Checks if the sequence number is valid
 * 
 * @param fd - the fd that is involved in this transaction (only for logging purpose)
 * @param inc_seq - the incoming sequence number
 * @param saved_seq - reference to the last saved sequence number (will be updated if this message is valid)
 * @return - self explanatory 
 */
static bool _is_seq_valid (int fd, uint16_t inc_seq, uint16_t &saved_seq)
{
    //if inc_seq is UNO_SEQ_ID_RESET (0), that means that the device is new or the sequence number has overflown
    if(inc_seq == UNO_SEQ_ID_RESET) { 
        #ifdef UNO_TRANSPORT_DEBUG
            fprintf (stderr, UNO_DBG_PREFIX "WARN::seq id has been overflown/reset for fd=%d" UNO_DBG_SUFFIX, 
                                    fd);
        #endif
        saved_seq = inc_seq;
        return true; 
    }
    //incoming must always be greater than saved
    else if(inc_seq > saved_seq) {
        #ifdef UNO_TRANSPORT_DEBUG
            if((inc_seq-saved_seq) > 1) {
                fprintf (stderr, UNO_DBG_PREFIX "WARN::possible lost packetsinc_seq-saved_seq=%" PRIu16 " for fd=%d"
                                UNO_DBG_SUFFIX, inc_seq-saved_seq, fd);
            }
        #endif
        saved_seq = inc_seq;
        return true;
    }
    else {
        //invalid sequence
        #ifdef UNO_TRANSPORT_DEBUG
            if(inc_seq == saved_seq) {
                fprintf (stderr, UNO_DBG_PREFIX "dropping duplicate packet with seq=%" PRIu16 " for fd=%d" UNO_DBG_SUFFIX, 
                                    inc_seq, fd);
            }
            else {
                fprintf (stderr, UNO_DBG_PREFIX "dropping out of order packet with seq=%" PRIu16 " for fd=%d" 
                                    " expected=%" PRIu16 UNO_DBG_SUFFIX, 
                                    inc_seq, fd, saved_seq+1);
            }
        #endif
        return false;
    }
}

/**
 * @brief - Sends an explicit connection closure message from the server (caller asked for connection closure)
 * 
 * @param fd - the fd through which the message is to be sent
 * @param dst - const reference to the struct containing the destination address
 * @param seq - optional pointer to the variable where the sequence id is being tracked, will be incremented if provided
 * @return int - returns number of bytes sent if successful, else UNO_TRANSPORT_ERR if failure
 */
static int _uno_send_conn_close_expl (int fd, const struct sockaddr_in &dst, uint16_t *seq)
{
    uint16_t seq_id = (seq==nullptr) ? (0) : (*seq);
    uint8_t hdr_ser[UNO_TRANSPORT_HDR_LEN] = {0,};
    uint8_t cmd_flags = 0;

    //this is the data section for the connection request command
    uint8_t data[UNO_TRANSPORT_CTRL_CONN_REQ_LEN];
    data[0] = UNO_TRANSPORT_CTRL_CONN_REQ;  //command id
    data[1] = UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_CLOSE_SERVER_EXPL;    //command mode
    
    //set the control msg bit in the cmd field of the hdr and serialise the hdr
    cmd_flags = UNO_SET_BIT(cmd_flags, UNO_TRANSPORT_CMD_CONTROL_MSG);
    uno_hdr hdr (seq_id, cmd_flags, sizeof(data));

    //send this msg out
    hdr.serialise (hdr_ser);
    int ret = _uno_send (fd, hdr_ser, data, sizeof(data), &dst, seq);
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf(stderr, UNO_DBG_PREFIX "sent explicit connection close req %d bytes to fd=%d port %hu" UNO_DBG_SUFFIX, ret, 
                            fd, ntohs(dst.sin_port));
    #endif
    return ret;
}

/**
 * @brief - Sends an explicit connection timedout message from the server (no data in the specified keepalive duration)
 * 
 * @param fd - the fd through which the message is to be sent
 * @param dst - const reference to the struct containing the destination address
 * @param seq - optional pointer to the variable where the sequence id is being tracked, will be incremented if provided
 * @return int - returns number of bytes sent if successful, else UNO_TRANSPORT_ERR if failure
 */
static int _uno_send_conn_close_to (int fd, const struct sockaddr_in &dst, uint16_t *seq)
{
    uint16_t seq_id = (seq==nullptr) ? (0) : (*seq);
    uint8_t hdr_ser[UNO_TRANSPORT_HDR_LEN] = {0,};
    uint8_t cmd_flags = 0;

    //this is the data section for the connection request command
    uint8_t data[UNO_TRANSPORT_CTRL_CONN_REQ_LEN];
    data[0] = UNO_TRANSPORT_CTRL_CONN_REQ;  //command id
    data[1] = UNO_TRANSPORT_CTRL_CONN_REQ_OPT_MODE_CLOSE_SERVER_TO;    //command mode
    
    //set the control msg bit in the cmd field of the hdr and serialise the hdr
    cmd_flags = UNO_SET_BIT(cmd_flags, UNO_TRANSPORT_CMD_CONTROL_MSG);
    uno_hdr hdr (seq_id, cmd_flags, sizeof(data));

    //send this msg out
    hdr.serialise (hdr_ser);
    int ret = _uno_send (fd, hdr_ser, data, sizeof(data), &dst, seq);
    #ifdef UNO_TRANSPORT_DEBUG
        fprintf(stderr, UNO_DBG_PREFIX "sent timedout connection close req %d bytes to fd=%d port %hu" UNO_DBG_SUFFIX, ret, 
                            fd, ntohs(dst.sin_port));
    #endif
    return ret;
}

/**
 * @brief - linearly searches through the list and finds the connection that is set to timeout the earliest
 * 
 * @param client_list - reference to the client list
 * @return std::list<UnoConnection>::iterator - returns an iterator at the position in the client list
 */
static std::list<UnoConnection>::iterator _uno_find_most_stale_connection (std::list<UnoConnection> &client_list)
{
    if(client_list.empty()) {
        return client_list.end();
    }
    //initialise the oldest time
    struct timeval oldest;
    auto ret = client_list.begin();
    memcpy (&oldest, &(ret->last_msg_time), sizeof(oldest));

    //skip the first item since it is already stored in oldest
    auto iter=client_list.begin();
    ++iter;
    //loop through the list
    for(; iter!=client_list.end(); ++iter) {
        //compare seconds field
        if(iter->last_msg_time.tv_sec < oldest.tv_sec) {
            //update oldest and ret
            memcpy (&oldest, &(iter->last_msg_time), sizeof(oldest));
            ret = iter;
        }
        //if seconds are equal, compare nanoseconds field
        else if(iter->last_msg_time.tv_nsec < iter->last_msg_time.tv_nsec) {
            //update oldest and ret
            memcpy (&oldest, &(iter->last_msg_time), sizeof(oldest));
            ret = iter;
        }
        //else ignore
        else {
            ;   //left here for readability, the compiler will optimise this out
        }
    }

    return ret;
}

/**
 * @brief - computes the difference between 2 timespec structs. performs (end-start)
 * 
 * @param start - const reference to the timespec containing the start time
 * @param end - const reference to the timespec containing the end time.
 * @param result - reference to the struct where the result of the difference will be stored
 */
static void __uno_timespec_diff (const struct timespec &start, const struct timespec &end, struct timespec &result)
{
    assert (end.tv_sec >= start.tv_sec);
    //assumes end is greater than start, does not check, it is the caller's responsibility to ensure this
    if(end.tv_nsec < start.tv_nsec) {
        assert (end.tv_sec > start.tv_sec);
        result.tv_sec = end.tv_sec - start.tv_sec - 1;
        result.tv_nsec = end.tv_nsec - start.tv_nsec + UNO_SEC_TO_NS(1);
    }
    else {
        result.tv_sec = end.tv_sec - start.tv_sec;
        result.tv_nsec = end.tv_nsec - start.tv_nsec;
    }
}

/**
 * @brief - computes the sum between 2 timespec structs
 * 
 * @param start - const reference to the timespec containing the start time
 * @param end - const reference to the timespec containing the end time
 * @param result - reference to the struct where the result will be stored
 */
static void __uno_timespec_sum (const struct timespec &start, const struct timespec &end, struct timespec &result)
{
    result.tv_sec = start.tv_sec + end.tv_sec;
    result.tv_nsec = start.tv_nsec + end.tv_nsec;

    //now check for "overflow", if result.tv_nsec is greater than 10^9
    if(result.tv_nsec > UNO_SEC_TO_NS(1)) {
        result.tv_sec++;
        result.tv_nsec = result.tv_nsec % UNO_SEC_TO_NS(1);
    }

}

/**
 * @brief - computes the value to be passed to timerfd_settime given the last message time, the keepalive duration (in ms),
            the current time
 * 
 * @param last_msg_time - const reference to the struct containing the last message time
 * @param keepalive_dur_ms - the expected keepalive duration (in milliseconds)
 * @param curr_time - const reference to the timespec containing the curr time
 * @param timer_val - the timespec where the computed timer value is to be written
 */
static void _uno_compute_timer_value (const struct timespec &last_msg_time, uint16_t keepalive_dur_ms, 
                                        const struct timespec &curr_time, struct timespec &timer_val)
{
    //transform keepalive_dur_ms to struct timespec
    struct timespec _keep_alive = {0,};
    _keep_alive.tv_sec = UNO_MS_TO_SEC(keepalive_dur_ms);
    if(keepalive_dur_ms % 1000) {
        _keep_alive.tv_nsec = UNO_MS_TO_NS(keepalive_dur_ms % 1000);
    }

    //compute expiry time
    struct timespec _expiry_time = {0,};
    __uno_timespec_sum (last_msg_time, _keep_alive, _expiry_time);

    //now the timervalue is the difference between _expiry_time and curr_time
    __uno_timespec_diff (curr_time, _expiry_time, timer_val);
}