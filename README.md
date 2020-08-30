# UnoTransport

UnoTransport is a custom "connection" oriented protocol that provides in order delivery and acknowledgement based messaging. It is a custom protocol that is designed over UDP. This entire project is written in C++. The features supported are:

* Connection oriented streams. There is a connect() API in the client object
* Reliable message send. The sender expects protocol level acknowledgements for every packet
* Supports multiple connections on a single thread (uses [poll()](https://linux.die.net/man/2/poll) for multiplexing several FD's in a single thread)
* Supports timing out specific connections if data is not sent.
* Supports keepalive message if there is no data to be sent

The reason behind this name is I was writing a LAN multiplayer game of Uno and I got sidetracked into making this protocol. And now I'm too lazy to refactor the code and rename everything.

## DISCLAIMER

I made this protocol as a learning exercise on how to design and implement a protocol. This protocol/implementation has several flaws and if you need a connection oriented protocol, you're better off just using TCP.
I have used the linux specific timerfd feature. So this code is probably not very portable.

## How It works
Every message is prefixed with a fixed length (7 byte) header containing the following fields:
* Sequence ID of this message
* Retry count of this message
* Version of this protocol
* Command flags for this message
* Length of this message

This can be seen documented at the top of the UnoTransport.cpp file.

Sequence ID: This field contains a 2 byte sequence ID for every message. For connection oriented messages, this field is used to detect duplicate or out of order messages. As of now, they will be dropped

Retry count: This field contains a 1 byte retry count for messages. This field is used for debugging purposes

Version: The supported version of the protocol is checked on both server and client sides. As of now, messages containing mismatched version will be dropped

Command flags: This is a 1 byte field in the header, where different bits can be set to indicate the type of message this is. There are 4 types of messages as of now:
* Control message (Could be connection req/rsp, keepalive message, etc)
* Data message (Message that is intended for the caller of the recv_msg() function)
* ACK message (for any message that is marked as reliable, an ACK must be sent)
* Special message (for lack of a better name, this is a connectionless message sent to the caller of recv_msg()) (not implemented as of now)

### Connection process
When a client wants to connect to a server, it sends a connect request. When the server receives a connect request, it creates a new socket with a random port. This port information and the expected keepalive duration are sent to the client as part of the connection response.

Now, the client can talk to the server over this new "connection"

## Usage

Use the given makefile to build an example code. 

```
make
```

Build a debug version to see debug logs

```
make debug
```

## Credits
This readme file was created using https://www.makeareadme.com/
