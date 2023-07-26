// File:        Socket.h
// Date:        6/27/07
// Author:      Tim Harris
// Description: This file contains declarations for several classes used to
//              wrap network sockets with UDP and TCP protocols. The SocketHeader
//              class is used when enabling headers for the UDP- and TCPSockets.
//              The headers are helpful when sending large files that may be split
//              up and when reading a TCP socket that may contain multiple packets.
//              The UDPSocket class wraps a connectionless socket and can not
//              guarantee packets will be received in the same order sent or to
//              even that they will reach the destination.
//              The TCPSocket class represents a connection between two sockets.
//              The class will allow only two clients to communicate with each other
//              with a guaranteed delivery of packets.
//

#ifndef _AVTAS_SUPPORT_SOCKET_H_
#define _AVTAS_SUPPORT_SOCKET_H_

#if !(defined(WIN32) || defined(LINUX))
#error Must define either WIN32 or LINUX.
#endif

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <string>

#ifdef WIN32
#include <winsock.h>
#include <time.h>
#include <io.h>
#define WSOCK32_VER 0x0101
#else
#include <sys/socket.h>
#include <sys/time.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <fcntl.h>
#include <netinet/ip.h>
#include <netinet/udp.h>
#include <netinet/tcp.h>
#endif

#include "SocketBuffer.h"

namespace avtas
{
namespace support
{

#define DEFAULT_PORT        8000
#define DEFAULT_RCVBUF_SIZE 2048
#define MAX_INBOUND_CALLS     10
#define REXFLAG             0xbeefface

/******************** Class declarations ********************/
class SocketHeader;
class Socket;
class UDPSocket;
class TCPSocket;
/****************** End class declarations ******************/

/* SocketHeader - A container class used to facilitate   */
/* large packet transfers                                   */
class SocketHeader
{
public:
    ulong                       rexFlag;
    ulong                       msgSize;

    SocketHeader( void );
    SocketHeader( const SocketHeader& that );
    SocketHeader& operator=( const SocketHeader& that );
    ~SocketHeader( void );

    void copyTo( void* buf );
    void copyFrom( void* buf );
    inline ulong hdrSize( void )
    {
        return ( 2 * sizeof( ulong ) );
    }
};

/* Socket - The parent class (abstract) of the different */
/* socket types (datagram, stream, etc)                     */
class Socket
{
protected:
    int                         sockFD;
    double                      timeout;
    bool                        printErrs; /* Controls debug output */
    bool                        useTimeout;
    bool                        useHeaders; /* Allows transfer of large packets */
    SocketBuffer                inBuf;
    SocketBuffer                outBuf;
    ulong                       recvMsgCount;
    ulong                       sentMsgCount;
    ulong                       recvByteCount;
    ulong                       sentByteCount;

    struct sockaddr_in          peerAddress;
    ushort                      localPort;
    bool                        firstInit;
    bool                        connected;

    int                         initSocket( int socketType );
    int                         setNonBlocking( void );
    int                         setReuseAddr( void );
    inline void                 incRecvMsgCount( ulong add )
    {
        recvMsgCount += add;
    }
    inline void                 incSentMsgCount( ulong add )
    {
        sentMsgCount += add;
    }
    inline void                 incRecvByteCount( ulong add )
    {
        recvByteCount += add;
    }
    inline void                 incSentByteCount( ulong add )
    {
        sentByteCount += add;
    }
    std::string                 ipAddrToString( unsigned int ip );
#ifdef LINUX
    unsigned int                getLocalIPAddress( std::string interface="eth0" );
#endif

public:
    Socket( void );
    Socket( const Socket& that );
    Socket&                     operator=( const Socket& that );
    virtual                     ~Socket( void );
    virtual int                 setPeerAddr( const char* addr, ushort port ) = 0;
    virtual int                 getPeerAddr( char* addr, ulong addrSize, ushort* port ) = 0;
    inline ulong                getPeerAddr();
    std::string                 getPeerName( void );
    ushort                      getPeerPort( void );
    virtual int                 initLocalPort( ushort port, Socket* newsock = 0 ) = 0;
    virtual int                 sendData( const void* buf, ulong size ) = 0;
    virtual int                 recvData( void** buf, ulong* size ) = 0;

    virtual int                 disconnect( void ) = 0;
    inline bool                 isConnected( void )
    {
        return connected;
    }
    inline ushort               getLocalPort( void )
    {
        return localPort;
    }

    inline void                 enableErrorOutput( void )
    {
        printErrs = true;
    }
    inline void                 disableErrorOutput( void )
    {
        printErrs = false;
    }
    inline void                 enableHeaders( void )
    {
        useHeaders = true;
    }
    inline void                 disableHeaders( void )
    {
        useHeaders = false;
    }
    inline void                 setTimeout( double to )
    {
        timeout = to;
    }
    inline double               getTimeout( void )
    {
        return timeout;
    }
    inline void                 enableTimeout( void )
    {
        useTimeout = true;
    }
    inline void                 disableTimeout( void )
    {
        useTimeout = false;
    }
    int                         getSocketError( void );
    inline ulong                getRecvMsgCount( void )
    {
        return recvMsgCount;
    }
    inline ulong                getSentMsgCount( void )
    {
        return sentMsgCount;
    }
    inline ulong                getRecvByteCount( void )
    {
        return recvByteCount;
    }
    inline ulong                getSentByteCount( void )
    {
        return sentByteCount;
    }
    inline void                 clearRecvMsgCount( void )
    {
        recvMsgCount = 0;
    }
    inline void                 clearSentMsgCount( void )
    {
        sentMsgCount = 0;
    }
    inline void                 clearRecvByteCount( void )
    {
        recvByteCount = 0;
    }
    inline void                 clearSentByteCount( void )
    {
        sentByteCount = 0;
    }
    inline void                 setRecvBufferSize( ulong size )
    {
        inBuf.setBufferSize( size );
    }
    inline ulong                getRecvBufferSize( void )
    {
        return inBuf.getBufferSize();
    }
    inline void 		      getPeerAddress( struct sockaddr_in& peer )
    {
        peer = peerAddress;
    }
    inline void 		      getPeerAddress( struct sockaddr& peer )
    {
        peer = *( struct sockaddr* )&peerAddress;
    }
    inline void               setPeerAddress( const struct sockaddr_in& i_sockaddr )
    {
        peerAddress = i_sockaddr;
    }
};

/* UDPSocket - A datagram socket class                   */
/* This socket is connectionless, so setPeerAddr can be     */
/* called as many times as necessary without affecting the  */
/* socket.                                                  */
class UDPSocket : public Socket
{
public:
    UDPSocket( void );
    UDPSocket( const char* addr, ushort port );
    UDPSocket( const UDPSocket& that );
    UDPSocket&                  operator=( const UDPSocket& that );
    virtual                     ~UDPSocket( void );
    virtual int                 setPeerAddr( const char* addr, ushort port );
    virtual int                 getPeerAddr( char* addr, ulong addrSize, ushort* port );
    virtual int                 initLocalPort( ushort port, Socket* newsock = 0 );
    virtual int                 sendData( const void* buf, ulong size );
    virtual int                 recvData( void** buf, ulong* size );
    virtual int                 disconnect( void );
    virtual int                 setBroadcast( void );
    virtual int                 joinMulticastGroup( std::string group );
    virtual int                 leaveMulticastGroup( std::string group );
    inline int                  setPeerAddr( const unsigned long addr, ushort port = 0 )
    {
        peerAddress.sin_addr.s_addr = addr;
        peerAddress.sin_port = port > 1024 ? htons( port ) : peerAddress.sin_port;

        return 0;
    }
};

/* TCPSocket - A stream socket class                     */
/* This socket is connection-based. Calling setPeerAddr     */
/* has the effect of closing the previous opened connection */
/* (if any) and reconnecting to the new destination.        */
class TCPSocket : public Socket
{
protected:
    int                         initAcceptedConnection( int newFD );

public:
    TCPSocket( void );
    TCPSocket( const char* addr, ushort port );
    TCPSocket( const TCPSocket& that );
    TCPSocket&                  operator=( const TCPSocket& that );
    virtual                     ~TCPSocket( void );
    virtual int                 setPeerAddr( const char* addr, ushort port );
    virtual int                 getPeerAddr( char* addr, ulong addrSize, ushort* port );
    virtual int                 initLocalPort( ushort port, Socket* newsock = 0 );
    virtual int                 sendData( const void* buf, ulong size );
    virtual int                 recvData( void** buf, ulong* size );
    virtual int                 disconnect( void );

    /* Note that the current implementation of recvData() will not
       receive multiple IP packet messages since recv() is only
       called once. */
};

} // end namespace support
} // end namespace avtas

#endif // defined _AVTAS_SUPPORT_SOCKET_H_


