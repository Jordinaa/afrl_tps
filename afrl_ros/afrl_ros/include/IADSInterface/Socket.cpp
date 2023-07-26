#include "Socket.h"

#ifdef WIN32
#define close(X) closesocket(X)
#endif
#include <iostream>
using namespace std;

namespace avtas
{
namespace support
{

/***************** SocketHeader *****************/
SocketHeader::SocketHeader( void )
{
    rexFlag = REXFLAG;
    msgSize = 0;
    return;
}

SocketHeader::SocketHeader( const SocketHeader& that ) :
    rexFlag( that.rexFlag ),
    msgSize( that.msgSize )
{
    return;
}

SocketHeader& SocketHeader::operator=( const SocketHeader& that )
{
    if ( this != &that )
    {
        rexFlag = that.rexFlag;
        msgSize = that.msgSize;
    }
    return *this;
}

SocketHeader::~SocketHeader( void )
{
    return;
}

void SocketHeader::copyTo( void* buf )
{
    ulong                       tmp;

    if ( buf == 0 )
    {
        return;
    }

    tmp = htonl( rexFlag );
    memcpy( buf, &tmp, sizeof( ulong ) );
    tmp = htonl( msgSize );
    memcpy( ( ( char* )buf ) + sizeof( ulong ), &tmp, sizeof( ulong ) );

    return;
}

void SocketHeader::copyFrom( void* buf )
{
    ulong                       tmp;

    if ( buf == 0 )
    {
        rexFlag = msgSize = 0;
        return;
    }

    memcpy( &tmp, buf, sizeof( ulong ) );
    rexFlag = ntohl( tmp );
    memcpy( &tmp, ( ( char* )buf ) + sizeof( ulong ), sizeof( ulong ) );
    msgSize = ntohl( tmp );

    return;
}
/*************** End SocketHeader ***************/

/******************** Socket ********************/
Socket::Socket( void ) :
    sockFD        ( -1 ),
    timeout       ( 0.0 ),
    printErrs     ( false ),
    useTimeout    ( false ),
    useHeaders    ( false ),
    recvMsgCount  ( 0 ),
    sentMsgCount  ( 0 ),
    recvByteCount ( 0 ),
    sentByteCount ( 0 ),
    firstInit     ( true )
{
    inBuf.allocate( DEFAULT_RCVBUF_SIZE );
    return;
}

Socket::Socket( const Socket& that ) :
    sockFD        ( 0 ),
    timeout       ( 0.0 ),
    printErrs     ( false ),
    useTimeout    ( false ),
    useHeaders    ( false ),
    inBuf         ( that.inBuf ),
    recvMsgCount  ( 0 ),
    sentMsgCount  ( 0 ),
    recvByteCount ( 0 ),
    sentByteCount ( 0 ),
    firstInit     ( true )
{
    return;
}

Socket& Socket::operator=( const Socket& that )
{
    if ( this != &that )
    {
        sockFD        = 0;
        timeout       = 0.0;
        printErrs     = false;
        useTimeout    = false;
        useHeaders    = false;
        recvMsgCount  = 0;
        sentMsgCount  = 0;
        recvByteCount = 0;
        sentByteCount = 0;
        firstInit     = true;
        inBuf         = that.inBuf;
    }
    return *this;
}

Socket::~Socket( void )
{
    if ( sockFD != -1 )
    {
        close( sockFD );
    }
    return;
}

int Socket::getSocketError( void )
{
    int                         status, sockErr;

#if defined(IRIX) || defined(WIN32)
    int                         optargsize = sizeof( int );
#elif LINUX
    socklen_t                   optargsize = sizeof( int );
#endif

    status = getsockopt( sockFD, SOL_SOCKET, SO_ERROR, ( char* )( &sockErr ), &optargsize );

    if ( ( status == -1 ) && ( printErrs ) )
    {
        perror( "ERROR (Socket::PrintSocketError) getsockopt()" );
    }

    return sockErr;
}

int Socket::initSocket( int socketType )
{
    int                         status = 0;

    /* Close if already opened */
    if ( sockFD != -1 )
    {
        status = close( sockFD );
        if ( status != 0 )
        {
            perror( "ERROR (Socket::initSocket) close()" );
        }
    }

    sockFD = socket( AF_INET, socketType, 0 );
    if ( sockFD == -1 )
    {
        /* An error occurred, see errno */
        status = -1;
        if ( printErrs )
        {
            perror( "ERROR (Socket::initSocket) socket()" );
        }
        return status;
    }

    /* Make it non-blocking */
    status = setNonBlocking();

    /* Set to reuse address */
    if ( status == 0 )
    {
        status = setReuseAddr();
    }

    /* Initialize socket data */
    connected = false;
    if ( firstInit )
    {
        peerAddress.sin_family      = AF_INET;
        peerAddress.sin_port        = htons( DEFAULT_PORT );
        peerAddress.sin_addr.s_addr = htonl( INADDR_ANY );
        memset( &( peerAddress.sin_zero ), '\0', 8 );
        localPort = 0;
        firstInit = false;
    }

    return status;
}

int Socket::setNonBlocking( void )
{
    int                         status = 0;

#ifdef WIN32
    ulong                       enable = 1;
    status = ioctlsocket( sockFD, FIONBIO, &enable );
    if ( status != 0 )
    {
        if ( printErrs )
        {
            perror( "ERROR (Socket::setNonBlocking) ioctlsocket()" );
        }
    }
#elif LINUX
    status = fcntl( sockFD, F_SETFL, O_NONBLOCK );
    if ( status != 0 )
    {
        if ( printErrs )
        {
            perror( "ERROR (Socket::setNonBlocking) fnctl()" );
        }
    }
#endif

    return status;
}

int Socket::setReuseAddr( void )
{
    int                         status = 0;

#ifdef WIN32
    const char                  yes = 1;
#elif LINUX
    int                         yes = 1;
#endif

    status = setsockopt( sockFD, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof( int ) );
    if ( status != 0 )
    {
        if ( printErrs )
        {
            perror( "ERROR (Socket::setReuseAddr) setsockopt()" );
        }
    }

    return status;
}

inline ulong Socket::getPeerAddr()
{
    return peerAddress.sin_addr.s_addr;
}

std::string Socket::getPeerName(void)
{
    char addr[INET_ADDRSTRLEN];
    inet_ntop( AF_INET, &peerAddress.sin_addr, addr, INET_ADDRSTRLEN );

    return std::string( peerAddress.sin_addr.s_addr?addr:"" );
}

ushort Socket::getPeerPort( void )
{
    return ntohs( peerAddress.sin_port );
}

#ifdef LINUX
#include <net/if.h>
#include <sys/ioctl.h>

unsigned int Socket::getLocalIPAddress( std::string interface )
{
    int fd = socket( AF_INET, SOCK_DGRAM, 0 );
    struct ifreq ifr;
    ifr.ifr_addr.sa_family = AF_INET;
    strncpy( ifr.ifr_name, interface.c_str(), IFNAMSIZ-1 );
    ioctl( fd, SIOCGIFADDR, &ifr );
    close( fd );

    return ( ( struct sockaddr_in* )&ifr.ifr_addr )->sin_addr.s_addr;
}
#endif

std::string Socket::ipAddrToString( unsigned int ip )
{
    char addr[16];
    unsigned char* ipPtr = ( unsigned char* )&ip;
    sprintf( addr, "%d.%d.%d.%d"
             , ( int )ipPtr[0]
             , ( int )ipPtr[1]
             , ( int )ipPtr[2]
             , ( int )ipPtr[3]
           );

    return addr;
}
/****************** End Socket ******************/

/****************** UDPSocket *******************/
UDPSocket::UDPSocket( void )
{
#ifdef WIN32
    int err;
    WSADATA wsa_data;
    err = WSAStartup( WSOCK32_VER, &wsa_data );

    if ( err != 0 )
    {
        fprintf( stderr, "ERROR (UDPSocket): WSAStartup() failed\n" );
    }
#endif

    initSocket( SOCK_DGRAM );
    return;
}

UDPSocket::UDPSocket( const char* addr, ushort port )
{
#ifdef WIN32
    int err;
    WSADATA wsa_data;
    err = WSAStartup( WSOCK32_VER, &wsa_data );

    if ( err != 0 )
    {
        fprintf( stderr, "ERROR (UDPSocket): WSAStartup() failed\n" );
    }
#endif

    initSocket( SOCK_DGRAM );
    if ( ( void* )addr )
    {
        if ( strlen( addr ) != 0 )
        {
            setPeerAddr( addr, port );
            return;
        }
    }

    initLocalPort( port );
    return;
}

UDPSocket::UDPSocket( const UDPSocket& that ) :
    Socket( that )
{
    return;
}

UDPSocket& UDPSocket::operator=( const UDPSocket& that )
{
    if ( this != &that )
    {
        if ( connected )
        {
            disconnect();
        }
        Socket::operator=( that );
    }
    return *this;
}

UDPSocket::~UDPSocket( void )
{
    if ( sockFD != -1 )
    {
        close( sockFD );
    }

#ifdef WIN32
    WSACleanup();
#endif

    return;
}

int UDPSocket::setPeerAddr( const char* addr, ushort port )
{
    struct hostent*             h      = 0;
    int                         status = 0;

    localPort = port;

    h = gethostbyname( addr );
    if ( h == 0 )
    {
        /* An error occurred, see h_errno */
        status = -1;
        if ( printErrs )
        {
#ifdef WIN32
            fprintf( stderr, "ERROR (UDPSocket::setPeerAddr) gethostbyname(): An error occurred.\n" );
#elif LINUX
            herror( "ERROR (UDPSocket::setPeerAddr) gethostbyname()" );
#endif
        }
        peerAddress.sin_addr.s_addr = htonl( INADDR_ANY );
    }
    else
    {
        /* Address is already in network byte order */
        memcpy( &( peerAddress.sin_addr.s_addr ), h->h_addr_list[0], sizeof( ulong ) );
    }
    peerAddress.sin_port = htons( port );
    memset( &( peerAddress.sin_zero ), '\0', 8 );
    return status;
}

int UDPSocket::getPeerAddr( char* addr, ulong addrSize, ushort* port )
{
    strncpy( addr, inet_ntoa( peerAddress.sin_addr ), addrSize );
    *port = ntohs( peerAddress.sin_port );
    return 0;
}

int UDPSocket::initLocalPort( ushort port, Socket* )
{
    int                         status = 0;
    struct sockaddr_in          myAddr;

    if ( sockFD == -1 )
    {
        status = initSocket( SOCK_DGRAM );
        if ( status != 0 )
        {
            return status;
        }
    }

    if ( localPort != 0 )
    {
        /* Need to create a new socket first */
        localPort = 0;
        status = initSocket( SOCK_DGRAM );
    }

    if ( status == 0 )
    {
        myAddr.sin_family      = AF_INET;
        myAddr.sin_port        = htons( port );
        myAddr.sin_addr.s_addr = INADDR_ANY;
        memset( &( myAddr.sin_zero ), '\0', 8 );
        status = bind( sockFD, ( struct sockaddr* )( &myAddr ), sizeof( struct sockaddr ) );
        if ( status != 0 )
        {
            if ( printErrs )
            {
                perror( "ERROR (UDPSocket::initLocalPort) bind()" );
            }
        }
        else
        {
            struct sockaddr_in temp;
            socklen_t len = sizeof( temp );
            getsockname( sockFD, ( struct sockaddr* )&temp, &len );
            localPort = ntohs( temp.sin_port );

            //localPort = port;
        }
    }
    else
    {
        if ( printErrs )
        {
            fprintf( stderr, "ERROR (UDPSocket::initLocalPort): This socket isn't initialized\n" );
        }
        return -1;
    }

    return status;
}

int UDPSocket::sendData( const void* buf, ulong size )
{
    int                         snt, sel;
    ulong                       totalSent;
    fd_set                      wfds;
    timeval                     tout;
    SocketHeader             hdr;
    const void*                 sendBuf = buf;
#ifdef WIN32
    int                         wsa_errno = 0;
#endif

    if ( buf == 0 )
    {
        fprintf( stderr, "ERROR (UDPSocket::sendData): Passed data pointer is null\n" );
        return -1;
    }

    /* Create a buffer with a header if enabled */
    if ( useHeaders )
    {
        size += hdr.hdrSize();
        outBuf.allocate( size );
        hdr.msgSize = size;
        hdr.copyTo( outBuf.buffer() );
        memcpy( ( ( char* )outBuf.buffer() ) + hdr.hdrSize(), buf, size - hdr.hdrSize() );
        sendBuf = outBuf.buffer();
    }

    /* Select information */
    FD_ZERO( &wfds );
    FD_SET( sockFD, &wfds );
    tout.tv_sec  = ( ulong )timeout;
    tout.tv_usec = ( ulong )( ( timeout - ( ulong )timeout ) * 1000000.0 );

    /* Keep sending until all data is sent */
    totalSent = 0;
    snt       = 0;
    sel       = 1;
    while ( ( totalSent < size ) && ( sel > 0 ) )
    {
        sel = select( sockFD + 1, 0, &wfds, 0, &tout );
        if ( sel > 0 )
        {
            snt = sendto( sockFD, ( ( char* )sendBuf ) + totalSent, size - totalSent, 0,
                          ( struct sockaddr* )( &peerAddress ), sizeof( struct sockaddr ) );
#ifdef WIN32
            wsa_errno = WSAGetLastError();
            if ( ( snt < 0 ) && ( errno != EAGAIN ) && ( wsa_errno != WSAEWOULDBLOCK ) )
#elif LINUX
            if ( ( snt < 0 ) && ( errno != EAGAIN ) && ( errno != EWOULDBLOCK ) )
#endif
            {
                if ( printErrs )
                {
                    perror( "ERROR (UDPSocket::sendData) sendto()" );
                }
                sel = -1;
            }
            else if ( snt >= 0 )
            {
                totalSent += snt;
            }
        }
        else if ( sel == 0 )
        {
            if ( printErrs )
            {
                fprintf( stderr, "ERROR (UDPSocket::sendData) select(): Timed out\n" );
            }
            sel = -1;
        }
        else
        {
            if ( printErrs )
            {
                perror( "ERROR (UDPSocket::sendData) select()" );
            }
            sel = -1;
        }
    }

    if ( totalSent != size )
    {
        if ( printErrs )
        {
            fprintf( stderr, "ERROR (UDPSocket::sendData): Sent only %ld of %ld bytes\n", totalSent, size );
        }
    }

    /* Update send counters */
    incSentMsgCount( 1 );
    incSentByteCount( useHeaders ? totalSent - hdr.hdrSize() : totalSent );

    return ( ( totalSent == size ) ? 0 : -1 );
}

int UDPSocket::recvData( void** buf, ulong* size )
{
    int                         rcv, sel;
    ulong                       totalRecv, expectedSize=0;
#if defined(IRIX) || defined(WIN32)
    int                         addrLen = sizeof( struct sockaddr );
#elif LINUX
    socklen_t                   addrLen = sizeof( struct sockaddr );
#endif
    fd_set                      rfds;
    timeval                     tout;
    SocketHeader             hdr;
    bool                        recvOnce = true;

    /* Select data */
    tout.tv_sec  = ( ulong )timeout;
    tout.tv_usec = ( ulong )( ( timeout - ( ulong )timeout ) * 1000000.0 );

    /* Peek at the header if enabled */
    if ( useHeaders )
    {
        /* Sanity check */
        if ( inBuf.getBufferSize() < hdr.hdrSize() )
        {
            if ( printErrs )
            {
                fprintf( stderr, "ERROR (UDPSocket::recvData): Inbound message buffer is smaller than header size\n" );
            }
            return -1;
        }

        /* Attempt to peek at the header */
        totalRecv = 0;
        rcv       = 0;
        sel       = 1;
        while ( ( totalRecv < hdr.hdrSize() ) && ( sel > 0 ) )
        {
            FD_ZERO( &rfds );
            FD_SET( sockFD, &rfds );

            sel = select( sockFD + 1, &rfds, 0, 0, &tout );
            if ( sel > 0 )
            {
                /* Receive some data */
#ifdef WIN32
                rcv = recvfrom( sockFD, ( char* )inBuf.buffer(), hdr.hdrSize(), MSG_PEEK,
                                ( struct sockaddr* )( &peerAddress ), &addrLen );
#elif LINUX
                rcv = recvfrom( sockFD, inBuf.buffer(), hdr.hdrSize(), MSG_PEEK,
                                ( struct sockaddr* )( &peerAddress ), &addrLen );
#endif
                if ( ( rcv < 0 ) && ( errno != EAGAIN ) )
                {
                    if ( printErrs )
                    {
                        perror( "ERROR (UDPSocket::recvData) recvfrom()" );
                    }
                    return -1;
                }
                else if ( rcv == 0 )
                {
                    if ( printErrs )
                    {
                        fprintf( stderr, "ERROR (UDPSocket::recvData) recvfrom(): Source unavailable\n" );
                    }
                    return -1;
                }
                else
                {
                    totalRecv += rcv;
                }
            }
            else if ( sel == 0 )
            {
                if ( printErrs )
                {
                    fprintf( stderr, "ERROR (UDPSocket::recvData) select(): Timed out\n" );
                }
                return -1;
            }
            else
            {
                if ( printErrs )
                {
                    perror( "ERROR (UDPSocket::recvData) select()" );
                }
                return -1;
            }
        }

        /* Check the header size */
        if ( totalRecv != hdr.hdrSize() )
        {
            if ( printErrs )
            {
                fprintf( stderr, "ERROR (UDPSocket::recvData): Couldn't read header\n" );
            }
            return -1;
        }

        /* Check the header flag */
        hdr.copyFrom( inBuf.buffer() );
        if ( hdr.rexFlag != REXFLAG )
        {
            /* Headers are enabled, but the received packet doesn't appear to have one...
            Print an error but continue as if headers aren't enabled */
            if ( printErrs )
            {
                fprintf( stderr, "ERROR (UDPSocket::recvData): Headers enabled, but received packet has no header (flag = 0x%lx != 0x%x)\n", hdr.rexFlag, REXFLAG );
            }

            /* Since incoming message size is unknown, just receive once */
            recvOnce = true;
        }
        else
        {
            /* Headers enabled, header found...
            Allocate enough buffer for the whole message */
            inBuf.allocate( hdr.msgSize );

            /* Returned buffer will be message after header,
            size is known */
            *buf         = ( ( char* )inBuf.buffer() ) + hdr.hdrSize();
            expectedSize = hdr.msgSize;
            recvOnce     = false;
        }
    }

    if ( recvOnce )
    {
        /* Obtain the inbound buffer pointer, no header to skip,
           size is unknown until message is completely read */
        *buf         = inBuf.buffer();
        expectedSize = inBuf.getBufferSize();
    }

    /* Check for available data */
    totalRecv = 0;
    rcv       = 0;
    sel       = 1;
    while ( ( totalRecv < expectedSize ) && ( sel > 0 ) )
    {
        FD_ZERO( &rfds );
        FD_SET( sockFD, &rfds );

        sel = select( sockFD + 1, &rfds, 0, 0, &tout );
        if ( sel > 0 )
        {
            /* Receive some data */
            rcv = recvfrom( sockFD, ( ( char* )inBuf.buffer() ) + totalRecv, expectedSize - totalRecv, 0,
                            ( struct sockaddr* )( &peerAddress ), &addrLen );
            if ( ( rcv < 0 ) && ( errno != EAGAIN ) )
            {
                if ( printErrs )
                {
                    perror( "ERROR (UDPSocket::recvData) recvfrom()" );
                }
                return -1;
            }
            else if ( rcv == 0 )
            {
                if ( printErrs )
                {
                    fprintf( stderr, "ERROR (UDPSocket::recvData) recvfrom(): Source unavailable\n" );
                }
                return -1;
            }
            else
            {
                totalRecv += rcv;
            }
        }
        else if ( sel == 0 )
        {
            if ( printErrs )
            {
                fprintf( stderr, "ERROR (UDPSocket::recvData) select(): Timed out\n" );
            }
            return -1;
        }
        else
        {
            if ( printErrs )
            {
                perror( "ERROR (UDPSocket::recvData) select()" );
            }
            return -1;
        }

        if ( recvOnce )
        {
            /* Break out of while loop */
            expectedSize = totalRecv;
        }
    }

    if ( totalRecv != expectedSize )
    {
        if ( printErrs )
        {
            fprintf( stderr, "ERROR (UDPSocket::recvData): Received only %ld of %ld bytes\n", totalRecv, expectedSize );
        }
    }

    if ( recvOnce )
    {
        *size = totalRecv;
    }
    else
    {
        /* Header was received also, but don't count it in the size */
        *size = totalRecv - hdr.hdrSize();
    }

    /* Increment the recv counters */
    incRecvMsgCount( 1 );
    incRecvByteCount( *size );

    return ( totalRecv == expectedSize ? 0 : -1 );
}

int UDPSocket::disconnect( void )
{
    return 0;
}

int UDPSocket::setBroadcast( void )
{
    int yes=1;
    return setsockopt( sockFD, SOL_SOCKET, SO_BROADCAST, &yes, sizeof( yes ) );
}
int UDPSocket::joinMulticastGroup( string group )
{

    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr( group.c_str() );
    mreq.imr_interface.s_addr = htonl( INADDR_ANY );

    int result = setsockopt( sockFD, IPPROTO_IP, IP_ADD_MEMBERSHIP, ( char* )&mreq, sizeof( mreq ) );

    return result;
}
int UDPSocket::leaveMulticastGroup( std::string group )
{
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr( group.c_str() );
    mreq.imr_interface.s_addr = htonl( INADDR_ANY );

    return setsockopt( sockFD, IPPROTO_IP, IP_DROP_MEMBERSHIP, ( char* )&mreq, sizeof( mreq ) );
}
/**************** End UDPSocket *****************/

/****************** TCPSocket *******************/
TCPSocket::TCPSocket( void )
{
#ifdef WIN32
    int err;
    WSADATA wsa_data;
    err = WSAStartup( WSOCK32_VER, &wsa_data );

    if ( err != 0 )
    {
        fprintf( stderr, "ERROR (TCPSocket): WSAStartup() failed\n" );
    }
#endif

    initSocket( SOCK_STREAM );
    return;
}

TCPSocket::TCPSocket( const char* addr, ushort port )
{
#ifdef WIN32
    int err;
    WSADATA wsa_data;
    err = WSAStartup( WSOCK32_VER, &wsa_data );

    if ( err != 0 )
    {
        fprintf( stderr, "ERROR (TCPSocket): WSAStartup() failed\n" );
    }
#endif

    initSocket( SOCK_STREAM );
    setPeerAddr( addr, port );
    return;
}

TCPSocket::TCPSocket( const TCPSocket& that ) :
    Socket( that )
{
    return;
}

TCPSocket& TCPSocket::operator=( const TCPSocket& that )
{
    if ( this != &that )
    {
        if ( connected )
        {
            disconnect();
        }
        Socket::operator=( that );
    }
    return *this;
}
TCPSocket::~TCPSocket( void )
{
    if ( sockFD != -1 )
    {
        close( sockFD );
    }

#ifdef WIN32
    WSACleanup();
#endif

    return;
}

int TCPSocket::setPeerAddr( const char* addr, ushort port )
{
    struct hostent*             h      = 0;
    int                         status = 0;
    int                         sockErr;
    fd_set                      wfds;
    timeval                     tout;
#ifdef WIN32
    int                         wsa_errno = 0;
#endif

    /* Get the peer address */
    h = gethostbyname( addr );
    if ( h == 0 )
    {
        /* An error occurred, see h_errno */
        status = -1;
        if ( printErrs )
        {
#ifdef WIN32
            fprintf( stderr, "ERROR (TCPSocket::setPeerAddr) gethostbyname(): An error occurred.\n" );
#elif LINUX
            herror( "ERROR (TCPSocket::setPeerAddr) gethostbyname()" );
#endif
        }
        peerAddress.sin_addr.s_addr = htonl( INADDR_ANY );
        return status;
    }
    else
    {
        /* Address is already in network byte order */
        memcpy( &( peerAddress.sin_addr.s_addr ), h->h_addr_list[0], sizeof( ulong ) );
    }
    peerAddress.sin_family = AF_INET;
    peerAddress.sin_port = htons( port );
    memset( &( peerAddress.sin_zero ), '\0', 8 );

    /* If already connected, disconnect first */
    if ( connected )
    {
        status = initSocket( SOCK_STREAM );
        if ( status != 0 )
        {
            return status;
        }
    }

    /* Connect to remote host */
    status = connect( sockFD, ( sockaddr* )( &peerAddress ), sizeof( struct sockaddr ) );
    if ( status != 0 )
    {
#ifdef WIN32
        wsa_errno = WSAGetLastError();
        if ( ( wsa_errno == WSAEINPROGRESS ) || ( wsa_errno == WSAEWOULDBLOCK ) )
        {
#elif LINUX
        if ( errno == EINPROGRESS )
        {
#endif
            FD_ZERO( &wfds );
            FD_SET( sockFD, &wfds );
            tout.tv_sec  = ( ulong )timeout;
            tout.tv_usec = ( ulong )( ( timeout - ( ulong )timeout ) * 1000000.0 );

            status = select( sockFD + 1, 0, &wfds, 0, &tout );
            if ( status == -1 )
            {
                if ( printErrs )
                {
                    perror( "ERROR (TCPSocket::setPeerAddr) select()" );
                }
                disconnect();
                return status;
            }
            else if ( status == 0 )
            {
                /* select timed out */
                if ( printErrs )
                {
                    fprintf( stderr, "ERROR (TCPSocket::setPeerAddr) select(): Timed out waiting to connect.\n" );
                }
                disconnect();
                return status;
            }
            else
            {
                /* Check if connection completed successfully */
                sockErr = getSocketError();
                if ( sockErr != 0 )
                {
                    if ( printErrs )
                    {
                        fprintf( stderr, "ERROR (TCPSocket::setPeerAddr) connect() in progress: %s\n", strerror( sockErr ) );
                    }
                    disconnect();
                    return -1;
                }
            }
        }
        else
        {
            if ( printErrs )
            {
                perror( "ERROR (TCPSocket::setPeerAddr) connect()" );
            }
            disconnect();
            return status;
        }
    }

    connected = true;

    return 0;
}

int TCPSocket::getPeerAddr( char* addr, ulong addrSize, ushort* port )
{
    int                         status = 0;

#if defined(IRIX) || defined(WIN32)
    int                         addrLen = sizeof( struct sockaddr );
#elif LINUX
    socklen_t                   addrLen = sizeof( struct sockaddr );
#endif

    if ( sockFD == -1 )
    {
        fprintf( stderr, "ERROR (TCPSocket::getPeerAddr): This socket isn't initialized\n" );
        return -1;
    }

    /* Get the peer address struct */
    status = getpeername( sockFD, ( struct sockaddr* )( &peerAddress ), &addrLen );
    if ( status != 0 )
    {
        if ( printErrs )
        {
            perror( "ERROR (TCPSocket::getPeerAddr) getpeername()" );
        }
        return status;
    }

    strncpy( addr, inet_ntoa( peerAddress.sin_addr ), addrSize );
    *port = ntohs( peerAddress.sin_port );

    return 0;
}

int TCPSocket::initLocalPort( ushort port, Socket* newsock )
{
    int                         status = 0;
    struct sockaddr_in          myAddress;
    fd_set                      rfds;
    timeval                     tout;
    int                         newSockFD;

    /* addrLen parameter for accept() */
#if defined(IRIX) || defined(WIN32)
    int                         addrLen = sizeof( struct sockaddr );
#elif LINUX
    socklen_t                   addrLen = sizeof( struct sockaddr );
#endif

    /* Obtain an initialized and unconnected socket */
    if ( connected || ( sockFD == -1 ) )
    {
        status = initSocket( SOCK_STREAM );
        if ( status != 0 )
        {
            return status;
        }
    }

    myAddress.sin_family      = AF_INET;
    myAddress.sin_port        = htons( port );
    myAddress.sin_addr.s_addr = INADDR_ANY;
    memset( &( myAddress.sin_zero ), '\0', 8 );
    status = bind( sockFD, ( struct sockaddr* )( &myAddress ), sizeof( struct sockaddr ) );
    if ( status != 0 )
    {
        if ( printErrs )
        {
            perror( "ERROR (TCPSocket::initLocalPort) bind()" );
        }
        disconnect();
        return status;
    }

    /* Set up to listen for connection */
    status = listen( sockFD, MAX_INBOUND_CALLS );
    if ( status != 0 )
    {
        if ( printErrs )
        {
            perror( "ERROR (TCPSocket::initLocalPort) listen()" );
        }
        disconnect();
        return status;
    }

    /* Select data */
    FD_ZERO( &rfds );
    FD_SET( sockFD, &rfds );
    tout.tv_sec  = ( ulong )timeout;
    tout.tv_usec = ( ulong )( ( timeout - ( ulong )timeout ) * 1000000.0 );

    status = select( sockFD + 1, &rfds, 0, 0, &tout );
    if ( status == -1 )
    {
        if ( printErrs )
        {
            perror( "ERROR (TCPSocket::initLocalPort) select()" );
            disconnect();
            return status;
        }
    }
    else if ( status == 0 )
    {
        /* No connections pending */
        disconnect();
        return -1;
    }

    /* Accept first pending connection */
    status = accept( sockFD, ( struct sockaddr* )( &peerAddress ), &addrLen );
    if ( status != -1 )
    {
        newSockFD = status;

        if ( newsock == 0 )
        {
            /* No new socket object was provided, so this object will handle
            the recently accepted connection */
            status = close( sockFD );
            if ( status != 0 )
            {
                if ( printErrs )
                {
                    perror( "ERROR (TCPSocket::initLocalPort) close()" );
                }
            }
            initAcceptedConnection( newSockFD );
        }
        else
        {
            /* A new socket object was provided, so it will handle the
            recently accepted connection if it is a TCP socket */
            TCPSocket* tcpsock = ( TCPSocket* )newsock;
            if ( tcpsock != 0 )
            {
                tcpsock->initAcceptedConnection( newSockFD );
            }
            else
            {
                if ( printErrs )
                {
                    fprintf( stderr, "ERROR (TCPSocket::initLocalPort) Passed socket is not a TCPSocket\n" );
                }
                status = -1;
            }
        }
    }
    else
    {
        if ( printErrs )
        {
            fprintf( stderr, "ERROR (TCPSocket::initLocalPort) accept()\n" );
        }
        disconnect();
    }

    return status;
}

int TCPSocket::sendData( const void* buf, ulong size )
{
    int                         snt, sel;
    ulong                       totalSent;
    fd_set                      wfds;
    timeval                     tout;
    SocketHeader             hdr;
    const void*                       sendBuf = buf;
#ifdef WIN32
    int                         wsa_errno = 0;
#endif

    if ( buf == 0 )
    {
        fprintf( stderr, "ERROR (TCPSocket::sendData): Passed data pointer is null\n" );
        return -1;
    }

    if ( ( sockFD == -1 ) || ( !connected ) )
    {
        fprintf( stderr, "ERROR (TCPSocket::sendData): This socket isn't connected\n" );
        return -1;
    }

    /* Create a buffer with a header if enabled */
    if ( useHeaders )
    {
        size += hdr.hdrSize();
        outBuf.allocate( size );
        hdr.msgSize = size;
        hdr.copyTo( outBuf.buffer() );
        memcpy( ( ( char* )outBuf.buffer() ) + hdr.hdrSize(), buf, size - hdr.hdrSize() );
        sendBuf = outBuf.buffer();
    }

    /* Select information */
    FD_ZERO( &wfds );
    FD_SET( sockFD, &wfds );
    tout.tv_sec  = ( ulong )timeout;
    tout.tv_usec = ( ulong )( ( timeout - ( ulong )timeout ) * 1000000.0 );

    /* Keep sending until all data is sent */
    totalSent = 0;
    snt       = 0;
    sel       = 1;
    while ( ( totalSent < size ) && ( sel > 0 ) )
    {
        sel = select( sockFD + 1, 0, &wfds, 0, &tout );
        if ( sel > 0 )
        {
            snt = send( sockFD, ( ( char* )sendBuf ) + totalSent, size - totalSent, 0 );
#ifdef WIN32
            if ( ( snt < 0 ) && ( errno != EAGAIN ) && ( wsa_errno != WSAEWOULDBLOCK ) )
            {
#elif LINUX
            if ( ( snt < 0 ) && ( errno != EAGAIN ) && ( errno != EWOULDBLOCK ) )
            {
#endif
                if ( printErrs )
                {
                    perror( "ERROR (TCPSocket::sendData) send()" );
                }
                sel = -1;
            }
            else if ( snt >= 0 )
            {
                totalSent += snt;
            }
        }
        else if ( sel == 0 )
        {
            if ( printErrs )
            {
                fprintf( stderr, "ERROR (TCPSocket::sendData) select(): Timed out\n" );
            }
            sel = -1;
        }
        else
        {
            if ( printErrs )
            {
                perror( "ERROR (TCPSocket::sendData) select()" );
            }
            sel = -1;
        }
    }

    if ( totalSent != size )
    {
        if ( printErrs )
        {
            fprintf( stderr, "ERROR (TCPSocket::sendData): Sent only %ld of %ld bytes\n", totalSent, size );
        }
    }

    /* Update send counters */
    incSentMsgCount( 1 );
    incSentByteCount( useHeaders ? totalSent - hdr.hdrSize() : totalSent );

    return ( ( totalSent == size ) ? 0 : -1 );
}

int TCPSocket::recvData( void** buf, ulong* size )
{
    int                         rcv, sel;
    ulong                       totalRecv, expectedSize=0;
    fd_set                      rfds;
    timeval                     tout;
    SocketHeader             hdr;
    bool                        recvOnce = true;

    /* Select data */
    tout.tv_sec  = ( ulong )timeout;
    tout.tv_usec = ( ulong )( ( timeout - ( ulong )timeout ) * 1000000.0 );

    /* Peek at the header if enabled */
    if ( useHeaders )
    {
        /* Sanity check */
        if ( inBuf.getBufferSize() < hdr.hdrSize() )
        {
            if ( printErrs )
            {
                fprintf( stderr, "ERROR (TCPSocket::recvData): Inbound message buffer is smaller than header size\n" );
            }
            return -1;
        }

        /* Attempt to peek at the header */
        totalRecv = 0;
        rcv       = 0;
        sel       = 1;
        while ( ( totalRecv < hdr.hdrSize() ) && ( sel > 0 ) )
        {
            FD_ZERO( &rfds );
            FD_SET( sockFD, &rfds );

            sel = select( sockFD + 1, &rfds, 0, 0, &tout );
            if ( sel > 0 )
            {
                /* Receive some data */
#ifdef WIN32
                rcv = recv( sockFD, ( char* )inBuf.buffer(), hdr.hdrSize(), MSG_PEEK );
#elif LINUX
                rcv = recv( sockFD, inBuf.buffer(), hdr.hdrSize(), MSG_PEEK );
#endif
                if ( ( rcv < 0 ) && ( errno != EAGAIN ) )
                {
                    if ( printErrs )
                    {
                        perror( "ERROR (TCPSocket::recvData) recv()" );
                    }
                    return -1;
                }
                else if ( rcv == 0 )
                {
                    if ( printErrs )
                    {
                        fprintf( stderr, "ERROR (TCPSocket::recvData) recv(): Source unavailable\n" );
                    }
                    return -1;
                }
                else
                {
                    totalRecv += rcv;
                }
            }
            else if ( sel == 0 )
            {
                if ( printErrs )
                {
                    fprintf( stderr, "ERROR (TCPSocket::recvData) select(): Timed out\n" );
                }
                return -1;
            }
            else
            {
                if ( printErrs )
                {
                    perror( "ERROR (TCPSocket::recvData) select()" );
                }
                return -1;
            }
        }

        /* Check the header size */
        if ( totalRecv != hdr.hdrSize() )
        {
            if ( printErrs )
            {
                fprintf( stderr, "ERROR (TCPSocket::recvData): Couldn't read header\n" );
            }
            return -1;
        }

        /* Check the header flag */
        hdr.copyFrom( inBuf.buffer() );
        if ( hdr.rexFlag != REXFLAG )
        {
            /* Headers are enabled, but the received packet doesn't appear to have one...
            Print an error but continue as if headers aren't enabled */
            if ( printErrs )
            {
                fprintf( stderr, "ERROR (TCPSocket::recvData): Headers enabled, but received packet has no header (flag = 0x%lx != 0x%x)\n", hdr.rexFlag, REXFLAG );
            }

            /* Since incoming message size is unknown, just receive once */
            recvOnce = true;
        }
        else
        {
            /* Headers enabled, header found...
            Allocate enough buffer for the whole message */
            inBuf.allocate( hdr.msgSize );

            /* Returned buffer will be message after header,
            size is known */
            *buf         = ( ( char* )inBuf.buffer() ) + hdr.hdrSize();
            expectedSize = hdr.msgSize;
            recvOnce     = false;
        }
    }

    if ( recvOnce )
    {
        /* Obtain the inbound buffer pointer, no header to skip,
           size is unknown until message is completely read */
        *buf         = inBuf.buffer();
        expectedSize = inBuf.getBufferSize();
    }

    /* Check for available data */
    totalRecv = 0;
    rcv       = 0;
    sel       = 1;
    while ( ( totalRecv < expectedSize ) && ( sel > 0 ) )
    {
        FD_ZERO( &rfds );
        FD_SET( sockFD, &rfds );

        sel = select( sockFD + 1, &rfds, 0, 0, &tout );
        if ( sel > 0 )
        {
            /* Receive some data */
            rcv = recv( sockFD, ( ( char* )inBuf.buffer() ) + totalRecv, expectedSize - totalRecv, 0 );
            if ( ( rcv < 0 ) && ( errno != EAGAIN ) )
            {
                if ( printErrs )
                {
                    perror( "ERROR (TCPSocket::recvData) recv()" );
                }
                return -1;
            }
            else if ( rcv == 0 )
            {
                if ( printErrs )
                {
                    fprintf( stderr, "ERROR (TCPSocket::recvData) recv(): Source unavailable\n" );
                }
                return -1;
            }
            else
            {
                totalRecv += rcv;
            }
        }
        else if ( sel == 0 )
        {
            if ( printErrs )
            {
                fprintf( stderr, "ERROR (TCPSocket::recvData) select(): Timed out\n" );
            }
            return -1;
        }
        else
        {
            if ( printErrs )
            {
                perror( "ERROR (TCPSocket::recvData) select()" );
            }
            return -1;
        }

        if ( recvOnce )
        {
            /* Break out of while loop */
            expectedSize = totalRecv;
        }
    }

    if ( totalRecv != expectedSize )
    {
        if ( printErrs )
        {
            fprintf( stderr, "ERROR (TCPSocket::recvData): Received only %ld of %ld bytes\n", totalRecv, expectedSize );
        }
    }

    if ( recvOnce )
    {
        *size = totalRecv;
    }
    else
    {
        /* Header was received also, but don't count it in the size */
        *size = totalRecv - hdr.hdrSize();
    }

    /* Increment the recv counters */
    incRecvMsgCount( 1 );
    incRecvByteCount( *size );

    return ( totalRecv == expectedSize ? 0 : -1 );
}

int TCPSocket::disconnect( void )
{
    int                         status = 0;

    status = initSocket( SOCK_STREAM );

    return status;
}

int TCPSocket::initAcceptedConnection( int newFD )
{
    int                         status;

    sockFD = newFD;
    connected = true;

    /* Make it non-blocking */
    status = setNonBlocking();

    /* Set to reuse address */
    if ( status == 0 )
    {
        status = setReuseAddr();
    }

    return status;
}
/**************** End TCPSocket *****************/

} // end namespace support
} // end namespace avtas

