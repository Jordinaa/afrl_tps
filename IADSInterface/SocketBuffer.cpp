#include "SocketBuffer.h"

namespace avtas
{
namespace support
{

SocketBuffer::SocketBuffer( void ) :
    buf( ( void* )0 ),
    bufSize( 0 ),
    useLimit( false ),
    sizeLimit( 0 )
{
    return;
}

SocketBuffer::SocketBuffer( const SocketBuffer& that ) :
    bufSize( that.bufSize ),
    useLimit( that.useLimit ),
    sizeLimit( that.sizeLimit )
{
    if ( bufSize )
    {
        buf = malloc( bufSize );
        memcpy( buf, that.buf, bufSize );
    }
    else
    {
        buf = ( void* )0;
    }
    return;
}

SocketBuffer& SocketBuffer::operator=( const SocketBuffer& that )
{
    if ( this != &that )
    {
        bufSize = that.bufSize;
        useLimit = that.useLimit;
        sizeLimit = that.sizeLimit;
        if ( buf != ( void* )0 )
        {
            free( buf );
        }
        if ( bufSize )
        {
            buf = malloc( bufSize );
            memcpy( buf, that.buf, bufSize );
        }
        else
        {
            buf = ( void* )0;
        }
    }
    return *this;
}

SocketBuffer::~SocketBuffer( void )
{
    if ( buf != ( void* )0 )
    {
        free( buf );
        buf = ( void* )0;
    }
    return;
}

void SocketBuffer::allocate( ulong size )
{
    /* Limit to the size limit if enabled */
    if ( useLimit && ( size > sizeLimit ) )
    {
        size = sizeLimit;
    }

    /* If buffer isn't yet big enough, reallocate it */
    if ( bufSize < size )
    {
        if ( buf != ( void* )0 )
        {
            free( buf );
        }
        buf = malloc( size );
        bufSize = size;
    }

    return;
}

void SocketBuffer::setBufferSize( ulong size )
{
    /* Limit to the size limit if enabled */
    if ( useLimit && ( size > sizeLimit ) )
    {
        size = sizeLimit;
    }

    /* If buffer isn't the requested size, reallocate it */
    if ( bufSize != size )
    {
        if ( buf != ( void* )0 )
        {
            free( buf );
        }
        if ( size != 0 )
        {
            buf = malloc( size );
            bufSize = size;
        }
    }

    return;
}

void SocketBuffer::enableSizeLimit( void )
{
    useLimit = true;

    /* If buffer is above size limit, reallocate it */
    if ( bufSize > sizeLimit )
    {
        if ( buf != ( void* )0 )
        {
            free( buf );
        }
        buf = malloc( sizeLimit );
        bufSize = sizeLimit;
    }

    return;
}

void SocketBuffer::setSizeLimit( ulong size )
{
    sizeLimit = size;

    /* If buffer is above size limit, reallocate it */
    if ( useLimit && ( bufSize > sizeLimit ) )
    {
        if ( buf != ( void* )0 )
        {
            free( buf );
        }
        buf = malloc( sizeLimit );
        bufSize = sizeLimit;
    }

    return;
}

void SocketBuffer::copyFrom( void* data, ulong size )
{
    /* Limit copy amount to the buffer size */
    if ( size > bufSize )
    {
        size = bufSize;
    }

    allocate( size );
    memcpy( buf, data, size );

    return;
}

void SocketBuffer::copyTo( void* data, ulong size )
{
    /* Limit copy amount to the buffer size */
    if ( size > bufSize )
    {
        size = bufSize;
    }

    allocate( size );
    memcpy( data, buf, size );

    return;
}

} // end namespace support
} // end namespace avtas

