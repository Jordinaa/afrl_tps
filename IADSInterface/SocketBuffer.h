// File:        SocketBuffer.h
// Date:        6/27/07
// Author:      Tim Harris
// Description: Represents a dynamically allocated buffer that cuts down
// on hash fragmentation by only allocating new memory when the current
// buffer is too small

#ifndef _AVTAS_SUPPORT_SOCKETBUFFER_H_
#define _AVTAS_SUPPORT_SOCKETBUFFER_H_

#include <stdlib.h>
#include <string.h>

#ifndef ulong
typedef unsigned long int  ulong;
#endif
#ifndef ushort
typedef unsigned short int ushort;
#endif
#ifndef uchar
typedef unsigned char      uchar;
#endif

namespace avtas
{
namespace support
{

class SocketBuffer
{
private:
    void*                       buf;
    ulong                       bufSize;
    bool                        useLimit;
    ulong                       sizeLimit;
public:
    SocketBuffer( void );
    SocketBuffer( const SocketBuffer& that );
    SocketBuffer&                     operator=( const SocketBuffer& that );
    ~SocketBuffer( void );
    void                        allocate( ulong size );
    void                        setBufferSize( ulong size );
    inline ulong                getBufferSize( void )
    {
        return bufSize;
    }
    void                        enableSizeLimit( void );
    inline void                 disableSizeLimit( void )
    {
        useLimit = false;
    }
    void                        setSizeLimit( ulong size );
    inline ulong                getSizeLimit( void )
    {
        return sizeLimit;
    }
    inline void*                buffer( void )
    {
        return buf;
    }
    void                        copyFrom( void* data, ulong size );
    void                        copyTo( void* data, ulong size );
};

} // end namespace support
} // end namespace avtas

#endif // defined _AVTAS_SUPPORT_SOCKETBUFFER_H_

