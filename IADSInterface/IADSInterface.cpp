#include "IADSInterface.h"

#include "Socket.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>

IADSInterface::IADSInterface( void )
{
}

IADSInterface::IADSInterface( std::string peer, uint16_t port, uint16_t num=0 )
    : _sender( new avtas::support::UDPSocket( peer.c_str(), port ) )
{
    if ( !num )
    {
        return;
    }
    setNumParameters( num );
}
IADSInterface::~IADSInterface( void )
{
    if ( _data )
    {
        delete[] _data;
    }
    delete _sender;
}
IADSInterface &IADSInterface::setNumParameters( int num )
{
    _numParam = num;
    _size = num*sizeof( data_T );
    if ( nullptr != _data )
    {
        delete[] _data;
        _data = nullptr;
    }
    _data = new data_T[num]();
    return *this;
}
IADSInterface &IADSInterface::setParameter( int index, int value )
{
    if ( index < _numParam )
    {
        _data[index].i = value;
    }
    return *this;
}
IADSInterface &IADSInterface::setParameter( int index, float value )
{
    if ( index < _numParam )
    {
        _data[index].f = value;
    }
    return *this;
}
IADSInterface &IADSInterface::setParameter( int index, double value )
{
    if ( index < _numParam )
    {
        _data[index].f = value;
    }

    return *this;
}

float IADSInterface::getParameter( int index )
{
    if ( index < _numParam )
    {
        return _data[index].f;
    }

    return 0.0;
}
IADSInterface &IADSInterface::sendData( void )
{
    _sender->sendData( _data, _size );
    return *this;
}

