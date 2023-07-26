#pragma once

#include <string>
#include <netdb.h>

namespace avtas
{
    namespace support
    {
        class UDPSocket;
    }
}

/*! \brief This class packages and sends data to the IADS source server */
class IADSInterface
{
public: // Constructor & Destructor
    /*! \brief Constructor */
    IADSInterface( void );

    /*! \brief Constructor
     * @param[in] peer Network address of source server
     * @param[in] port Receiving port of source server
     * @param[in] num Number of parameters in data message */
    IADSInterface( std::string peer, uint16_t port, uint16_t num );

    /*! \brief Destructor */
    ~IADSInterface( void );

    /*! \brief Set the number of parameters in data message
     * @param[in] num Number of parameters
     * @return Reference to this object */
    IADSInterface &setNumParameters( int num );

    /*! \brief Set the value of the parameter
     * @param[in] index Index of the parameter (usually by enumeration)
     * @param[in] value Value of parameter
     * @return Reference to this object */
    IADSInterface &setParameter( int index, int value );

    /*! \brief Set the value of the parameter
     * @param[in] index Index of the parameter (usually by enumeration)
     * @param[in] value Value of parameter
     * @return Reference to this object */
    IADSInterface &setParameter( int index, float value );

    /*! \brief Set the value of the parameter
     * @param[in] index Index of the parameter (usually by enumeration)
     * @param[in] value Value of parameter
     * @return Reference to this object */
    IADSInterface &setParameter( int index, double value );

    /*! \brief Set the value of the parameter
     * @param[in] index Index of the parameter (usually by enumeration)
     * @param[in] value Value of parameter
     * @return Reference to this object */
    float getParameter( int index );

    /*! \brief Send the data message to the source server
     * @return Reference to this object */
    IADSInterface &sendData( void );

protected:
    avtas::support::UDPSocket *_sender; /*!< \brief Socket handler */
    int _numParam = -1; /*!< \brief Number of parameters */
    typedef union
    {
        int i;
        float f;
    } data_T;
    data_T *_data = nullptr; /*!< \brief Pointer to data message */
    int _size = 0; /*!< \brief Size of data message */
};
