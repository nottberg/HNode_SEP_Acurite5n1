#include <arpa/inet.h>

#include "HNodeSEPPacket.hpp"

HNodeSEPPacket::HNodeSEPPacket()
{
    packetData.payloadLength = htonl( 0 );
}

HNodeSEPPacket::~HNodeSEPPacket()
{

}

void 
HNodeSEPPacket::setType( HNSEPP_TYPE_T value )
{
    packetData.type = (HNSEPP_TYPE_T) htonl(  value );
}

HNSEPP_TYPE_T 
HNodeSEPPacket::getType()
{
    return (HNSEPP_TYPE_T) ntohl( packetData.type );
}

void 
HNodeSEPPacket::setSensorIndex( uint32_t value )
{
    packetData.sensorIndex = htonl( value );
}

uint32_t
HNodeSEPPacket::getSensorIndex()
{
    return ntohl( packetData.sensorIndex );
}

void 
HNodeSEPPacket::setParam( int index, uint32_t value )
{
    if( index >= 5 )
        return;

    packetData.param[ index ] = htonl( value );
}

uint32_t
HNodeSEPPacket::getParam( int index )
{
    if( index >= 5 )
        return -1;

    return ntohl( packetData.param[ index ] );
}

void 
HNodeSEPPacket::setPayloadLength( uint32_t length )
{
    if( length > sizeof( packetData.payload ) )
        length = sizeof( packetData.payload );

    packetData.payloadLength = htonl( length );
}

uint32_t 
HNodeSEPPacket::getPayloadLength()
{
    return ntohl( packetData.payloadLength );
}

uint8_t* 
HNodeSEPPacket::getPayloadPtr()
{
    return packetData.payload;
}

uint32_t 
HNodeSEPPacket::getPacketLength()
{
    return getPayloadLength() + ( sizeof( uint32_t ) * 8 );
}

uint8_t* 
HNodeSEPPacket::getPacketPtr()
{
    return (uint8_t*) &packetData;
}

uint32_t 
HNodeSEPPacket::getMaxPacketLength()
{
    return (sizeof packetData);
}



