#include "HNodeSEPPacket.hpp"

HNodeSEPPacket::HNodeSEPPacket()
{

}

HNodeSEPPacket::~HNodeSEPPacket()
{

}

void 
HNodeSEPPacket::setType( HNSEPP_TYPE_T value )
{
    packetData.type = (HNSEPP_TYPE_T) value;
}

HNSEPP_TYPE_T 
HNodeSEPPacket::getType()
{
    return (HNSEPP_TYPE_T) packetData.type;
}

void 
HNodeSEPPacket::setParam( int index, uint32_t value )
{
    if( index >= 6 )
        return;

    packetData.param[ index ] = value;
}

uint32_t
HNodeSEPPacket::getParam( int index )
{
    if( index >= 6 )
        return -1;

    return packetData.param[ index ];
}

void 
HNodeSEPPacket::setPayloadLength( uint32_t length )
{
    if( length > sizeof( packetData.payload ) )
        length = sizeof( packetData.payload );

    packetData.payloadLength = length;
}

uint32_t 
HNodeSEPPacket::getPayloadLength()
{
    return packetData.payloadLength;
}

uint8_t* 
HNodeSEPPacket::getPayloadPtr()
{
    return packetData.payload;
}

uint32_t 
HNodeSEPPacket::getPacketLength()
{
    return packetData.payloadLength + ( sizeof( uint32_t ) * 8 );
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



