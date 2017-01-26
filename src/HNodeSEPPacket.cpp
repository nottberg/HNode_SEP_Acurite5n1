#include "HNodeSEPPacket.hpp"

HNodeWeatherEPPacket::HNodeWeatherEPPacket()
{

}

HNodeWeatherEPPacket::~HNodeWeatherEPPacket()
{

}

void 
HNodeWeatherEPPacket::setType( HNWEPP_TYPE_T value )
{
    packetData.type = (HNWEPP_TYPE_T) value;
}

HNWEPP_TYPE_T 
HNodeWeatherEPPacket::getType()
{
    return (HNWEPP_TYPE_T) packetData.type;
}

void 
HNodeWeatherEPPacket::setParam( int index, uint32_t value )
{
    if( index >= 6 )
        return;

    packetData.param[ index ] = value;
}

uint32_t
HNodeWeatherEPPacket::getParam( int index )
{
    if( index >= 6 )
        return -1;

    return packetData.param[ index ];
}

void 
HNodeWeatherEPPacket::setPayloadLength( uint32_t length )
{
    if( length > sizeof( packetData.payload ) )
        length = sizeof( packetData.payload );

    packetData.payloadLength = length;
}

uint32_t 
HNodeWeatherEPPacket::getPayloadLength()
{
    return packetData.payloadLength;
}

uint8_t* 
HNodeWeatherEPPacket::getPayloadPtr()
{
    return packetData.payload;
}

uint32_t 
HNodeWeatherEPPacket::getPacketLength()
{
    return packetData.payloadLength + ( sizeof( uint32_t ) * 8 );
}

uint8_t* 
HNodeWeatherEPPacket::getPacketPtr()
{
    return (uint8_t*) &packetData;
}

uint32_t 
HNodeWeatherEPPacket::getMaxPacketLength()
{
    return (sizeof packetData);
}



