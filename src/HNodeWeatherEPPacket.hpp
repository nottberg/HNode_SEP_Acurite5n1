#ifndef __HNODE_WEATHER_EPPACKET_H__
#define __HNODE_WEATHER_EPPACKET_H__

#include <stdint.h>

typedef enum HNodeWeatherEPPacketTypeEnum
{
    HNWEPP_TYPE_NOT_SET,
    HNWEPP_TYPE_HNW_MEASUREMENT
}HNWEPP_TYPE_T;

typedef struct HNWEPPPacketData
{
    uint32_t type;
    uint32_t param[6];
    uint32_t payloadLength;
    uint8_t  payload[1024];
}HNWEPP_PDATA_T;

class HNodeWeatherEPPacket
{
    private:
        HNWEPP_PDATA_T packetData;

    public:
        HNodeWeatherEPPacket();
       ~HNodeWeatherEPPacket();

        void setType( HNWEPP_TYPE_T value );
        HNWEPP_TYPE_T getType();

        void setParam( int index, uint32_t value );
        uint32_t getParam( int index );

        void setPayloadLength( uint32_t length );
        uint32_t getPayloadLength();
        uint8_t* getPayloadPtr();

        uint32_t getPacketLength();
        uint8_t* getPacketPtr();
        uint32_t getMaxPacketLength();
};

#endif // __HNODE_WEATHER_EPPACKET_H__
