#ifndef __HNODE_SEP_PACKET_H__
#define __HNODE_SEP_PACKET_H__

#include <stdint.h>

typedef enum HNodeSEPPacketTypeEnum
{
    HNSEPP_TYPE_NOT_SET,
    HNSEPP_TYPE_HNW_MEASUREMENT
}HNSEPP_TYPE_T;

typedef struct HNSEPPPacketData
{
    uint32_t type;
    uint32_t param[6];
    uint32_t payloadLength;
    uint8_t  payload[1024];
}HNSEPP_PDATA_T;

class HNodeSEPPacket
{
    private:
        HNSEPP_PDATA_T packetData;

    public:
        HNodeSEPPacket();
       ~HNodeSEPPacket();

        void setType( HNSEPP_TYPE_T value );
        HNSEPP_TYPE_T getType();

        void setParam( int index, uint32_t value );
        uint32_t getParam( int index );

        void setPayloadLength( uint32_t length );
        uint32_t getPayloadLength();
        uint8_t* getPayloadPtr();

        uint32_t getPacketLength();
        uint8_t* getPacketPtr();
        uint32_t getMaxPacketLength();
};

#endif // __HNODE_SEP_PACKET_H__
