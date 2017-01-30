#include <stdint.h>
#include <iostream>
#include <cstddef>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <errno.h>

#include "HNodeSEPPacket.hpp"
#include "HNodeSensorMeasurement.hpp"

int main()
{
    struct sockaddr_un addr;
    char *str;

    memset(&addr, 0, sizeof(struct sockaddr_un));  // Clear address structure
    addr.sun_family = AF_UNIX;                     // UNIX domain address

    // addr.sun_path[0] has already been set to 0 by memset() 

    // Abstract socket with name @acrt5n1d_readings
    str = "acrt5n1d_readings";
    strncpy( &addr.sun_path[1], str, strlen(str) );

    int fd = socket( AF_UNIX, SOCK_SEQPACKET, 0 );

    if( connect( fd, (struct sockaddr *) &addr, sizeof( sa_family_t ) + strlen( str ) + 1 ) == 0 )
    {
        while( 1 )
        {
            HNodeSEPPacket    packet;
            HNodeSensorMeasurement reading;
            uint32_t recvd = 0;

            //std::cout << "start recvd..." << std::endl;

            recvd += recv( fd, packet.getPacketPtr(), packet.getMaxPacketLength(), 0 );

            switch( packet.getType() )
            {
                case HNSEPP_TYPE_HNS_MEASUREMENT:
                {
                    reading.parsePacketData( packet.getPayloadPtr(), recvd );

                    std::cout << reading.getAsStr() << std::endl;
                    //std::cout << "recvd: " << recvd << std::endl;
                }
                break;

                default:
                {
                    std::cout << "Unknown Packet Type - len: " << recvd << "  type: " << packet.getType() << std::endl;
                }
                break;
            }
        }
    }
    else
    {
        std::cout << "connection error: " << errno << "  " << strerror(errno) << std::endl;
    }

    close(fd);
}

