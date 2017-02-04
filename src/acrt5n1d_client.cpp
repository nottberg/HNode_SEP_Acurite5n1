#include <stdint.h>
#include <iostream>
#include <cstddef>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <errno.h>

#include <boost/program_options.hpp>

#include "HNodeSEPPacket.hpp"
#include "HNodeSensorMeasurement.hpp"

namespace po = boost::program_options;

int main( int argc, char *argv[] )
{
    struct sockaddr_un addr;
    char *str;

    // Declare the supported options.
    po::options_description desc( "acrt5n1d client utility" );
    desc.add_options()
        ("help", "produce help message")
        ("ping", "send a ping packet")
        ("reset", "send a reset packet")
    ;

    po::variables_map vm;
    po::store( po::parse_command_line( argc, argv, desc ), vm );
    po::notify( vm );    

    if( vm.count( "help" ) )
    {
        std::cout << desc << "\n";
        return 1;
    }

    memset(&addr, 0, sizeof(struct sockaddr_un));  // Clear address structure
    addr.sun_family = AF_UNIX;                     // UNIX domain address

    // addr.sun_path[0] has already been set to 0 by memset() 

    // Abstract socket with name @acrt5n1d_readings
    str = "acrt5n1d_readings";
    strncpy( &addr.sun_path[1], str, strlen(str) );

    int fd = socket( AF_UNIX, SOCK_SEQPACKET, 0 );

    // Establish the connection.
    if( connect( fd, (struct sockaddr *) &addr, sizeof( sa_family_t ) + strlen( str ) + 1 ) == 0 )
    {

        if( vm.count("reset") )
        {
            HNodeSEPPacket packet;
            uint32_t length;

            packet.setType( HNSEPP_TYPE_HNS_RESET );

            std::cout << "Sending a RESET packet...";

            length = send( fd, packet.getPacketPtr(), packet.getPacketLength(), MSG_NOSIGNAL );

            std::cout << length << " bytes sent." << std::endl;
        }
        else if( "ping" )
        {
            HNodeSEPPacket packet;
            uint32_t length;

            packet.setType( HNSEPP_TYPE_HNS_PING );

            std::cout << "Sending a PING packet...";

            length = send( fd, packet.getPacketPtr(), packet.getPacketLength(), MSG_NOSIGNAL );

            std::cout << length << " bytes sent." << std::endl;
        }

        // Listen for packets
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

                case HNSEPP_TYPE_HNS_STATUS:
                {
                    std::string health;
                    struct timeval statusTime;
                    struct timeval lastMeasurementTime;
                    unsigned long measurementCount;
                    std::string msg;

                    //reading.parsePacketData( packet.getPayloadPtr(), recvd );

                    //std::cout << reading.getAsStr() << std::endl;
                    std::cout << "recvd: " << recvd << std::endl;
                    
                    health = ( packet.getSensorIndex() == 1 ) ? "OK" : "Degraded";

                    statusTime.tv_sec  = packet.getParam( 0 );
                    statusTime.tv_usec = packet.getParam( 1 );

                    lastMeasurementTime.tv_sec  = packet.getParam( 2 );
                    lastMeasurementTime.tv_usec = packet.getParam( 3 );

                    measurementCount = packet.getParam( 4 );
       
                    msg.assign( (const char *)packet.getPayloadPtr(), packet.getPayloadLength() );

                    std::cout << "Status: " << health;
                    std::cout << "  TS: " << statusTime.tv_sec;
                    std::cout << "  MTS: " << lastMeasurementTime.tv_sec;
                    std::cout << "  MC: " << measurementCount;
                    std::cout << "  Msg: " << msg;
                    std::cout << std::endl;

#if 0

    packet.setSensorIndex( healthOK );

    packet.setParam( 0, curTS->tv_sec );
    packet.setParam( 1, curTS->tv_usec );

    packet.setParam( 2, lastReadingTS.tv_sec );
    packet.setParam( 3, lastReadingTS.tv_usec );

    packet.setParam( 4, demod.getMeasurementCount() );

    packet.setPayloadLength( curErrMsg.size() );
    memcpy( packet.getPayloadPtr(), curErrMsg.c_str(), curErrMsg.size() );
#endif

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

