#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/unistd.h>
#include <sys/select.h>
#include <sys/un.h>
#include <sys/socket.h>
#include <sys/epoll.h>

#include <libdaemon/dfork.h>
#include <libdaemon/dsignal.h>
#include <libdaemon/dlog.h>
#include <libdaemon/dpid.h>
#include <libdaemon/dexec.h>

#include <map>

#include "HNodeSensorMeasurement.hpp"
#include "HNodeSEPPacket.hpp"
#include "Acurite5N1RTL433.hpp"

#define MAXEVENTS 8

typedef enum DaemonProcessResultEnum
{
  DP_RESULT_SUCCESS,
  DP_RESULT_FAILURE
}DP_RESULT_T;

class ClientRecord
{
    private:
        int clientFD;

    public:
        ClientRecord();
       ~ClientRecord();

        void setFD( int value );
        int getFD();
};

ClientRecord::ClientRecord()
{
    clientFD = 0;
}

ClientRecord::~ClientRecord()
{

}

void 
ClientRecord::setFD( int value )
{
    clientFD = value;
}

int 
ClientRecord::getFD()
{
    return clientFD;
}

class DaemonProcess : public RTL433DemodNotify
{ 
    private:
        bool quit;

        int epollFD;
        int signalFD; 
        int acceptFD;
       
        struct epoll_event event;
        struct epoll_event *events;

        std::map< int, ClientRecord > clientMap;

        RTL433Demodulator demod;

        bool           sendStatus;

        bool           healthOK;
        std::string    curErrMsg;
        struct timeval lastReadingTS;

        DP_RESULT_T addSocketToEPoll( int sfd );
        DP_RESULT_T removeSocketFromEPoll( int sfd );        
      
        DP_RESULT_T processNewClientConnections();
        DP_RESULT_T processClientRequest( int efd );
        DP_RESULT_T closeClientConnection( int cfd );

        void sendStatusPacket();

    public:
        DaemonProcess();
       ~DaemonProcess();

        DP_RESULT_T init();

        DP_RESULT_T addSignalSocket( int sfd );

        DP_RESULT_T openListenerSocket();

        DP_RESULT_T run();

        virtual void notifyNewMeasurement( uint32_t sensorIndex, HNodeSensorMeasurement &reading );
        virtual void signalError( std::string errMsg );
        virtual void signalRunning();
};

DaemonProcess::DaemonProcess()
{
    quit = false;

    sendStatus = false;

    healthOK   = false;

    curErrMsg.empty();

    lastReadingTS.tv_sec  = 0;
    lastReadingTS.tv_usec = 0;
}

DaemonProcess::~DaemonProcess()
{

}

DP_RESULT_T
DaemonProcess::addSocketToEPoll( int sfd )
{
    int flags, s;

    flags = fcntl( sfd, F_GETFL, 0 );
    if( flags == -1 )
    {
        daemon_log( LOG_ERR, "Failed to get socket flags: %s", strerror(errno) );
        return DP_RESULT_FAILURE;
    }

    flags |= O_NONBLOCK;
    s = fcntl( sfd, F_SETFL, flags );
    if( s == -1 )
    {
        daemon_log( LOG_ERR, "Failed to set socket flags: %s", strerror(errno) );
        return DP_RESULT_FAILURE; 
    }

    event.data.fd = sfd;
    event.events = EPOLLIN | EPOLLET;
    s = epoll_ctl( epollFD, EPOLL_CTL_ADD, sfd, &event );
    if( s == -1 )
    {
        return DP_RESULT_FAILURE;
    }

    return DP_RESULT_SUCCESS;
}

DP_RESULT_T
DaemonProcess::removeSocketFromEPoll( int sfd )
{
    int s;

    s = epoll_ctl( epollFD, EPOLL_CTL_DEL, sfd, NULL );
    if( s == -1 )
    {
        return DP_RESULT_FAILURE;
    }

    return DP_RESULT_SUCCESS;
}

DP_RESULT_T
DaemonProcess::init()
{
    epollFD = epoll_create1( 0 );
    if( epollFD == -1 )
    {
        daemon_log( LOG_ERR, "epoll_create: %s", strerror(errno) );
        return DP_RESULT_FAILURE;
    }

    // Buffer where events are returned 
    events = (struct epoll_event *) calloc( MAXEVENTS, sizeof event );

    demod.setNotify( this ); 

    demod.init();

    // Things initialized correctly.
    healthOK = true;

    return DP_RESULT_SUCCESS;
}

DP_RESULT_T
DaemonProcess::addSignalSocket( int sfd )
{
    signalFD = sfd;
    
    return addSocketToEPoll( signalFD );
}

DP_RESULT_T
DaemonProcess::openListenerSocket()
{
    struct sockaddr_un addr;
    char *str;

    memset(&addr, 0, sizeof(struct sockaddr_un));  // Clear address structure
    addr.sun_family = AF_UNIX;                     // UNIX domain address

    // addr.sun_path[0] has already been set to 0 by memset() 

    // Abstract socket with name @acrt5n1d_readings
    str = "acrt5n1d_readings";
    strncpy( &addr.sun_path[1], str, strlen(str) );

    acceptFD = socket( AF_UNIX, SOCK_SEQPACKET, 0 );
    if( acceptFD == -1 )
    {
        daemon_log( LOG_ERR, "Opening daemon listening socket failed (%s).", strerror(errno) );
        return DP_RESULT_FAILURE;
    }

    if( bind( acceptFD, (struct sockaddr *) &addr, sizeof( sa_family_t ) + strlen( str ) + 1 ) == -1 )
    {
        daemon_log( LOG_ERR, "Failed to bind socket to @acrt5n1d_readings (%s).", strerror(errno) );
        return DP_RESULT_FAILURE;
    }

    if( listen( acceptFD, 4 ) == -1 )
    {
        daemon_log( LOG_ERR, "Failed to listen on socket for @acrt5n1d_readings (%s).", strerror(errno) );
        return DP_RESULT_FAILURE;
    }

    return addSocketToEPoll( acceptFD );
}


DP_RESULT_T
DaemonProcess::processNewClientConnections( )
{
    uint8_t buf[16];

    // There are pending connections on the listening socket.
    while( 1 )
    {
        struct sockaddr in_addr;
        socklen_t in_len;
        int infd;

        in_len = sizeof in_addr;
        infd = accept( acceptFD, &in_addr, &in_len );
        if( infd == -1 )
        {
            if( (errno == EAGAIN) || (errno == EWOULDBLOCK) )
            {
                // All requests processed
                break;
            }
            else
            {
                // Error while accepting
                daemon_log( LOG_ERR, "Failed to accept for @acrt5n1d_readings (%s).", strerror(errno) );
                return DP_RESULT_FAILURE;
            }
        }

        daemon_log( LOG_ERR, "Adding client - sfd: %d", infd );

        ClientRecord client;

        client.setFD( infd );

        clientMap.insert( std::pair< int, ClientRecord >( infd, client ) );

        addSocketToEPoll( infd );
    }

    return DP_RESULT_SUCCESS;
}

                    
DP_RESULT_T
DaemonProcess::closeClientConnection( int clientFD )
{
    clientMap.erase( clientFD );

    removeSocketFromEPoll( clientFD );

    close( clientFD );

    daemon_log( LOG_ERR, "Closed client - sfd: %d", clientFD );

    return DP_RESULT_SUCCESS;
}

DP_RESULT_T
DaemonProcess::processClientRequest( int efd )
{
    // One of the clients has sent us a message.
    HNodeSEPPacket  packet;
    uint32_t        recvd = 0;

    //std::cout << "start recvd..." << std::endl;

    recvd += recv( efd, packet.getPacketPtr(), packet.getMaxPacketLength(), 0 );

    switch( packet.getType() )
    {
        case HNSEPP_TYPE_HNS_PING:
        {
            daemon_log( LOG_ERR, "Ping request from client: %d", efd );
            sendStatus = true;
        }
        break;

        case HNSEPP_TYPE_HNS_RESET:
        {
            daemon_log( LOG_ERR, "Reset request from client: %d", efd );
            sendStatus = true;
        }
        break;

        default:
        {
            daemon_log( LOG_ERR, "RX of unknown packet - len: %d  type: %d", recvd, packet.getType() );
        }
        break;
    }

}

DP_RESULT_T
DaemonProcess::run()
{
    // The event loop
    while( quit == false )
    {
        int n, i;

        // Check for events
        n = epoll_wait( epollFD, events, MAXEVENTS, 0 );

        // EPoll error
        if( n < 0 )
        {
            // If we've been interrupted by an incoming signal, continue, wait for socket indication
            if( errno == EINTR )
                continue;

            // Handle error
        }

        // Timeout
        if( n == 0 )
        {
            // Check if a status update should
            // be sent
            if( sendStatus )
            {
                sendStatusPacket();
                sendStatus = false;
            }

            // Run the RTLSDR loop
            demod.processSample();
            continue;
        }

        // Socket event
        for( i = 0; i < n; i++ )
	    {
            // Dispatch based on file desriptor
            if( signalFD == events[i].data.fd )
            {
                // There was signal activity from libdaemon
	            if( (events[i].events & EPOLLERR) || (events[i].events & EPOLLHUP) || (!(events[i].events & EPOLLIN)) )
	            {
                    // An error has occured on this fd, or the socket is not ready for reading (why were we notified then?) 
	                daemon_log( LOG_ERR, "epoll error on daemon signal socket\n" );
	                close (events[i].data.fd);
	                continue;
	            }

                // Read the signal from the socket
                int sig = daemon_signal_next();
                if( sig <= 0 ) 
                {
                    daemon_log(LOG_ERR, "daemon_signal_next() failed: %s", strerror(errno));
                    break;
                }

                // Act on the signal
                switch (sig)
                {

                    case SIGINT:
                    case SIGQUIT:
                    case SIGTERM:
                    {
                        daemon_log( LOG_WARNING, "Got SIGINT, SIGQUIT or SIGTERM." );
                        quit = true;
                    }
                    break;

                    case SIGHUP:
                    default:
                    {
                        daemon_log( LOG_INFO, "HUP - Ignoring signal" );
                        break;
                    }
                }
            }
            else if( acceptFD == events[i].data.fd )
	        {
                // New client connections
	            if( (events[i].events & EPOLLERR) || (events[i].events & EPOLLHUP) || (!(events[i].events & EPOLLIN)) )
	            {
                    /* An error has occured on this fd, or the socket is not ready for reading (why were we notified then?) */
                    daemon_log( LOG_ERR, "accept socket closed - restarting\n" );
                    close (events[i].data.fd);
	                continue;
	            }

                processNewClientConnections();
                continue;
            }
            else
            {
                // New client connections
	            if( (events[i].events & EPOLLERR) || (events[i].events & EPOLLHUP) || (!(events[i].events & EPOLLIN)) )
	            {
                    // An error has occured on this fd, or the socket is not ready for reading (why were we notified then?)
                    closeClientConnection( events[i].data.fd );

	                continue;
	            }

                // Handle a request from a client.
                processClientRequest( events[i].data.fd );
            }
        }
    }
}

void 
DaemonProcess::sendStatusPacket()
{
    HNodeSEPPacket packet;
    uint32_t length;
    struct timeval curTS;

    gettimeofday( &curTS, NULL );

    packet.setType( HNSEPP_TYPE_HNS_STATUS );

    packet.setSensorIndex( healthOK );

    packet.setParam( 0, curTS.tv_sec );
    packet.setParam( 1, curTS.tv_usec );

    packet.setParam( 2, lastReadingTS.tv_sec );
    packet.setParam( 3, lastReadingTS.tv_usec );
    
    packet.setPayloadLength( curErrMsg.size() );
    memcpy( packet.getPayloadPtr(), curErrMsg.c_str(), curErrMsg.size() );

    for( std::map< int, ClientRecord >::iterator it = clientMap.begin(); it != clientMap.end(); it++ )
    {
        length = send( it->first, packet.getPacketPtr(), packet.getPacketLength(), MSG_NOSIGNAL );         
    }    
}

void 
DaemonProcess::notifyNewMeasurement( uint32_t sensorIndex, HNodeSensorMeasurement &reading )
{
    HNodeSEPPacket packet;
    uint32_t length;

    packet.setType( HNSEPP_TYPE_HNS_MEASUREMENT );

    reading.buildPacketData( packet.getPayloadPtr(), length );

    packet.setPayloadLength( length );
    packet.setSensorIndex( sensorIndex );

    for( std::map< int, ClientRecord >::iterator it = clientMap.begin(); it != clientMap.end(); it++ )
    {
        length = send( it->first, packet.getPacketPtr(), packet.getPacketLength(), MSG_NOSIGNAL );         
    }    
}

void 
DaemonProcess::signalError( std::string errMsg )
{
    healthOK  = false;
    curErrMsg = errMsg;
}

void 
DaemonProcess::signalRunning()
{
    healthOK = true;
    curErrMsg.clear();
}

int 
main( int argc, char *argv[] ) 
{
    pid_t pid;

    // Reset signal handlers
    if( daemon_reset_sigs( -1 ) < 0 )
    {
        daemon_log( LOG_ERR, "Failed to reset all signal handlers: %s", strerror(errno) );
        return 1;
    }

    // Unblock signals
    if( daemon_unblock_sigs( -1 ) < 0 ) 
    {
        daemon_log( LOG_ERR, "Failed to unblock all signals: %s", strerror(errno) );
        return 1;
    }

    // Set indetification string for the daemon for both syslog and PID file
    daemon_pid_file_ident = daemon_log_ident = daemon_ident_from_argv0( argv[0] );

    // Check if we are called with -k parameter
    if( argc >= 2 && !strcmp( argv[1], "-k" ) )
    {
        int ret;

        // Kill daemon with SIGTERM
        // Check if the new function daemon_pid_file_kill_wait() is available, if it is, use it.
        if( ( ret = daemon_pid_file_kill_wait( SIGTERM, 5 ) ) < 0 )
            daemon_log( LOG_WARNING, "Failed to kill daemon: %s", strerror(errno) );

        return ret < 0 ? 1 : 0;
    }

    // Check that the daemon is not rung twice a the same time
    if( (pid = daemon_pid_file_is_running() ) >= 0 ) 
    {
        daemon_log( LOG_ERR, "Daemon already running on PID file %u", pid );
        return 1;
    }

    // Prepare for return value passing from the initialization procedure of the daemon process
    if( daemon_retval_init() < 0 )
    {
        daemon_log( LOG_ERR, "Failed to create pipe." );
        return 1;
    }

    // Do the fork
    if( ( pid = daemon_fork() ) < 0 )
    {
        // Exit on error
        daemon_retval_done();
        return 1;

    } 
    else if( pid )
    {
        // The parent
        int ret;

        // Wait for 20 seconds for the return value passed from the daemon process
        if( (ret = daemon_retval_wait( 20 ) ) < 0 )
        {
            daemon_log( LOG_ERR, "Could not recieve return value from daemon process: %s", strerror(errno) );
            return 255;
        }

        daemon_log( ret != 0 ? LOG_ERR : LOG_INFO, "Daemon returned %i as return value.", ret );
        return ret;

    } 
    else 
    {
        // The daemon
        DaemonProcess dp;

        //int fd, quit = 0;
        //fd_set fds;

        // Close FDs
        if( daemon_close_all(-1) < 0 )
        {
            daemon_log( LOG_ERR, "Failed to close all file descriptors: %s", strerror(errno) );

            // Send the error condition to the parent process
            daemon_retval_send( 1 );
            goto finish;
        }

        // Create the PID file
        if( daemon_pid_file_create() < 0 )
        {
            daemon_log( LOG_ERR, "Could not create PID file (%s).", strerror(errno) );
            daemon_retval_send( 2 );
            goto finish;
        }

        // Initialize signal handling
        if( daemon_signal_init( SIGINT, SIGTERM, SIGQUIT, SIGHUP, 0 ) < 0 )
        {
            daemon_log( LOG_ERR, "Could not register signal handlers (%s).", strerror(errno) );
            daemon_retval_send( 3 );
            goto finish;
        }

        /*... do some further init work here */
        if( dp.init() != DP_RESULT_SUCCESS )
        {
            daemon_log( LOG_ERR, "Failure while initializing." );
        }

        dp.addSignalSocket( daemon_signal_fd() );

        dp.openListenerSocket();

        /* Send OK to parent process */
        daemon_retval_send( 0 );

        daemon_log( LOG_INFO, "Sucessfully started" );

        dp.run();

        // Do cleanup
finish:
        daemon_log( LOG_INFO, "Exiting..." );
        daemon_retval_send( 255 );
        daemon_signal_done();
        daemon_pid_file_remove();

        return 0;
    }
}

