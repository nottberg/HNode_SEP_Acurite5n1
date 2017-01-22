#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <stdio.h> 
#include <glib.h>
#include <stdint.h>
#include <sys/socket.h>
#include <sys/un.h>

#include <iostream>
#include <cstddef>

#include <avahi-client/client.h>
#include <avahi-client/lookup.h>
#include <avahi-client/publish.h>

#include <avahi-common/alternative.h>
#include <avahi-common/error.h>
#include <avahi-common/timeval.h>

#include <avahi-glib/glib-watch.h>
#include <avahi-glib/glib-malloc.h>

#include <hnode-1.0/hnode-pktsrc.h>
#include <hnode-1.0/hnode-nodeobj.h>

#include <hnode-rest.hpp>

#include "HNodeWeatherEPPacket.hpp"
#include "HNodeWeatherMeasurement.hpp"

#include "Acurite5N1Manager.hpp"
#include "WeatherResource.hpp"

//guint8 gUID[16] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0xfe, 0xff};
guint8 gWeatherSignature[4] = {0x10, 0x26, 0x26, 0x10};

static gint wait_time = 0;
static gint instance_id = 0;
static gint debug_period = 10000;
static gint event_period = 10000;
static gint up_period = 4000;
static gint down_period = 10000;
static gboolean debug = FALSE;
static gboolean event = FALSE;
static gboolean updown = FALSE;


typedef struct WeatherNodeContext
{
    GHNode    *HNode;

    bool      hasInstanceID;
    guint32   instanceID;

    int       epfd;
    bool      epConnected;

    RESTDaemon   Rest;

    Acurite5N1Manager   wxManager;

    WeatherRootResource     wxResource;

    WeatherNodeContext() : wxResource( this->wxManager ) {}

}CONTEXT;

//static struct termios saved_io;

bool
hnode_load_configuration(CONTEXT *Context)
{
    // Load the configuration for the switches
    Context->wxManager.loadConfiguration();

    return false;
}

bool
hnode_start_hardware_interface(CONTEXT *Context)
{
    // Start everything up
    Context->wxManager.start();
    
    return false;
}

bool
hnode_start_rest_daemon(CONTEXT *Context)
{
    // Init the REST resources.
    Context->Rest.registerResource( &(Context->wxResource) );

    Context->Rest.setListeningPort( 8888 );
    Context->Rest.start();

    return false;
}

gboolean
hnode_process_ep_packet( GIOChannel *source, GIOCondition condition, gpointer data )
{
    CONTEXT *Context = (CONTEXT *) data;

    HNodeWeatherEPPacket    packet;
    HNodeWeatherMeasurement reading;
    uint32_t recvd = 0;

    //std::cout << "start recvd..." << std::endl;

    recvd += recv( Context->epfd, packet.getPacketPtr(), packet.getMaxPacketLength(), 0 );

    switch( packet.getType() )
    {
        case HNWEPP_TYPE_HNW_MEASUREMENT:
        {
            reading.parsePacketData( packet.getPayloadPtr(), recvd );

            std::cout << reading.getAsStr() << std::endl;
                    //std::cout << "recvd: " << recvd << std::endl;

            Context->wxManager.addNewMeasurement( reading );
        }
        break;

        default:
        {
            std::cout << "Unknown Packet Type - len: " << recvd << "  type: " << packet.getType() << std::endl;
        }
        break;
    }
}

gboolean
hnode_heartbeat( CONTEXT *Context )
{
    //printf("hnode_heartbeat - start: 0x%x\n", Context);
     
    // Get the current time
    //ScheduleDateTime curTime;
    //curTime.getCurrentTime();

    // If we are not connected to the acrt4335n1 daemon
    // then attempt to establish that connection here.
    if( Context->epConnected == false )
    {
        struct sockaddr_un addr;
        char *str;

        memset(&addr, 0, sizeof(struct sockaddr_un));  // Clear address structure
        addr.sun_family = AF_UNIX;                     // UNIX domain address

        // addr.sun_path[0] has already been set to 0 by memset() 

        // Abstract socket with name @acrt5n1d_readings
        str = "acrt5n1d_readings";
        strncpy( &addr.sun_path[1], str, strlen(str) );

        Context->epfd = socket( AF_UNIX, SOCK_SEQPACKET, 0 );

        if( connect( Context->epfd, (struct sockaddr *) &addr, sizeof( sa_family_t ) + strlen( str ) + 1 ) != 0 )
        {
            // Failed to connect
            close( Context->epfd );
            Context->epConnected = false;
        }
        else
        {
            // Connection established
            Context->epConnected = true;

            // add us to the event loop
            GIOChannel *channel = g_io_channel_unix_new( Context->epfd );

            // Pass connection as user_data to the watch callback
            g_io_add_watch( channel, G_IO_IN, (GIOFunc) hnode_process_ep_packet, Context );
        }
    }

    // Process any pertinent events
    uint32_t time = 0;
    Context->wxManager.processCurrentEvents( time );

    // Wait for the next timeout
    return TRUE;
}
void

hnode_get_unique_uid( CONTEXT *Context )
{
    char ethStr[64];
    FILE *pFile;
    guint8 uid[16];

    // Init to something other than zeros
    for( int idx = 0; idx < sizeof(uid); idx++ )
    {
        uid[idx] = idx;
    }

    // Try to get the first network interface address as unique
    pFile = fopen( "/sys/class/net/eth0/address", "r" );

    if( pFile != NULL )
    {
        fscanf(pFile, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", &uid[0], &uid[1], &uid[2], &uid[3], &uid[4], &uid[5]);
        fclose( pFile );
    }

    // Add a magic number for this type of hnode
    for( int idx = 0; idx < 4; idx++ )
    {
        uid[7+idx] = gWeatherSignature[idx];
    }

    // Handle any instance ID customization
    if( Context->hasInstanceID )
    {
        uid[12] = (Context->instanceID >> 24) & 0xFF;
        uid[13] = (Context->instanceID >> 16) & 0xFF;
        uid[14] = (Context->instanceID >>  8) & 0xFF;
        uid[15] = Context->instanceID & 0xFF;
    }

    // Debug dump
    for( int idx = 0; idx < sizeof(uid); idx++ )
    {
        printf( "%2.2x ", uid[idx] );
    }
    printf("\n");

   
    // Set the unique id
    g_hnode_set_uid(Context->HNode, uid);

}

static GOptionEntry entries[] = 
{
  { "print", 0, 0, G_OPTION_ARG_INT, &event_period, "Turn on printing of rx and tx to standard out", "N" },
  { "instance-id", 'i', 0, G_OPTION_ARG_INT, &instance_id, "If running more than one instance on a node give each a unique ID", "N" },
  { NULL }
};

int
main (AVAHI_GCC_UNUSED int argc, AVAHI_GCC_UNUSED char *argv[])
{
    GMainLoop *loop = NULL;
    CONTEXT   Context;
    GOptionContext *cmdline_context;
    GError *error = NULL;
    GHNodeAddress *AddrObj;

    // Parse any command line options.
    cmdline_context = g_option_context_new ("- a hnode for Accurite5n1 weather station.");
    g_option_context_add_main_entries (cmdline_context, entries, NULL); // GETTEXT_PACKAGE);
    g_option_context_parse (cmdline_context, &argc, &argv, &error);

    // Init the instance id, if present.
    Context.hasInstanceID = false;
    if( instance_id )
    {
        Context.hasInstanceID = true;
        Context.instanceID    = instance_id;
    }

    // Create the GLIB main loop 
    loop = g_main_loop_new (NULL, FALSE);

    // Allocate a server object
    Context.HNode = g_hnode_new();

    if( Context.HNode == NULL )
        exit(-1);

    // HNode intialization

    // Setup the HNode
    g_hnode_set_version(Context.HNode, 1, 0, 0);
    hnode_get_unique_uid( &Context );
    g_hnode_set_name_prefix(Context.HNode, (guint8*)"WeatherStation");

    g_hnode_set_endpoint_count(Context.HNode, 1);
    
    //guint16 EndPointIndex, guint16 AssociatedEPIndex, guint8 *MimeTypeStr, guint16 Port, guint8 MajorVersion, guint8 MinorVersion, guint16 MicroVersion)
    g_hnode_set_endpoint(Context.HNode, 0, 0, (guint8*)"hnode-weather-rest", REST_DAEMON_DEFAULT_PORT, 1, 0, 0);	

    hnode_load_configuration( &Context );

    hnode_start_hardware_interface( &Context );

    // Start up the manager object
    Context.wxManager.start(); 

    // Fire up the rest daemon
    hnode_start_rest_daemon( &Context );

    // Start up the server object
    g_hnode_start(Context.HNode);

    // Setup the periodic timer for handling timed events
    g_timeout_add_seconds( 1, (GSourceFunc) hnode_heartbeat, &Context );

    // Start the GLIB Main Loop 
    g_main_loop_run (loop);

    fail:
    // Clean up 
    g_main_loop_unref (loop);

    return 0;
}
