#include <string>
#include <iostream>
#include <fstream>
#include <cerrno>

#include <stdio.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>

#include <string.h>
#include <arpa/inet.h>

#include <boost/regex.hpp>
#include <boost/algorithm/string/trim.hpp>

#include <hnode-rest.hpp>

#include "Acurite5N1Manager.hpp"

WeatherManagerMeasurements::WeatherManagerMeasurements()
{

}

WeatherManagerMeasurements::~WeatherManagerMeasurements()
{

}

void 
WeatherManagerMeasurements::addNewMeasurement( HNodeWeatherMeasurement &measurement )
{
    history.push_front( measurement );

    if( history.size() > 2880 )
    {
        history.pop_back();
    }
}

bool 
WeatherManagerMeasurements::getCurrentMeasurement( HNodeWeatherMeasurement &measurement )
{
    if( history.size() == 0 )
        return false;

    measurement = history.front();

    return true;
}

Acurite5N1Manager::Acurite5N1Manager()
{
    //cfgPath         = "/etc/hnode";  
}

Acurite5N1Manager::~Acurite5N1Manager()
{

}


bool
Acurite5N1Manager::loadConfiguration()
{
#if 0
    std::string filePath;
    ScheduleConfig cfgReader;

    filePath = cfgPath + "/irrigation/schedule_config.xml";

    // Clear out any existing items.
    clear();

    // Load the local timezone information
    localtz.initFromSystemFiles();

    // Read everything from a file
    cfgReader.readConfig( filePath, this );

    eventLog.addLogEntry( "sch-load-config", "The configuration file was loaded." );
#endif
    return false;
}

bool
Acurite5N1Manager::saveConfiguration()
{
#if 0
    ScheduleConfig cfgWriter;
    std::string filePath;

    filePath = cfgPath + "/irrigation/schedule_config.xml";

    // Write everything out to a file
    cfgWriter.writeConfig( filePath, this );

    eventLog.addLogEntry( "sch-save-config", "The configuration file was saved." );
#endif
    return false;
}

void 
Acurite5N1Manager::processCurrentEvents( uint32_t &curTime )
{
    //printf( "processCurrentEvents: %s\n", curTime.getSimpleString().c_str() );
    //printf( "processCurrentEvents: %s\n", curTime.getISOString().c_str() );
    //printf( "hour: %d\n", curTime.getHour() );
    //printf( "minutes: %d\n", curTime.getMinute() );
    //printf( "seconds: %d\n", curTime.getSecond() );

}

void 
Acurite5N1Manager::populateContentNodeFromStatusProvider( unsigned int id, RESTContentNode *outNode, std::map< std::string, std::string > paramMap )
{

    switch( id )
    {
        case WXRSRC_STATID_CURRENT_READING:
        {
            // Give the root element a tag name
            outNode->setAsObject( "current-measurements" );

#if 0
            // Give the currect time as we see it
            ScheduleDateTime timestamp;
            timestamp.getCurrentTime();
            outNode->setField( "timestamp", timestamp.getISOString() );

            // Give the controllers timezone
            std::string tmpStr = localtz.getOlsenStr();
            outNode->setField( "timezone", tmpStr );

            // Give the controllers timezone
            tmpStr = localtz.getPosixStr();
            outNode->setField( "posix-timezone", tmpStr );

            // Master enable state
            outNode->setField( "master-enable", masterEnable ? "true" : "false" );
#endif

            // Add a list for any active zones
            RESTContentNode *mList = RESTContentHelperFactory::newContentNode();

            mList->setAsArray( "measurement-list" );
            outNode->addChild( mList );

            for( std::map< HNWM_TYPE_T, WeatherManagerMeasurements >::iterator it = measurements.begin(); it != measurements.end(); it++ )
            {
                RESTContentNode         *curNode;
                HNodeWeatherMeasurement  curM;
                struct timeval           tstamp;
                char                     tmpStr[128];
              
                if( it->second.getCurrentMeasurement( curM ) == false )
                {
                    continue;
                }

                curNode = RESTContentHelperFactory::newContentNode();

                curNode->setAsObject( "measurement" );

                curNode->setField( "type", curM.getTypeAsStr() );
                curNode->setField( "units", curM.getUnitsAsStr() );

                sprintf( tmpStr, "%d", curM.getCount() ); 
                curNode->setField( "epcount", tmpStr );

                sprintf( tmpStr, "%lf", curM.getReading() ); 
                curNode->setField( "reading", tmpStr );

                curM.getTimestamp( tstamp );

                sprintf( tmpStr, "%lu", tstamp.tv_sec ); 
                curNode->setField( "tstamp_sec", tmpStr );

                sprintf( tmpStr, "%lu", tstamp.tv_usec ); 
                curNode->setField( "tstamp_usec", tmpStr );

                mList->addChild( curNode );
            }

#if 0
            // Add todays scheduled events
            RESTContentNode *dsList = RESTContentHelperFactory::newContentNode();

            dsList->setAsArray( "todays-schedule" );
            outNode->addChild( dsList );

            ScheduleLocalDateTime startTime( timestamp );
            startTime.retreatToStartOfDay();

            ScheduleLocalDateTime endTime( timestamp );
            endTime.advanceToEndOfDay();

            eventList = getPotentialEventsForPeriod( startTime.getUTCTime(), endTime.getUTCTime() );

            // Do processing for active rules
            for( unsigned int index = 0; index < eventList->getEventCount(); ++index )
            {
                ScheduleEvent *event = eventList->getEvent( index );

                ScheduleDateTime evStart;
                ScheduleDateTime evEnd;

                event->getStartTime( evStart );
                event->getEndTime( evEnd );

                std::cout << "Event Entry -- ID: " << event->getId() << " Title: " << event->getDescription() << " Start: " << evStart.getISOString() << " End: " << evEnd.getISOString() << std::endl;

                RESTContentNode *curNode = RESTContentHelperFactory::newContentNode();

                curNode->setAsObject( "event" );

                curNode->setField( "id", event->getId() );
                curNode->setField( "start-time", evStart.getISOString() );
                curNode->setField( "end-time", evEnd.getISOString() );
                curNode->setField( "zone-name", event->getZoneRecord().getZoneName() );
                curNode->setField( "trigger-name", event->getTriggerRecord().getTriggerName() );
                curNode->setField( "duration", event->getDurationStr() );  
                curNode->setField( "erID", event->getTriggerRecord().getERID() );
                curNode->setField( "zgID", event->getZoneRecord().getGroupID() );
                curNode->setField( "zrID", event->getZoneRecord().getRuleID() );
                curNode->setField( "tgID", event->getTriggerRecord().getGroupID() );
                curNode->setField( "trID", event->getTriggerRecord().getRuleID() );
          
                dsList->addChild( curNode );
            } 

            freeScheduleEventList( eventList );
    

            // Add todays log events
            RESTContentNode *evList = RESTContentHelperFactory::newContentNode();

            outNode->addChild( evList );

            eventLog.populateTodaysEventsNode( evList, timestamp );
#endif
        }
        break;
#if 0
        case SCHRSRC_STATID_EVENTLOG:
        {
            eventLog.populateContentNode( outNode );
        }
        break;

        case SCHRSRC_STATID_CALENDAR:
        {
            ScheduleEventList *eventList;
            ScheduleDateTime   startTime;
            ScheduleDateTime   endTime;

            std::map< std::string, std::string >::iterator it = paramMap.find( "startTime" );
 
            if( it == paramMap.end() )
            { 
                startTime.getCurrentTime();
            }
            else
            {
                startTime.setTimeFromISOString( it->second );
            }

            it = paramMap.find( "endTime" );
 
            if( it == paramMap.end() )
            { 
                endTime.setTime( startTime );
                endTime.addHours( 2 );
            }
            else
            {
                endTime.setTimeFromISOString( it->second );
            }

            eventList = getPotentialEventsForPeriod( startTime, endTime );

            // Give the root element a tag name
            outNode->setAsObject( "schedule-event-calendar" );

            outNode->setField( "period-start", startTime.getISOString() );
            outNode->setField( "period-end", endTime.getISOString() );

            RESTContentNode *evList = RESTContentHelperFactory::newContentNode();

            evList->setAsArray( "event-list" );
            outNode->addChild( evList );

            // Do processing for active rules
            for( unsigned int index = 0; index < eventList->getEventCount(); ++index )
            {
                ScheduleEvent *event = eventList->getEvent( index );

                ScheduleDateTime evStart;
                ScheduleDateTime evEnd;

                event->getStartTime( evStart );
                event->getEndTime( evEnd );

                std::cout << "Event Entry -- ID: " << event->getId() << " Title: " << event->getDescription() << " Start: " << evStart.getISOString() << " End: " << evEnd.getISOString() << std::endl;

                RESTContentNode *curNode = RESTContentHelperFactory::newContentNode();

                curNode->setAsObject( "event" );

                curNode->setField( "id", event->getId() );
                curNode->setField( "start-time", evStart.getISOString() );
                curNode->setField( "end-time", evEnd.getISOString() );
                curNode->setField( "zone-name", event->getZoneRecord().getZoneName() );
                curNode->setField( "trigger-name", event->getTriggerRecord().getTriggerName() );
                curNode->setField( "duration", event->getDurationStr() );  
                curNode->setField( "erID", event->getTriggerRecord().getERID() );
                curNode->setField( "zgID", event->getZoneRecord().getGroupID() );
                curNode->setField( "zrID", event->getZoneRecord().getRuleID() );
                curNode->setField( "tgID", event->getTriggerRecord().getGroupID() );
                curNode->setField( "trID", event->getTriggerRecord().getRuleID() );
          
                evList->addChild( curNode );
            } 

            freeScheduleEventList( eventList );

        }
        break;
#endif

        default:
            std::cerr << "ERROR: Undefined status provider" << std::endl;
            return;
        break;
    }

}

void
Acurite5N1Manager::start()
{
    //demod = new RTL433Demodulator;
    //demod->start();
}

void 
Acurite5N1Manager::addNewMeasurement( HNodeWeatherMeasurement &measurement )
{
    std::map< HNWM_TYPE_T, WeatherManagerMeasurements >::iterator it;

    it = measurements.find( measurement.getType() );

    if( it == measurements.end() )
    {
        WeatherManagerMeasurements mtmp;

        measurements.insert( std::pair< HNWM_TYPE_T, WeatherManagerMeasurements >( measurement.getType(), mtmp ) );

        it = measurements.find( measurement.getType() );
    }
    
    it->second.addNewMeasurement( measurement );
}

RESTContentNode* 
Acurite5N1Manager::newObject( unsigned int type )
{
    return NULL;
}

void 
Acurite5N1Manager::freeObject( RESTContentNode *objPtr )
{

}

unsigned int 
Acurite5N1Manager::getTypeFromObjectElementName( std::string name )
{
    return 0;
}

RESTContentTemplate *
Acurite5N1Manager::getContentTemplateForType( unsigned int type )
{
    return NULL;
}

