#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>

#include "HNodeSensorMeasurement.hpp"

const char* HNodeWMTypeStrings[] = 
{
    "notset",           // HNSM_TYPE_NOT_SET            
    "windspeed",        // HNSM_TYPE_WIND_SPEED         
    "temperature",      // HNSM_TYPE_TEMPERATURE        
    "relativehumidity", // HNSM_TYPE_RELATIVE_HUMIDITY  
    "winddirection",    // HNSM_TYPE_WIND_DIRECTION     
    "rainfall"          // HNSM_TYPE_RAINFALL           
};

const char* HNodeWMUnitsStrings[] = 
{
    "notset",    // HNSM_UNITS_NOT_SET
    "kph",       // HNSM_UNITS_KPH    
    "mph",       // HNSM_UNITS_MPH    
    "celsius",   // HNSM_UNITS_CELSIUS
    "farenheit", // HNSM_UNITS_FAHRENHEIT 
    "percent",   // HNSM_UNITS_PERCENT    
    "degrees",   // HNSM_UNITS_DEGREES    
    "inches"     // HNSM_UNITS_INCHES     
};

HNodeSensorMeasurement::HNodeSensorMeasurement()
{
    type    = HNSM_TYPE_NOT_SET;
    units   = HNSM_UNITS_NOT_SET;
    count   = 0;
    tstamp.tv_sec  = 0;
    tstamp.tv_usec = 0;
    reading = 0.0;
}

HNodeSensorMeasurement::~HNodeSensorMeasurement()
{

}

void 
HNodeSensorMeasurement::setType( HNSM_TYPE_T value )
{
   type = value;
}

HNSM_TYPE_T 
HNodeSensorMeasurement::getType()
{
    return type;
}

void 
HNodeSensorMeasurement::setTypeFromStr( std::string value )
{
    type = HNSM_TYPE_NOT_SET;

    for( int i = 0; i < ( sizeof( HNodeWMTypeStrings )/sizeof( char* ) ); i++ )
    {
        if( value == HNodeWMTypeStrings[i] )
        {
            type = (HNSM_TYPE_T) i;
            return;
        }
    }
}

std::string 
HNodeSensorMeasurement::getTypeAsStr()
{
    return HNodeWMTypeStrings[ type ];
}

void 
HNodeSensorMeasurement::setUnits( HNSM_UNITS_T value )
{
    units = value;
}

HNSM_UNITS_T 
HNodeSensorMeasurement::getUnits()
{
    return units;
}

void 
HNodeSensorMeasurement::setUnitsFromStr( std::string value )
{
    units = HNSM_UNITS_NOT_SET;

    for( int i = 0; i < ( sizeof( HNodeWMUnitsStrings )/sizeof( char* ) ); i++ )
    {
        if( value == HNodeWMUnitsStrings[i] )
        {
            units = (HNSM_UNITS_T) i;
            return;
        }
    }
}

std::string 
HNodeSensorMeasurement::getUnitsAsStr()
{
    return HNodeWMUnitsStrings[ units ];
}

void 
HNodeSensorMeasurement::setCount( uint32_t value )
{
    count = value;
}

void 
HNodeSensorMeasurement::setReading( double value )
{
    reading = value;
}

void 
HNodeSensorMeasurement::setTimestamp( struct timeval &value )
{
    tstamp.tv_sec = value.tv_sec;
    tstamp.tv_usec = value.tv_usec;
}

void 
HNodeSensorMeasurement::updateTimestamp()
{
    gettimeofday( &tstamp, NULL );
}




uint32_t 
HNodeSensorMeasurement::getCount()
{
    return count;
}

double 
HNodeSensorMeasurement::getReading()
{
    return reading;
}

void 
HNodeSensorMeasurement::getTimestamp( struct timeval &value )
{
    value.tv_sec  = tstamp.tv_sec;
    value.tv_usec = tstamp.tv_usec;
}

std::string 
HNodeSensorMeasurement::getAsStr()
{
    char timeBuf[64];
    char tmpStr[512];
    std::string resultStr;

    strftime( timeBuf, sizeof timeBuf, "%Y-%m-%d %H:%M:%S", localtime( &(tstamp.tv_sec) ) );

    sprintf( tmpStr,   "%06d   %s  ", count, timeBuf );
    resultStr = tmpStr;

    switch( type )
    {
        case HNSM_TYPE_WIND_SPEED:
            sprintf( tmpStr,   "    wind speed: %2.1f kph", reading );
            resultStr += tmpStr;
        break;

        case HNSM_TYPE_TEMPERATURE:
            sprintf( tmpStr,   "  outside temp: %2.1f° F", reading );
            resultStr += tmpStr;
        break;

        case HNSM_TYPE_RELATIVE_HUMIDITY:
            sprintf( tmpStr,   "      humidity: %2.1f%% RH", reading );
            resultStr += tmpStr;
        break;

        case HNSM_TYPE_WIND_DIRECTION:
            sprintf( tmpStr,   "wind direction: %0.1f°", reading );
            resultStr += tmpStr;
        break;

        case HNSM_TYPE_RAINFALL:
            sprintf( tmpStr,   "    rain gauge: %0.2f in.", reading );
            resultStr += tmpStr;
        break;

        default:
            sprintf( tmpStr,   "   measurement: %2.1f", reading );
            resultStr += tmpStr;
        break;
    }

    return resultStr;
}

void 
HNodeSensorMeasurement::buildPacketData( uint8_t *bufPtr, uint32_t &length )
{
    uint32_t *dPtr = ( uint32_t* ) bufPtr;
    char strBuf[128];
    int  strLen;

    *dPtr  = htonl( type );
    dPtr += 1;

    *dPtr  = htonl( units );
    dPtr += 1;

    *dPtr  = htonl( count );
    dPtr += 1;

    *dPtr  = htonl( tstamp.tv_sec );
    dPtr += 1;

    *dPtr  = htonl( tstamp.tv_usec );
    dPtr += 1;

    // Format the double value as a string instead
    strLen = sprintf( strBuf, "%lf", reading );

    *dPtr  = htonl( strLen );
    dPtr += 1;

    memcpy( (uint8_t*)dPtr, strBuf, strLen );

    dPtr = (uint32_t*)(((uint8_t *)dPtr) + strLen);

    length = ( ( ( uint8_t* )dPtr ) - bufPtr);
}

void 
HNodeSensorMeasurement::parsePacketData( uint8_t *bufPtr, uint32_t length )
{
    uint32_t *dPtr = ( uint32_t* ) bufPtr;
    char strBuf[128];
    int  strLen;

    type  = (HNSM_TYPE_T) ntohl( *dPtr );
    dPtr += 1;

    units  = (HNSM_UNITS_T) ntohl( *dPtr );
    dPtr += 1;

    count  = ntohl( *dPtr );
    dPtr += 1;

    tstamp.tv_sec = ntohl( *dPtr );
    dPtr += 1;

    tstamp.tv_usec = ntohl( *dPtr );
    dPtr += 1;

    // Format the double value as a string instead
    strLen = ntohl( *dPtr );
    dPtr += 1;

    if( strLen > (sizeof strBuf) )
        return;

    memcpy( strBuf, (uint8_t *) dPtr, strLen );
    strBuf[ strLen ] = '\0';
    
    sscanf( strBuf, "%lf", &reading );
}


