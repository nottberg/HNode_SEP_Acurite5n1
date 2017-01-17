#include <stdio.h>
#include <string.h>
#include <arpa/inet.h>

#include "HNodeWeatherMeasurement.hpp"

HNodeWeatherMeasurement::HNodeWeatherMeasurement()
{
    type    = HNWM_TYPE_NOT_SET;
    units   = HNWM_UNITS_NOT_SET;
    count   = 0;
    tstamp.tv_sec  = 0;
    tstamp.tv_usec = 0;
    reading = 0.0;
}

HNodeWeatherMeasurement::~HNodeWeatherMeasurement()
{

}

void 
HNodeWeatherMeasurement::setType( HNWM_TYPE_T value )
{
   type = value;
}

void 
HNodeWeatherMeasurement::setUnits( HNWM_UNITS_T value )
{
    units = value;
}

void 
HNodeWeatherMeasurement::setCount( uint32_t value )
{
    count = value;
}

void 
HNodeWeatherMeasurement::setReading( double value )
{
    reading = value;
}

void 
HNodeWeatherMeasurement::setTimestamp( struct timeval &value )
{
    tstamp.tv_sec = value.tv_sec;
    tstamp.tv_usec = value.tv_usec;
}

void 
HNodeWeatherMeasurement::updateTimestamp()
{
    gettimeofday( &tstamp, NULL );
}

std::string 
HNodeWeatherMeasurement::getAsStr()
{
    char timeBuf[64];
    char tmpStr[512];
    std::string resultStr;

    strftime( timeBuf, sizeof timeBuf, "%Y-%m-%d %H:%M:%S", localtime( &(tstamp.tv_sec) ) );

    sprintf( tmpStr,   "%06d   %s  ", count, timeBuf );
    resultStr = tmpStr;

    switch( type )
    {
        case HNWM_TYPE_WIND_SPEED:
            sprintf( tmpStr,   "    wind speed: %2.1f kph", reading );
            resultStr += tmpStr;
        break;

        case HNWM_TYPE_TEMPERATURE:
            sprintf( tmpStr,   "  outside temp: %2.1f° F", reading );
            resultStr += tmpStr;
        break;

        case HNWM_TYPE_RELATIVE_HUMIDITY:
            sprintf( tmpStr,   "      humidity: %2.1f%% RH", reading );
            resultStr += tmpStr;
        break;

        case HNWM_TYPE_WIND_DIRECTION:
            sprintf( tmpStr,   "wind direction: %0.1f°", reading );
            resultStr += tmpStr;
        break;

        case HNWM_TYPE_RAINFALL:
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
HNodeWeatherMeasurement::buildPacketData( uint8_t *bufPtr, uint32_t &length )
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
HNodeWeatherMeasurement::parsePacketData( uint8_t *bufPtr, uint32_t length )
{
    uint32_t *dPtr = ( uint32_t* ) bufPtr;
    char strBuf[128];
    int  strLen;

    type  = (HNWM_TYPE_T) ntohl( *dPtr );
    dPtr += 1;

    units  = (HNWM_UNITS_T) ntohl( *dPtr );
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


