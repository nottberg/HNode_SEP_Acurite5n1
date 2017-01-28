#ifndef __HNODE_SENSOR_MEASUREMENT_H__
#define __HNODE_SENSOR_MEASUREMENT_H__

#include <stdint.h>
#include <time.h>
#include <sys/time.h>

#include <string>

typedef enum HNodeSensorMeasurementType
{
    HNSM_TYPE_NOT_SET            = 0,
    HNSM_TYPE_WIND_SPEED         = 1,
    HNSM_TYPE_TEMPERATURE        = 2,
    HNSM_TYPE_RELATIVE_HUMIDITY  = 3,
    HNSM_TYPE_WIND_DIRECTION     = 4,
    HNSM_TYPE_RAINFALL           = 5
}HNSM_TYPE_T;

typedef enum HNodeSensorMeasurementUnits
{
    HNSM_UNITS_NOT_SET           = 0,
    HNSM_UNITS_KPH               = 1,
    HNSM_UNITS_MPH               = 2,
    HNSM_UNITS_CELSIUS           = 3,
    HNSM_UNITS_FAHRENHEIT        = 4,
    HNSM_UNITS_PERCENT           = 5,
    HNSM_UNITS_DEGREES           = 6,
    HNSM_UNITS_INCHES            = 7
}HNSM_UNITS_T;

class HNodeSensorMeasurement
{
    private:
        HNSM_TYPE_T     type;  
        HNSM_UNITS_T    units;
        uint32_t        count;
        struct timeval  tstamp;
        double          reading;

    public:
        HNodeSensorMeasurement();
       ~HNodeSensorMeasurement();

        void setType( HNSM_TYPE_T value );
        HNSM_TYPE_T getType();
        void setTypeFromStr( std::string value );
        std::string getTypeAsStr();
  
        void setUnits( HNSM_UNITS_T value );
        HNSM_UNITS_T getUnits();
        void setUnitsFromStr( std::string value );
        std::string getUnitsAsStr();

        void setCount( uint32_t value );
        uint32_t getCount();

        void setReading( double value );
        double getReading();

        void setTimestamp( struct timeval &value );
        void getTimestamp( struct timeval &value );
        void updateTimestamp();

        std::string getAsStr();

        void buildPacketData( uint8_t *bufPtr, uint32_t &length );
        void parsePacketData( uint8_t *bufPtr, uint32_t length );
};

#endif // __HNODE_SENSOR_MEASUREMENT_H__
