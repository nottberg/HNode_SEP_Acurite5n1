#ifndef __ACURITE5N1_MANAGER_H__
#define __ACURITE5N1_MANAGER_H__

#include <string>
#include <list>
#include <vector>
#include <map>
#include <exception>

#include "boost/date_time/gregorian/gregorian.hpp"
#include "boost/date_time/posix_time/posix_time.hpp" 
#include "boost/date_time/local_time/local_time.hpp"

#include <libxml/encoding.h>
#include <libxml/xmlwriter.h>

#include <hnode-rest.hpp>

//#include "Acurite5N1RTL433.hpp"

class Acurite5N1Exception : public std::exception
{
    private:
        unsigned long eCode;
        std::string eMsg;

    public:
        Acurite5N1Exception( unsigned long errCode, std::string errMsg )
        {
            eCode = errCode;
            eMsg  = errMsg;
        }

       ~Acurite5N1Exception() throw() {};

        virtual const char* what() const throw()
        {
            return eMsg.c_str();
        }

        unsigned long getErrorCode() const throw()
        {
            return eCode;
        }

        std::string getErrorMsg() const throw()
        {
            return eMsg;
        }
};

typedef enum Acurite5N1StatusRequestIDs
{
    WXRSRC_STATID_CURRENT_READING = 1
}WXRSRC_STATID_T;

class Acurite5N1Manager : public RESTContentManager
{
    private:
        //RTL433Demodulator *demod;

        virtual void populateContentNodeFromStatusProvider( unsigned int id, RESTContentNode *outNode, std::map< std::string, std::string > paramMap );

    public:
        Acurite5N1Manager();
       ~Acurite5N1Manager();

        void setConfigurationPath( std::string cfgPath );
        bool loadConfiguration();
        bool saveConfiguration();

        void start();

        void processCurrentEvents( uint32_t &curTime );

        virtual RESTContentNode* newObject( unsigned int type );
        virtual void freeObject( RESTContentNode *objPtr );
        virtual unsigned int getTypeFromObjectElementName( std::string name );
        virtual RESTContentTemplate *getContentTemplateForType( unsigned int type );
};

#endif // __ACURITE5N1_MANAGER_H__

