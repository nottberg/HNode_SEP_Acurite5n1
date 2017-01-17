#ifndef __WEATHERRESOURCE_H__
#define __WEATHERRESOURCE_H__

#include <hnode-rest.hpp>
#include "Acurite5N1Manager.hpp"

class WeatherRootResource : public RESTResourceRESTStatusProvider
{
    private:

    public:
        WeatherRootResource( Acurite5N1Manager &mgr );
       ~WeatherRootResource();
};

#endif //__WEATHERRESOURCE_H__
