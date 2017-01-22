#include <string>
#include <iostream>
#include <fstream>
#include <cerrno>

#include <hnode-rest.hpp>
#include "Acurite5N1Manager.hpp"
#include "WeatherResource.hpp"

WeatherRootResource::WeatherRootResource( Acurite5N1Manager &mgr )
:RESTResourceRESTStatusProvider( "/measurements/current", mgr, WXRSRC_STATID_CURRENT_READING )
{

}

WeatherRootResource::~WeatherRootResource()
{

}


