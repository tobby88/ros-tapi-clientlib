#ifndef SERVICESERVER_H
#define SERVICESERVER_H

#include <string>
#include <vector>
#include "ros/advertise_service_options.h"
#include "ros/node_handle.h"
#include "ros/service_server.h"
#include "tapi_client.hpp"

#define ServiceServerOptionsForTapi(type, callfctn)                                                                    \
  ros::AdvertiseServiceOptions::create<type>("", boost::bind((callfctn), this, _1, _2), ros::VoidPtr(this), NULL)

namespace Tapi
{
class ServiceServer : public TapiClient
{
public:
  // Constructor/Destructor
  ServiceServer(ros::NodeHandle *nh, std::string nodename = "");
  ~ServiceServer();

  // Public member functions
  ros::ServiceServer *AddFeature(ros::AdvertiseServiceOptions opt, std::string featurename = "");

private:
  // Private member variables
  ros::NodeHandle *nh;
  std::string nodename;
  std::vector<std::pair<ros::AdvertiseServiceOptions, ros::ServiceServer *>> services;
};
}

#endif  // SERVICESERVER_H
