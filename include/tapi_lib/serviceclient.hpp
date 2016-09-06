#ifndef SERVICECLIENT_H
#define SERVICECLIENT_H

#include <string>
#include <vector>
#include "ros/node_handle.h"
#include "ros/service_client.h"
#include "ros/service_client_options.h"
#include "ros/subscriber.h"
#include "tapi_client.hpp"
#include "tapi_lib/Connection.h"
#include "tapi_lib/Feature.h"

namespace Tapi
{
class ServiceClient : public TapiClient
{
public:
  // Constructor/Destructor
  ServiceClient(ros::NodeHandle *nh, std::string nodename = "");
  ~ServiceClient();

  // Public member functions
  // Templated functions
  template <typename T>
  ros::ServiceClient **AddFeature(std::string featurename = "")
  {
    ros::ServiceClientOptions opt;
    ros::M_string empty;
    ros::ServiceClient *client = 0;
    std::string featureUUID = getNextFeatureUUID();
    std::string temptopic = "/Tapi/" + uuid + "/" + featureUUID;
    opt.init<T>(temptopic, false, empty);
    ros::ServiceClient **clientptr = 0;
    std::pair<ros::ServiceClientOptions, ros::ServiceClient *> newEntry;
    newEntry = std::make_pair(opt, client);
    clients.push_back(newEntry);
    tapi_lib::Feature feature;
    ros::AdvertiseServiceOptions temp;
    temp = ros::AdvertiseServiceOptions::create<T>("", NULL, ros::VoidPtr(), NULL);
    std::string temp2 = temp.datatype;
    feature.FeatureType = temp2;
    feature.Name = featurename;
    feature.UUID = featureUUID;
    featureMsgs.push_back(feature);
    clientptr = &clients.at(clients.size() - 1).second;
    return clientptr;
  }

private:
  // Private member functions
  void readConfigMsg(const tapi_lib::Connection::ConstPtr &msg);

  // Private member variables
  std::vector<std::pair<ros::ServiceClientOptions, ros::ServiceClient *>> clients;
  ros::Subscriber configSub;
  ros::NodeHandle *nh;
  std::string nodename;
  std::string serviceName;
};
}

#endif  // SERVICECLIENT_H
