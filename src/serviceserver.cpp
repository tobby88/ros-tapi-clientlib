#include "include/tapi_lib/serviceserver.hpp"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

ServiceServer::ServiceServer(ros::NodeHandle* nh, string nodename)
  : TapiClient(nh, nodename, PUBLISHER_DEVICE), nh(nh), nodename(nodename)
{
}

ServiceServer::~ServiceServer()
{
  for (int i = 0; i < services.size(); i++)
  {
    services[i].second->shutdown();
    delete services[i].second;
    services[i].second = 0;
  }
}

// Public member functions

ros::ServiceServer* ServiceServer::AddFeature(ros::AdvertiseServiceOptions opt, string featurename)
{
  std::string featureUUID = getNextFeatureUUID();
  ros::ServiceServer* service = 0;
  std::string serviceName = "/Tapi/" + uuid + "/" + featureUUID;
  opt.service = serviceName;

  service = new ros::ServiceServer(nh->advertiseService(opt));

  if (service)
  {
    pair<ros::AdvertiseServiceOptions, ros::ServiceServer*> newEntry;
    newEntry = make_pair(opt, service);
    services.push_back(newEntry);
    tapi_lib::Feature feature;
    string temp = opt.datatype;
    feature.FeatureType = temp;
    feature.Name = featurename;
    feature.UUID = featureUUID;
    featureMsgs.push_back(feature);
  }
  connect();
  return service;
}
}
