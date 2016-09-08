#include "include/tapi_lib/serviceclient.hpp"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

ServiceClient::ServiceClient(ros::NodeHandle* nh, string nodename)
  : TapiClient(nh, nodename, SERVICE_CLIENT), nh(nh), nodename(nodename)
{
  configSub = nh->subscribe("/Tapi/Config", 1000, &ServiceClient::readConfigMsg, this);
}

ServiceClient::~ServiceClient()
{
  configSub.shutdown();
  for (int i = 0; i < clients.size(); i++)
  {
    clients[i].second->shutdown();
    delete clients[i].second;
    clients[i].second = 0;
  }
}

// Private member functions
void ServiceClient::readConfigMsg(const tapi_lib::Connection::ConstPtr& msg)
{
  for (int i = 0; i < featureMsgs.size(); i++)
  {
    if (msg->SubscriberUUID == uuid && msg->SubscriberFeatureUUID == featureMsgs[i].UUID)
    {
      if (msg->PublisherUUID == "0" || msg->PublisherFeatureUUID == "0")
        if (clients[i].second)
        {
          clients[i].second->shutdown();
          delete clients[i].second;
          clients[i].second = 0;
        }
        else
          ;
      else
      {
        if (serviceName != "/Tapi/" + msg->PublisherUUID + "/" + msg->PublisherFeatureUUID || clients[i].second == 0)
        {
          serviceName = "/Tapi/" + msg->PublisherUUID + "/" + msg->PublisherFeatureUUID;
          clients[i].first.service = serviceName;
          if (clients[i].second)
          {
            clients[i].second->shutdown();
            delete clients[i].second;
            clients[i].second = 0;
          }
          clients[i].second = new ros::ServiceClient(nh->serviceClient(clients[i].first));
        }
      }
    }
  }
}
}
