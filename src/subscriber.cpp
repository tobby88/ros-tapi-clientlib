#include "include/tapi_clientlib/subscriber.hpp"
#include "tapi_msgs/Feature.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

Subscriber::Subscriber(ros::NodeHandle* nh, string nodename)
  : TapiClient(nh, nodename, SUBSCRIBER_DEVICE), nh(nh), nodename(nodename)
{
  subscribers.clear();
  configSub = nh->subscribe("Tapi/Config", 1000, &Subscriber::readConfigMsg, this);
}

Subscriber::~Subscriber()
{
  for (int i = 0; i < subscribers.size(); i++)
  {
    subscribers[i].second->shutdown();
    delete subscribers[i].second;
    subscribers[i].second = 0;
    delete coefficients[i];
  }
}

// Public member functions

double* Subscriber::AddFeature(ros::SubscribeOptions opt, string featurename)
{
  double* dblptr = 0;
  ros::Subscriber* subscriber = 0;
  pair<ros::SubscribeOptions, ros::Subscriber*> newEntry;
  newEntry = make_pair(opt, subscriber);
  subscribers.push_back(newEntry);
  tapi_msgs::Feature feature;
  string temp = opt.datatype;
  feature.FeatureType = temp;
  feature.Name = featurename;
  feature.UUID = getNextFeatureUUID();
  featureMsgs.push_back(feature);
  dblptr = new double(1.0);
  coefficients.push_back(dblptr);
  return dblptr;
}

// Private member functions

void Subscriber::readConfigMsg(const tapi_msgs::Connection::ConstPtr& msg)
{
  for (int i = 0; i < featureMsgs.size(); i++)
  {
    if (msg->SubscriberUUID == uuid && msg->SubscriberFeatureUUID == featureMsgs[i].UUID)
    {
      if (msg->PublisherUUID == "0" || msg->PublisherFeatureUUID == "0")
        if (subscribers[i].second)
        {
          subscribers[i].second->shutdown();
          delete subscribers[i].second;
          subscribers[i].second = 0;
        }
        else
          ;
      else
      {
        if (topicName != "Tapi/" + msg->SenderUUID + "/" + msg->SenderFeatureUUID || subscribers[i].second == 0)
        {
          topicName = "Tapi/" + msg->SenderUUID + "/" + msg->SenderFeatureUUID;
          subscribers[i].first.topic = topicName;
          if (subscribers[i].second)
          {
            subscribers[i].second->shutdown();
            delete subscribers[i].second;
            subscribers[i].second = 0;
          }
          subscribers[i].second = new ros::Subscriber(nh->subscribe(subscribers[i].first));
        }
        *coefficients[i] = msg->Coefficient;
      }
    }
  }
}
}
