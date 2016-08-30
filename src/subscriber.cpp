#include "subscriber.hpp"
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "tobbyapi_msgs/Feature.h"

using namespace std;

namespace TobbyAPI
{
// Constructor/Destructor

Subscriber::Subscriber(ros::NodeHandle* nh, string nodename)
  : TobbyApiClient(nh, nodename, RECEIVER_DEVICE), nh(nh), nodename(nodename)
{
  subscribers.clear();
  configSub = nh->subscribe("TobbyAPI/Config", 1000, &Subscriber::readConfigMsg, this);
}

Subscriber::~Subscriber()
{
  for (int i = 0; i < subscribers.size(); i++)
  {
    subscribers[i].second->shutdown();
    delete subscribers[i].second;
    delete coefficients[i];
  }
}

// Public member functions

double* Subscriber::AddFeature(ros::SubscribeOptions opt, string featurename, string description)
{
  double* dblptr = 0;
  uint8_t type;
  bool error = false;
  opt.topic = "TobbyAPI/" + uuid + "/" + generateUUID();

  if (opt.datatype == "std_msgs/Float64")
    type = tobbyapi_msgs::Feature::Type_AnalogValue;
  else if (opt.datatype == "sensor_msgs/CompressedImage")
    type = tobbyapi_msgs::Feature::Type_Images;
  else if (opt.datatype == "std_msgs/Bool")
    type = tobbyapi_msgs::Feature::Type_Switch;
  else if (opt.datatype == "std_msgs/Int8")
    type = tobbyapi_msgs::Feature::Type_Tristate;
  else
    error = true;

  if (!error)
  {
    ros::Subscriber* subscriber = 0;
    string featureUUID = getNextFeatureUUID();
    opt.topic = "TobbyAPI/" + uuid + "/" + featureUUID;
    pair<ros::SubscribeOptions, ros::Subscriber*> newEntry;
    newEntry = make_pair(opt, subscriber);
    subscribers.push_back(newEntry);
    tobbyapi_msgs::Feature feature;
    feature.FeatureType = type;
    feature.Name = featurename;
    feature.Description = description;
    feature.UUID = featureUUID;
    featureMsgs.push_back(feature);
    dblptr = new double(1.0);
    coefficients.push_back(dblptr);
  }
  return dblptr;
}

// Private member functions

void Subscriber::readConfigMsg(const tobbyapi_msgs::Config::ConstPtr& msg)
{
  for (int i = 0; i < featureMsgs.size(); i++)
  {
    if (msg->ReceiverUUID == uuid && msg->ReceiverFeatureUUID == featureMsgs[i].UUID)
    {
      if (msg->SenderUUID == "0" || msg->SenderFeatureUUID == "0")
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
        if (topicName != "TobbyAPI/" + msg->SenderUUID + "/" + msg->SenderFeatureUUID)
        {
          topicName = "TobbyAPI/" + msg->SenderUUID + "/" + msg->SenderFeatureUUID;
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
