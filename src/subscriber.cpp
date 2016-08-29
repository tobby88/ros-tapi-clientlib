#include <include/tobbyapi_clientlib/subscriber.hpp>
#include "functional"
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "tobbyapi_msgs/Feature.h"

#define SubscribeOptionsForAPI(type, buffer, callfctn)                                                                 \
  ros::SubscribeOptions::create<type>("", buffer, boost::bind((callfctn), this, _1), ros::VoidPtr(), NULL)

using namespace std;

namespace TobbyAPI
{
// Constructor/Destructor

Subscriber::Subscriber(ros::NodeHandle* nh, string nodename)
  : TobbyApiClient(nh, nodename, RECEIVER_DEVICE), nh(nh), nodename(nodename)
{
}

Subscriber::~Subscriber()
{
  for (int i = 0; i < subscribers.size(); i++)
  {
    subscribers[i].second->shutdown();
    delete subscribers[i].second;
  }
}

// Public member functions

ros::Subscriber* Subscriber::AddFeature(ros::SubscribeOptions opt, string featurename, string description)
{
  uint8_t type;
  bool error = false;
  ros::Subscriber* subscriber = 0;
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
    string featureUUID = getNextFeatureUUID();
    opt.topic = "TobbyAPI/" + uuid + "/" + featureUUID;
    subscriber = new ros::Subscriber(nh->subscribe(opt));
    subscriber->shutdown();
    pair<ros::SubscribeOptions, ros::Subscriber*> newEntry;
    newEntry.first=opt;
    newEntry.second=subscriber;
    subscribers.push_back(newEntry);
    tobbyapi_msgs::Feature feature;
    feature.FeatureType = type;
    feature.Name = featurename;
    feature.Description = description;
    feature.UUID = featureUUID;
    featureMsgs.push_back(feature);
  }
  return subscriber;
}
}
