#include "publisher.hpp"
#include "sensor_msgs/CompressedImage.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "tapi_msgs/Feature.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

Publisher::Publisher(ros::NodeHandle* nh, string nodename)
  : TapiClient(nh, nodename, SENDER_DEVICE), nh(nh), nodename(nodename)
{
}

Publisher::~Publisher()
{
  for (int i = 0; i < publishers.size(); i++)
  {
    publishers[i]->shutdown();
    delete publishers[i];
  }
}

// Public member functions

ros::Publisher* Publisher::AddFeature(uint8_t type, unsigned long queuesize, string featurename, string description)
{
  string featureUUID = getNextFeatureUUID();
  ros::Publisher* publisher = 0;
  string publisherName = "Tapi/" + uuid + "/" + featureUUID;

  switch (type)
  {
    case tapi_msgs::Feature::Type_AnalogValue:
      publisher = new ros::Publisher(nh->advertise<std_msgs::Float64>(publisherName, queuesize));
      break;
    case tapi_msgs::Feature::Type_Images:
      publisher = new ros::Publisher(nh->advertise<sensor_msgs::CompressedImage>(publisherName, queuesize));
      break;
    case tapi_msgs::Feature::Type_Switch:
      publisher = new ros::Publisher(nh->advertise<std_msgs::Bool>(publisherName, queuesize));
      break;
    case tapi_msgs::Feature::Type_Tristate:
      publisher = new ros::Publisher(nh->advertise<std_msgs::Int8>(publisherName, queuesize));
      break;
    default:
      break;
  }
  if (publisher)
  {
    publishers.push_back(publisher);
    tapi_msgs::Feature feature;
    feature.FeatureType = type;
    feature.Name = featurename;
    feature.Description = description;
    feature.UUID = featureUUID;
    featureMsgs.push_back(feature);
  }
  return publisher;
}
}
