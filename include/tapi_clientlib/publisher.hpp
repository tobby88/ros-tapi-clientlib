#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <string>
#include <vector>
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscribe_options.h"
#include "tapi_client.hpp"
#include "tapi_msgs/Feature.h"

namespace Tapi
{
class Publisher : public TapiClient
{
public:
  // Constructor/Destructor
  Publisher(ros::NodeHandle *nh, std::string nodename = "");
  ~Publisher();

  // Public member functions
  // Templated functions
  template <typename T>
  ros::Publisher *AddFeature(std::string featurename = "", unsigned long queuesize = 1)
  {
    std::string featureUUID = getNextFeatureUUID();
    ros::Publisher *publisher = 0;
    std::string publisherName = "/Tapi/" + uuid + "/" + featureUUID;

    publisher = new ros::Publisher(nh->advertise<T>(publisherName, queuesize));

    if (publisher)
    {
      publishers.push_back(publisher);
      tapi_msgs::Feature feature;
      ros::SubscribeOptions temp;
      temp = ros::SubscribeOptions::create<T>("", 1, NULL, ros::VoidPtr(), NULL);
      feature.FeatureType = temp.datatype;
      feature.Name = featurename;
      feature.UUID = featureUUID;
      featureMsgs.push_back(feature);
    }
    return publisher;
  }

private:
  // Private member variables
  ros::NodeHandle *nh;
  std::string nodename;
  std::vector<ros::Publisher *> publishers;
};
}

#endif  // PUBLISHER_H
