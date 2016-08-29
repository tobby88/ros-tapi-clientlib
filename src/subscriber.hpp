#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <string>
#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "tobbyapi_client.hpp"
#include "tobbyapi_msgs/Config.h"

#define SubscribeOptionsForAPI(type, buffer, callfctn)                                                                 \
  ros::SubscribeOptions::create<type>("", buffer, boost::bind((callfctn), this, _1), ros::VoidPtr(), NULL)

namespace TobbyAPI
{
class Subscriber : public TobbyApiClient
{
public:
  // Constructor/Destructor
  Subscriber(ros::NodeHandle *nh, std::string nodename = "");
  ~Subscriber();

  // Public member functions
  void AddFeature(ros::SubscribeOptions opt, std::string featurename = "", std::string description = "");

private:
  // Private member functions
  void readConfigMsg(const tobbyapi_msgs::Config::ConstPtr &msg);

  // Private member variables
  ros::Subscriber configSub;
  ros::NodeHandle *nh;
  std::string nodename;
  std::vector<std::pair<ros::SubscribeOptions, ros::Subscriber *>> subscribers;
};
}

#endif  // SUBSCRIBER_H
