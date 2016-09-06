#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <string>
#include <vector>
#include "ros/node_handle.h"
#include "ros/subscribe_options.h"
#include "ros/subscriber.h"
#include "tapi_client.hpp"
#include "tapi_lib/Connection.h"

#define SubscribeOptionsForTapi(type, buffer, callfctn)                                                                \
  ros::SubscribeOptions::create<type>("", buffer, boost::bind((callfctn), this, _1), ros::VoidPtr(), NULL)

namespace Tapi
{
class Subscriber : public TapiClient
{
public:
  // Constructor/Destructor
  Subscriber(ros::NodeHandle *nh, std::string nodename = "");
  ~Subscriber();

  // Public member functions
  double *AddFeature(ros::SubscribeOptions opt, std::string featurename = "");

private:
  // Private member functions
  void readConfigMsg(const tapi_lib::Connection::ConstPtr &msg);

  // Private member variables
  std::vector<double *> coefficients;
  ros::Subscriber configSub;
  ros::NodeHandle *nh;
  std::string nodename;
  std::vector<std::pair<ros::SubscribeOptions, ros::Subscriber *>> subscribers;
  std::string topicName;
};
}

#endif  // SUBSCRIBER_H
