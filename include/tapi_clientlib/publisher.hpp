#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <string>
#include "ros/ros.h"
#include "tapi_client.hpp"

namespace Tapi
{
class Publisher : public TapiClient
{
public:
  // Constructor/Destructor
  Publisher(ros::NodeHandle *nh, std::string nodename = "");
  ~Publisher();

  // Public member functions
  ros::Publisher *AddFeature(uint8_t type, std::string featurename = "", std::string description = "");

private:
  // Private member variables
  ros::NodeHandle *nh;
  std::string nodename;
  std::vector<ros::Publisher *> publishers;
};
}

#endif  // PUBLISHER_H
