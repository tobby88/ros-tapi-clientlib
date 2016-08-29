#include <string>
#include "ros/ros.h"
#include "src/tobbyapi_client.hpp"

namespace TobbyAPI
{
class Publisher : public TobbyApiClient
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
