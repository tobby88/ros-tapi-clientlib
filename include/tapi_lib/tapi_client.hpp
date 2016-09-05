#ifndef TAPI_CLIENT_H
#define TAPI_CLIENT_H

#define WAIT_MS_ON_ERROR 1000L
#define PUBLISHER_DEVICE 1
#define SUBSCRIBER_DEVICE 2

#include <string>
#include <thread>
#include <vector>
#include "ros/node_handle.h"
#include "ros/service_client.h"
#include "std_msgs/Header.h"
#include "tapi_msgs/Feature.h"

namespace Tapi
{
class TapiClient
{
public:
  // Constructor/Destructor
  TapiClient(ros::NodeHandle* nh, std::string nodename, uint8_t deviceType);
  ~TapiClient();

protected:
  // Protected meber functions
  std::string generateUUID();
  std::string getNextFeatureUUID();

  // Protected member variables
  std::vector<tapi_msgs::Feature> featureMsgs;
  std::string uuid;

private:
  // Private member functions
  bool connect();
  void heartbeat();
  void loadUUIDs();

  // Private member variables
  uint8_t deviceType;
  std::vector<std::string> featureUUIDs;
  std::string filenameDevUUID;
  std::string filenameFeatureUUIDs;
  bool firstRun;
  std_msgs::Header header;
  unsigned long heartbeatInterval;
  std::thread* heartbeatThread;
  ros::ServiceClient helloClient;
  ros::NodeHandle* nh;
  std::string nodename;
};
}
#endif  // TAPI_CLIENT_H
