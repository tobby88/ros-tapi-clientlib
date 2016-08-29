#ifndef TOBBYAPI_CLIENT_H
#define TOBBYAPI_CLIENT_H

#define WAIT_MS_ON_ERROR 1000L
#define SENDER_DEVICE 1
#define RECEIVER_DEVICE 2

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <string>
#include <thread>
#include <vector>
#include "tobbyapi_msgs/Feature.h"
#include "tobbyapi_msgs/HelloRequest.h"

namespace TobbyAPI
{
class TobbyApiClient
{
public:
  // Constructor/Destructor
  TobbyApiClient(ros::NodeHandle* nh, std::string nodename, uint8_t deviceType);
  ~TobbyApiClient();

protected:
  // Protected meber functions
  std::string generateUUID();
  std::string getNextFeatureUUID();

  // Protected member variables
  std::vector<tobbyapi_msgs::Feature> featureMsgs;
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
#endif  // TOBBYAPI_CLIENT_H
