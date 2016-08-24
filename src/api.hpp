#ifndef API_H
#define API_H

#define STANDARD_HEARTBEAT_INTERVAL 2000L

#include "assignment.hpp"
#include "device.hpp"
#include "ros/ros.h"
#include "tobbyapi_msgs/Hello.h"
#include <map>
#include <string>
#include <vector>

class Api
{
public:
  // Constructor/Destructor
  Api(ros::NodeHandle* nh);
  ~Api();

  // Public member functions
  bool CheckPending();
  bool ConnectFeatures(std::string feature1UUID, std::string feature2UUID,
                       double coefficient);
  void DebugOutput();
  bool DeleteConnection(std::string receiverFeatureUUID);
  void Done();
  std::vector<Assignment*> GetConnections();
  std::vector<Device*> GetDevicesSorted();
  void Run();

private:
  // Private member variables
  ros::Publisher configPub;
  std::map<std::string, Assignment> connections;
  std::map<std::string, Device> devices;
  ros::ServiceServer helloServ;
  ros::NodeHandle* nh;
  bool pendingChanges;
  ros::AsyncSpinner* spinner;

  // Private member functions
  void changed();
  static bool compareDeviceNames(const Device* first, const Device* second);
  Device* getDeviceByFeatureUUID(std::string uuid);
  bool hello(tobbyapi_msgs::Hello::Request& helloReq,
             tobbyapi_msgs::Hello::Response& helloResp);
  void sendAllConnections();
};

#endif // API_H
