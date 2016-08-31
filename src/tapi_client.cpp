#include "include/tapi_clientlib/tapi_client.hpp"
#include <uuid/uuid.h>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <thread>
#include "tapi_msgs/Hello.h"

using namespace ros;
using namespace std;

namespace Tapi
{
// Constructor/Destructor

TapiClient::TapiClient(NodeHandle* nh, string nodename, uint8_t deviceType)
  : nh(nh), nodename(nodename), deviceType(deviceType)
{
  firstRun = true;
  string homedir = getenv("HOME");
  filenameDevUUID = homedir + "/.ros/tapi_" + nodename + "_dev_uuid.txt";
  filenameFeatureUUIDs = homedir + "/.ros/tapi_" + nodename + "_feature_uuids.txt";
  loadUUIDs();
  helloClient = nh->serviceClient<tapi_msgs::Hello>("Tapi/HelloServ");
  heartbeatThread = new thread(&TapiClient::heartbeat, this);
}

TapiClient::~TapiClient()
{
  heartbeatThread->join();
  delete heartbeatThread;
  helloClient.shutdown();
}

// Protected member functions

string TapiClient::generateUUID()
{
  uuid_t uuidt;
  char uuid_array[37];
  string uuid_string;
  uuid_generate_random(uuidt);
  uuid_unparse(uuidt, uuid_array);
  uuid_string = uuid_array;
  replace(uuid_string.begin(), uuid_string.end(), '-', '_');
  return uuid_string;
}

void TapiClient::heartbeat()
{
  while (ros::ok())
  {
    bool success = false;
    success = connect();
    if (!success)
    {
      this_thread::sleep_for(chrono::milliseconds(WAIT_MS_ON_ERROR));
      continue;
    }
    else
      this_thread::sleep_for(chrono::milliseconds(heartbeatInterval));
  }
}

string TapiClient::getNextFeatureUUID()
{
  string uuid;
  if (featureMsgs.size() < featureUUIDs.size())
    uuid = featureUUIDs[featureMsgs.size()];
  else
  {
    uuid = generateUUID();
    ofstream uuidFileOutput;
    uuidFileOutput.open(filenameFeatureUUIDs, ios::app);
    featureUUIDs.push_back(uuid);
    uuidFileOutput << uuid << "\n";
    uuidFileOutput.close();
  }
  return uuid;
}

// Private member functions

bool TapiClient::connect()
{
  bool status = false;
  tapi_msgs::Hello hello;
  header.stamp = Time::now();
  header.seq++;
  hello.request.Header = header;
  hello.request.Name = nodename;
  hello.request.UUID = uuid;
  if (deviceType == RECEIVER_DEVICE)
    hello.request.DeviceType = tapi_msgs::HelloRequest::Type_ReceiverDevice;
  else if (deviceType == SENDER_DEVICE)
    hello.request.DeviceType = tapi_msgs::HelloRequest::Type_SenderDevice;
  else
  {
    ROS_ERROR("Unknown type of device");
    return false;
  }
  hello.request.Features = featureMsgs;
  if (helloClient.call(hello))
  {
    status = hello.response.Status;
    if (firstRun)
    {
      if (status)
        ROS_INFO("Connection established, Status OK, Heartbeat %u", hello.response.Heartbeat);
      else
        ROS_INFO("Connection error, Heartbeat %u", hello.response.Heartbeat);
      firstRun = false;
    }
    heartbeatInterval = hello.response.Heartbeat;
  }
  else
  {
    ROS_ERROR("Failed to establish connection to hello service");
    status = false;
  }

  return status;
}

void TapiClient::loadUUIDs()
{
  // Read (or generate, if not existing) Device-UUID
  string uuid;
  ifstream uuidFileInput;
  uuidFileInput.open(filenameDevUUID);
  if (uuidFileInput.is_open())
  {
    getline(uuidFileInput, uuid);
  }
  uuidFileInput.close();
  if (uuid.empty())
  {
    ofstream uuidFileOutput;
    uuidFileOutput.open(filenameDevUUID);
    uuid = generateUUID();
    uuidFileOutput << uuid;
    uuidFileOutput.close();
  }
  this->uuid = uuid;

  // Now also look for Feature-UUIDs
  uuidFileInput.open(filenameFeatureUUIDs);
  if (uuidFileInput.is_open())
  {
    featureUUIDs.clear();
    while (getline(uuidFileInput, uuid))
      featureUUIDs.push_back(uuid);
  }
  uuidFileInput.close();
  return;
}
}
