/******************************************************************************
*  Copyright (C) 2016 by Tobias Holst                                         *
*                                                                             *
*  This file is part of tapi_lib.                                             *
*                                                                             *
*  tapi_lib is free software: you can redistribute it and/or modify           *
*  it under the terms of the GNU General Public License as published by       *
*  the Free Software Foundation, either version 3 of the License, or          *
*  (at your option) any later version.                                        *
*                                                                             *
*  tapi_lib is distributed in the hope that it will be useful,                *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of             *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
*  GNU General Public License for more details.                               *
*                                                                             *
*  You should have received a copy of the GNU General Public License          *
*  along with tapi_lib.  If not, see <http://www.gnu.org/licenses/>.          *
*                                                                             *
*  Diese Datei ist Teil von tapi_lib.                                         *
*                                                                             *
*  tapi_lib ist Freie Software: Sie können es unter den Bedingungen           *
*  der GNU General Public License, wie von der Free Software Foundation,      *
*  Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren                 *
*  veröffentlichten Version, weiterverbreiten und/oder modifizieren.          *
*                                                                             *
*  tapi_lib wird in der Hoffnung, dass es nützlich sein wird, aber            *
*  OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite         *
*  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK  *
*  Siehe die GNU General Public License für weitere Details.                  *
*                                                                             *
*  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem  *
*  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>. *
*******************************************************************************/

#include "include/tapi_lib/tapi_client.hpp"
#include <uuid/uuid.h>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <thread>
#include "tapi_lib/Device.h"
#include "tapi_lib/Hello.h"

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
  filenameDevUUID = homedir + "/.ros/tapi_" + nodename + to_string((int)deviceType) + "_dev_uuid.txt";
  filenameFeatureUUIDs = homedir + "/.ros/tapi_" + nodename + to_string((int)deviceType) + "_feature_uuids.txt";
  loadUUIDs();
  helloClient = nh->serviceClient<tapi_lib::Hello>("/Tapi/HelloServ");
  heartbeatThread = new thread(&TapiClient::heartbeat, this);
  if (this->deviceType > 2)
    this->deviceType -= 2;
}

TapiClient::~TapiClient()
{
  heartbeatThread->join();
  delete heartbeatThread;
  helloClient.shutdown();
}

// Protected member functions

bool TapiClient::connect()
{
  bool status = false;
  tapi_lib::Hello hello;
  header.stamp = Time::now();
  header.seq++;
  hello.request.Header = header;
  hello.request.Name = nodename;
  hello.request.UUID = uuid;
  if (deviceType == SUBSCRIBER_DEVICE)
    hello.request.DeviceType = tapi_lib::Device::Type_Subscriber;
  else if (deviceType == PUBLISHER_DEVICE)
    hello.request.DeviceType = tapi_lib::Device::Type_Publisher;
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
