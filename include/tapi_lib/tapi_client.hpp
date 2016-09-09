/******************************************************************************
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

#ifndef TAPI_CLIENT_H
#define TAPI_CLIENT_H

#define WAIT_MS_ON_ERROR 1000L
#define PUBLISHER_DEVICE 1
#define SUBSCRIBER_DEVICE 2
#define SERVICE_SERVER 3
#define SERVICE_CLIENT 4

#include <string>
#include <thread>
#include <vector>
#include "ros/node_handle.h"
#include "ros/service_client.h"
#include "std_msgs/Header.h"
#include "tapi_lib/Feature.h"

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
  bool connect();
  std::string generateUUID();
  std::string getNextFeatureUUID();

  // Protected member variables
  std::vector<tapi_lib::Feature> featureMsgs;
  std::string uuid;

private:
  // Private member functions
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
