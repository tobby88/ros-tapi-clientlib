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
  ros::SubscribeOptions::create<type>("", buffer, boost::bind((callfctn), this, _1), ros::VoidPtr(this), NULL)

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
