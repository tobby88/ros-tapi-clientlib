/******************************************************************************
 *  Copyright (C) 2016 by Tobias Holst                                        *
 *                                                                            *
 *  This file is part of tapi_lib.                                            *
 *                                                                            *
 *  tapi_lib is free software: you can redistribute it and/or modify          *
 *  it under the terms of the GNU General Public License as published by      *
 *  the Free Software Foundation, either version 3 of the License, or         *
 *  (at your option) any later version.                                       *
 *                                                                            *
 *  tapi_lib is distributed in the hope that it will be useful,               *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
 *  GNU General Public License for more details.                              *
 *                                                                            *
 *  You should have received a copy of the GNU General Public License         *
 *  along with tapi_lib.  If not, see <http://www.gnu.org/licenses/>.         *
 *                                                                            *
 *  Diese Datei ist Teil von tapi_lib.                                        *
 *                                                                            *
 *  tapi_lib ist Freie Software: Sie können es unter den Bedingungen          *
 *  der GNU General Public License, wie von der Free Software Foundation,     *
 *  Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren                *
 *  veröffentlichten Version, weiterverbreiten und/oder modifizieren.         *
 *                                                                            *
 *  tapi_lib wird in der Hoffnung, dass es nützlich sein wird, aber           *
 *  OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite        *
 *  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK *
 *  Siehe die GNU General Public License für weitere Details.                 *
 *                                                                            *
 *  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem *
 *  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.*
 ******************************************************************************/

#ifndef SERVICECLIENT_H
#define SERVICECLIENT_H

#include <string>
#include <vector>
#include "ros/node_handle.h"
#include "ros/service_client.h"
#include "ros/service_client_options.h"
#include "ros/subscriber.h"
#include "tapi_client.hpp"
#include "tapi_lib/Connection.h"
#include "tapi_lib/Feature.h"

namespace Tapi
{
class ServiceClient : public TapiClient
{
public:
  // Constructor/Destructor
  ServiceClient(ros::NodeHandle *nh, std::string nodename = "");
  ~ServiceClient();

  // Public member functions
  // Templated functions
  template <typename T>
  ros::ServiceClient **AddFeature(std::string featurename = "")
  {
    ros::ServiceClientOptions opt;
    ros::M_string empty;
    ros::ServiceClient *client = 0;
    std::string featureUUID = getNextFeatureUUID();
    std::string temptopic = "/Tapi/" + uuid + "/" + featureUUID;
    opt.init<T>(temptopic, false, empty);
    ros::ServiceClient **clientptr = 0;
    std::pair<ros::ServiceClientOptions, ros::ServiceClient *> newEntry;
    newEntry = std::make_pair(opt, client);
    clients.push_back(newEntry);
    tapi_lib::Feature feature;
    ros::AdvertiseServiceOptions temp;
    temp = ros::AdvertiseServiceOptions::create<T>("", NULL, ros::VoidPtr(), NULL);
    std::string temp2 = temp.datatype;
    feature.FeatureType = temp2;
    feature.Name = featurename;
    feature.UUID = featureUUID;
    featureMsgs.push_back(feature);
    clientptr = &clients.at(clients.size() - 1).second;
    connect();
    return clientptr;
  }

private:
  // Private member functions
  void readConfigMsg(const tapi_lib::Connection::ConstPtr &msg);

  // Private member variables
  std::vector<std::pair<ros::ServiceClientOptions, ros::ServiceClient *>> clients;
  ros::Subscriber configSub;
  ros::NodeHandle *nh;
  std::string nodename;
  std::string serviceName;
};
}

#endif  // SERVICECLIENT_H
