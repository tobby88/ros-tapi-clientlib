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

/*!
 * \file serviceserver.cpp
 * \ingroup tapi_lib
 * \author Tobias Holst
 * \date 06 Sep 2016
 * \brief Definition of the Tapi::ServiceServer-class and its member functions
 */

#include "include/tapi_lib/serviceserver.hpp"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

ServiceServer::ServiceServer(ros::NodeHandle* nh, string nodename)
  : TapiClient(nh, nodename, SERVICE_SERVER), nh(nh), nodename(nodename)
{
}

ServiceServer::~ServiceServer()
{
  for (int i = 0; i < services.size(); i++)
  {
    services[i].second->shutdown();
    delete services[i].second;
    services[i].second = 0;
  }
}

// Public member functions

ros::ServiceServer* ServiceServer::AddFeature(ros::AdvertiseServiceOptions opt, string featurename)
{
  // Get the unique id for our feature
  std::string featureUUID = getNextFeatureUUID();

  // Create the ServiceServer with the topic-/service-name generated from the uuid
  ros::ServiceServer* service = 0;
  std::string serviceName = "/Tapi/" + uuid + "/" + featureUUID;
  opt.service = serviceName;
  service = new ros::ServiceServer(nh->advertiseService(opt));

  // Service was created successfully, save the data in the services-vector and create the associated Feature message
  if (service)
  {
    pair<ros::AdvertiseServiceOptions, ros::ServiceServer*> newEntry;
    newEntry = make_pair(opt, service);
    services.push_back(newEntry);
    tapi_lib::Feature feature;
    string temp = opt.datatype;
    feature.FeatureType = temp;
    feature.Name = featurename;
    feature.UUID = featureUUID;
    featureMsgs.push_back(feature);
  }
  connect();
  return service;
}
}
