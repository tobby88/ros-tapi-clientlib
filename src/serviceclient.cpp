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
 * \file serviceclient.cpp
 * \ingroup tapi_lib
 * \author Tobias Holst
 * \date 06 Sep 2016
 * \brief Definition of the Tapi::ServiceClient-class and its member functions
 */

#include "include/tapi_lib/serviceclient.hpp"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

ServiceClient::ServiceClient(ros::NodeHandle* nh, string nodename)
  : TapiClient(nh, nodename, SERVICE_CLIENT), nh(nh), nodename(nodename)
{
  configSub = nh->subscribe("/Tapi/Config", 1000, &ServiceClient::readConfigMsg, this);
}

ServiceClient::~ServiceClient()
{
  configSub.shutdown();
  for (int i = 0; i < clients.size(); i++)
  {
    clients[i].second->shutdown();
    delete clients[i].second;
    clients[i].second = 0;
  }
}

// Private member functions
void ServiceClient::readConfigMsg(const tapi_lib::Connection::ConstPtr& msg)
{
  // Iterate through all of our features to check if this Config message contains a uuid of us
  for (int i = 0; i < featureMsgs.size(); i++)
  {
    if (msg->SubscriberUUID == uuid && msg->SubscriberFeatureUUID == featureMsgs[i].UUID)
    {
      // Message is for us. Now check if we shall disconnect
      if (msg->PublisherUUID == "0" || msg->PublisherFeatureUUID == "0")
        if (clients[i].second)
        {
          clients[i].second->shutdown();
          delete clients[i].second;
          clients[i].second = 0;
        }
        else
          ;
      // Or we shall connect
      else
      {
        // Check if there currently is a connection and if it's the same as we shall connect. Because only if we change
        // the connection we need to stop the old service and start/create the new one
        if (serviceName[i] != "/Tapi/" + msg->PublisherUUID + "/" + msg->PublisherFeatureUUID || clients[i].second == 0)
        {
          serviceName[i] = "/Tapi/" + msg->PublisherUUID + "/" + msg->PublisherFeatureUUID;
          clients[i].first.service = serviceName[i];
          if (clients[i].second)
          {
            clients[i].second->shutdown();
            delete clients[i].second;
            clients[i].second = 0;
          }
          clients[i].second = new ros::ServiceClient(nh->serviceClient(clients[i].first));
        }
      }
    }
  }
}
}
