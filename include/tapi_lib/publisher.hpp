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

#ifndef PUBLISHER_H
#define PUBLISHER_H

#include <string>
#include <vector>
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscribe_options.h"
#include "tapi_client.hpp"
#include "tapi_lib/Feature.h"

namespace Tapi
{
class Publisher : public TapiClient
{
public:
  // Constructor/Destructor
  Publisher(ros::NodeHandle *nh, std::string nodename = "");
  ~Publisher();

  // Public member functions
  // Templated functions
  template <typename T>
  ros::Publisher *AddFeature(std::string featurename = "", unsigned long queuesize = 1)
  {
    std::string featureUUID = getNextFeatureUUID();
    ros::Publisher *publisher = 0;
    std::string publisherName = "/Tapi/" + uuid + "/" + featureUUID;

    publisher = new ros::Publisher(nh->advertise<T>(publisherName, queuesize));

    if (publisher)
    {
      publishers.push_back(publisher);
      tapi_lib::Feature feature;
      ros::SubscribeOptions temp;
      temp = ros::SubscribeOptions::create<T>("", 1, NULL, ros::VoidPtr(), NULL);
      feature.FeatureType = temp.datatype;
      feature.Name = featurename;
      feature.UUID = featureUUID;
      featureMsgs.push_back(feature);
    }
    connect();
    return publisher;
  }

private:
  // Private member variables
  ros::NodeHandle *nh;
  std::string nodename;
  std::vector<ros::Publisher *> publishers;
};
}

#endif  // PUBLISHER_H
