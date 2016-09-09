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

#include "include/tapi_lib/subscriber.hpp"
#include "tapi_lib/Feature.h"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

Subscriber::Subscriber(ros::NodeHandle* nh, string nodename)
  : TapiClient(nh, nodename, SUBSCRIBER_DEVICE), nh(nh), nodename(nodename)
{
  configSub = nh->subscribe("/Tapi/Config", 1000, &Subscriber::readConfigMsg, this);
}

Subscriber::~Subscriber()
{
  configSub.shutdown();
  for (int i = 0; i < subscribers.size(); i++)
  {
    subscribers[i].second->shutdown();
    delete subscribers[i].second;
    subscribers[i].second = 0;
    delete coefficients[i];
  }
}

// Public member functions

double* Subscriber::AddFeature(ros::SubscribeOptions opt, string featurename)
{
  double* dblptr = 0;
  ros::Subscriber* subscriber = 0;
  pair<ros::SubscribeOptions, ros::Subscriber*> newEntry;
  newEntry = make_pair(opt, subscriber);
  subscribers.push_back(newEntry);
  tapi_lib::Feature feature;
  string temp = opt.datatype;
  feature.FeatureType = temp;
  feature.Name = featurename;
  feature.UUID = getNextFeatureUUID();
  featureMsgs.push_back(feature);
  dblptr = new double(1.0);
  coefficients.push_back(dblptr);
  connect();
  return dblptr;
}

// Private member functions

void Subscriber::readConfigMsg(const tapi_lib::Connection::ConstPtr& msg)
{
  for (int i = 0; i < featureMsgs.size(); i++)
  {
    if (msg->SubscriberUUID == uuid && msg->SubscriberFeatureUUID == featureMsgs[i].UUID)
    {
      if (msg->PublisherUUID == "0" || msg->PublisherFeatureUUID == "0")
        if (subscribers[i].second)
        {
          subscribers[i].second->shutdown();
          delete subscribers[i].second;
          subscribers[i].second = 0;
        }
        else
          ;
      else
      {
        if (topicName != "/Tapi/" + msg->PublisherUUID + "/" + msg->PublisherFeatureUUID || subscribers[i].second == 0)
        {
          topicName = "/Tapi/" + msg->PublisherUUID + "/" + msg->PublisherFeatureUUID;
          subscribers[i].first.topic = topicName;
          if (subscribers[i].second)
          {
            subscribers[i].second->shutdown();
            delete subscribers[i].second;
            subscribers[i].second = 0;
          }
          subscribers[i].second = new ros::Subscriber(nh->subscribe(subscribers[i].first));
        }
        *coefficients[i] = msg->Coefficient;
      }
    }
  }
}
}
