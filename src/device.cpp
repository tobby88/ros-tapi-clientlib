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

#include "include/tapi_lib/device.hpp"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

Device::Device(uint8_t type, string name, string uuid, unsigned long lastSeq, ros::Time lastSeen,
               unsigned long heartbeat, map<string, Feature> features)
  : type(type), name(name), uuid(uuid), lastSeq(lastSeq), lastSeen(lastSeen), heartbeat(heartbeat), features(features)
{
  active = true;
}

Device::~Device()
{
}

// Public member functions

bool Device::Active()
{
  return active;
}

void Device::Deactivate()
{
  active = false;
}

Feature* Device::GetFeatureByUUID(string uuid)
{
  if (features.count(uuid) > 0)
    return &features.at(uuid);
  else
    return 0;
}

unsigned long Device::GetHeartbeat()
{
  return heartbeat;
}

ros::Time Device::GetLastSeen()
{
  return lastSeen;
}

unsigned long Device::GetLastSeq()
{
  return lastSeq;
}

string Device::GetName() const
{
  if (name.empty())
    return uuid;
  else
    return name;
}

vector<Feature*> Device::GetSortedFeatures()
{
  vector<Feature*> featureList;
  for (auto it = features.begin(); it != features.end(); ++it)
    featureList.push_back(&it->second);
  if (featureList.size() > 1)
    sort(featureList.begin(), featureList.end(), compareFeatureNames);
  return featureList;
}

uint8_t Device::GetType()
{
  return type;
}

string Device::GetUUID()
{
  return uuid;
}

void Device::Update(uint8_t type, string name, unsigned long lastSeq, ros::Time lastSeen, unsigned long heartbeat,
                    map<string, Feature> featureUpdate)
{
  this->type = type;
  this->name = name;
  this->lastSeq = lastSeq;
  this->lastSeen = lastSeen;
  this->heartbeat = heartbeat;
  if (featureUpdate.size() == 0)
  {
    features.clear();
    return;
  }
  if (features.size() == 0)
  {
    features = featureUpdate;
    return;
  }

  bool equ = true;
  if (features.size() == featureUpdate.size())
  {
    for (auto it = features.begin(); it != features.end(); ++it)
    {
      if (!featureUpdate.count(it->second.GetUUID()) == 1)
        equ = false;
      else if (!(featureUpdate.at(it->second.GetUUID()) == it->second))
        equ = false;
    }
  }
  else
    equ = false;

  if (!equ)
  {
    for (auto it = featureUpdate.begin(); it != featureUpdate.end(); ++it)
    {
      if (features.count(it->second.GetUUID()) == 0)
        features.emplace(it->second.GetUUID(), it->second);
      else
        features.at(it->second.GetUUID()).Update(it->second.GetType(), it->second.GetName());
    }
    for (auto it = features.begin(); it != features.end(); ++it)
    {
      if (featureUpdate.count(it->second.GetUUID()) == 0)
        features.erase(it->second.GetUUID());
    }
  }
  active = true;
}

// Private member functions

bool Device::compareFeatureNames(const Feature* first, const Feature* second)
{
  string temp1, temp2;
  temp1 = first->GetName();
  temp2 = second->GetName();
  transform(temp1.begin(), temp1.end(), temp1.begin(), ::towlower);
  transform(temp2.begin(), temp2.end(), temp2.begin(), ::towlower);
  bool result = temp1 < temp2;
  return result;
}
}
