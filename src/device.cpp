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
 * \file device.cpp
 * \ingroup tapi_lib
 * \author Tobias Holst
 * \date 20 Nov 2015
 * \brief Definition of the Tapi::Device-class and its member functions
 */

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
  // Iterate through all features and save a pointer to them in the vector
  vector<Feature*> featureList;
  for (auto it = features.begin(); it != features.end(); ++it)
    featureList.push_back(&it->second);

  // Vector contains more than on entry -> sort it alphabetically
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
  // First update basic data
  this->type = type;
  this->name = name;
  this->lastSeq = lastSeq;
  this->lastSeen = lastSeen;
  this->heartbeat = heartbeat;

  // New feature map is empty - clear our own feature map
  if (featureUpdate.size() == 0)
  {
    features.clear();
    return;
  }

  // Old feature map is empty, just use the new one as current feature map
  if (features.size() == 0)
  {
    features = featureUpdate;
    return;
  }

  // Both feature maps contain one or more features but the same number of feature -> compare them by iterating through
  // all and check if there exists a feature with the same uuid and if yes if they are identical
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

  // The number of features wasn't the same or at least one feature didn't exist or was different -> update the feature
  // map
  if (!equ)
  {
    // Iterate through all new features
    for (auto it = featureUpdate.begin(); it != featureUpdate.end(); ++it)
    {
      // If the feature doesn't exist -> add it to the feature map
      if (features.count(it->second.GetUUID()) == 0)
        features.emplace(it->second.GetUUID(), it->second);
      // Feature already exists -> just update its data
      else
        features.at(it->second.GetUUID()).Update(it->second.GetType(), it->second.GetName());
    }

    // Now iterate through all existing features and check, if there is one or more missing in the new feature map. If
    // yes -> delete it from our feature map.
    // TODO: Check if the pointers are safe -> Maybe it's necessary to first store what has to be deleted and then
    // iterate through the delete-vector to actually delete the features
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
