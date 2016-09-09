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

#ifndef DEVICE_H
#define DEVICE_H

#include <map>
#include <string>
#include <vector>
#include "feature.hpp"
#include "ros/time.h"

namespace Tapi
{
class Device
{
public:
  // Constructor/Destructor
  Device(uint8_t type, std::string name, std::string uuid, unsigned long lastSeq, ros::Time lastSeen,
         unsigned long heartbeat, std::map<std::string, Feature> features);
  ~Device();

  // Public member functions
  bool Active();
  void Deactivate();
  Feature* GetFeatureByUUID(std::string uuid);
  unsigned long GetHeartbeat();
  ros::Time GetLastSeen();
  unsigned long GetLastSeq();
  std::string GetName() const;
  std::vector<Feature*> GetSortedFeatures();
  uint8_t GetType();
  std::string GetUUID();
  void Update(uint8_t type, std::string name, unsigned long lastSeq, ros::Time lastSeen, unsigned long heartbeat,
              std::map<std::string, Feature> features);

private:
  // Private member variables
  bool active;
  std::map<std::string, Feature> features;
  unsigned long heartbeat;
  ros::Time lastSeen;
  unsigned long lastSeq;
  std::string name;
  uint8_t type;
  std::string uuid;

  // Private member functions
  static bool compareFeatureNames(const Feature* first, const Feature* second);
};
}

#endif  // DEVICE_H
