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
 * \file device.hpp
 * \ingroup tapi_lib
 * \author Tobias Holst
 * \date 20 Nov 2015
 * \brief Declaration of the Tapi::Device-class and definition of its member variables
 */

#ifndef DEVICE_H
#define DEVICE_H

#include <map>
#include <string>
#include <vector>
#include "feature.hpp"
#include "ros/time.h"

namespace Tapi
{
/*!
 * \brief Store information about a Tapi-compliant device, necessary e.g. for the core (\c tapi_core) or the gui (\c
 * tapi_gui)
 * \author Tobias Holst
 * \version 2.1.1
 */
class Device
{
public:
  // Constructor/Destructor

  /*!
   * \brief Create a \c Device object
   * \param type Type of the device (tapi_lib::Device::Type_Publisher for Publisher and ServiceServer or
   * tapi_lib::Device::Type_Subscriber for Subscriber and ServiceClients)
   * \param name Name of the device
   * \param uuid Unique ID of the device
   * \param lastSeq Sequence number of the header of the device when it has been seen for the last time
   * \param lastSeen Timestamp when the device has been seen for the last time
   * \param heartbeat Heartbeat (delay the device shall wait between two Hello-calls) configured for this device
   * \param features Map of Tapi::Feature objects to describe the features of the device
   * \see \c Device.msg
   * \see Tapi::Feature
   */
  Device(uint8_t type, std::string name, std::string uuid, unsigned long lastSeq, ros::Time lastSeen,
         unsigned long heartbeat, std::map<std::string, Feature> features);

  //! Empty destructor
  ~Device();

  // Public member functions

  /*!
   * \brief Get the state of the device
   * \return \c true if the device is active, \c false if the device is inactive (hasn't connected to hello service for
   * a longer time)
   * \see Tapi::Device::active
   */
  bool Active();

  /*!
   * \brief Mark the device as inactive
   * \see Tapi::Device::Active
   * \see Tapi::Device::active
   */
  void Deactivate();

  /*!
   * \brief Get a pointer to a feature of this device with a given feature uuid
   * \param uuid Unique ID of the wanted feature
   * \return Pointer to the feature if found, 0 if not found
   */
  Feature* GetFeatureByUUID(std::string uuid);

  /*!
   * \brief Get the configured heartbeat time (time the device shall wait between calls of the Hello-service)
   * \return Heartbeat time
   * \see \c Tapi::TapiCore::hello in the \c tapi_core package
   * \see Tapi::Device::heartbeat
   */
  unsigned long GetHeartbeat();

  /*!
   * \brief Timestamp of the device's last Hello call
   * \return Last-seen timestamp
   * \see Tapi::Device::lastSeen
   */
  ros::Time GetLastSeen();

  /*!
   * \brief Sequence number in the header when the device has been seen the last time
   * \return Sequence number of the last call
   * \see Tapi::Device::lastSeq
   */
  unsigned long GetLastSeq();

  /*!
   * \brief Get the name of the device
   * \return Device's name
   * \see Tapi::Device::name
   */
  std::string GetName() const;

  /*!
   * \brief Get a \c vector of pointers to all device's features in alphabetical order
   * \return A \c vector of \c Feature-pointers in alphabetical order
   * \see Tapi::Device::features
   * \see Tapi::Feature
   */
  std::vector<Feature*> GetSortedFeatures();

  /*!
   * \brief Get the type of the device (tapi_lib::Device::Type_Publisher for Publisher and ServiceServer or
   * tapi_lib::Device::Type_Subscriber for Subscriber and ServiceClients)
   * \return Type of the device (tapi_lib::Device::Type_Publisher for Publisher and ServiceServer or
   * tapi_lib::Device::Type_Subscriber for Subscriber and ServiceClients)
   * \see \c Device.msg
   */
  uint8_t GetType();

  /*!
   * \brief Get the unique ID of the device
   * \return The unique ID of the device
   * \see Tapi::Device::uuid
   */
  std::string GetUUID();

  /*!
   * \brief Update the data of the device
   * \param type Type of the device (tapi_lib::Device::Type_Publisher for Publisher and ServiceServer or
   * tapi_lib::Device::Type_Subscriber for Subscriber and ServiceClients)
   * \param name Name of the device
   * \param lastSeq Sequence number of the header of the device when it has been seen for the last time
   * \param lastSeen Timestamp when the device has been seen for the last time
   * \param heartbeat Heartbeat (delay the device shall wait between two Hello-calls) configured for this device
   * \param features Map of Tapi::Feature objects to describe the features of the device
   * \see \c Device.msg
   * \see Tapi::Feature
   */
  void Update(uint8_t type, std::string name, unsigned long lastSeq, ros::Time lastSeen, unsigned long heartbeat,
              std::map<std::string, Feature> features);

private:
  // Private member variables

  /*!
   * \brief Saves the state of the device (\c true = working/connected to tapi_core, \c false = not connected to the
   * core)
   * \see Tapi::Device::Active
   * \see Tapi::Device::Deactivate
   */
  bool active;

  //! Map of Tapi::Feature objects to describe the features of the device
  std::map<std::string, Feature> features;

  //! Heartbeat (delay the device shall wait between two Hello-calls) configured for this device
  unsigned long heartbeat;

  //! lastSeen Timestamp when the device has been seen for the last time
  ros::Time lastSeen;

  //! Sequence number of the header of the device when it has been seen for the last time
  unsigned long lastSeq;

  //! Name of the device
  std::string name;

  /*!
   * \brief Type of the device (tapi_lib::Device::Type_Publisher for Publisher and ServiceServer or
   * tapi_lib::Device::Type_Subscriber for Subscriber and ServiceClients)
   * \see \c Device.msg
   */
  uint8_t type;

  //! Unique ID of the device
  std::string uuid;

  // Private member functions

  /*!
   * \brief To sort Feature there has to be a compare function, comparing their names.
   * \param first Pointer to the first Feature to compare
   * \param second Pointer to the second Feature to compare
   * \return \c true if the first Feature's name has to be above the second's Feature name in an alphabetically sort, \c
   * false when it's vice versa
   * \see Tapi::Feature
   */
  static bool compareFeatureNames(const Feature* first, const Feature* second);
};
}

#endif  // DEVICE_H
