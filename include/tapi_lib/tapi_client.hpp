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
 * \file tapi_client.hpp
 * \ingroup tapi_lib
 * \author Tobias Holst
 * \date 26 Aug 2016
 * \brief Declaration of the Tapi::TapiClient-class and definition of its member variables
 */

#ifndef TAPI_CLIENT_H
#define TAPI_CLIENT_H

#define WAIT_MS_ON_ERROR 1000L
#define PUBLISHER_DEVICE 1
#define SUBSCRIBER_DEVICE 2
#define SERVICE_SERVER 3
#define SERVICE_CLIENT 4

#include <string>
#include <thread>
#include <vector>
#include "ros/node_handle.h"
#include "ros/service_client.h"
#include "std_msgs/Header.h"
#include "tapi_lib/Feature.h"

namespace Tapi
{
/*!
 * \brief Create Tapi-compliant devices with the help of this class
 * Tapi::TapiClient
 * \author Tobias Holst
 * \version 2.1.1
 */
class TapiClient
{
public:
  // Constructor/Destructor

  /*!
   * \brief Create a TapiClient object as the helper to connect to the core of Tapi and create Tapi-compliant
   * publishers/subscribers/serviceservers/serviceclients. Usually called by classes who inherits from Tapi::TapiClient
   * \param nh Pointer to a \c ros::NodeHandle created outside of this class
   * \param nodename Name of the device
   * \param deviceType Type of the device, 1 for a PUBLISHER_DEVICE, 2 for a SUBSCRIBER_DEVICE, 3 for a SERVICE_SERVER
   * and 4 for a SERVICE_CLIENT
   */
  TapiClient(ros::NodeHandle* nh, std::string nodename, uint8_t deviceType);

  //! Shutdown the Hello-client and its heartbeat-thread and free the memory
  ~TapiClient();

protected:
  // Protected meber functions

  /*!
   * \brief Connect to the Hello-Service on /Tapi/HelloServ and send our device data
   * \return \c true if connected successfully, \c false if not
   * \see \c Hello.srv
   */
  bool connect();

  /*!
   * \brief Generates a new unique id by using the uuid library and then convert it to a ros compatible string
   * (underscores instead of dashes)
   * \return Unique ID as a ros compatible string
   */
  std::string generateUUID();

  /*!
   * \brief Get the next id of/for a feature
   *
   * It will use the ID from the config-file if there still is one. If not it will generate on and append it to the
   * config file.
   * \return Next unique id for a feature as a ros compatible string
   * \see Tapi::TapiClient::loadUUIDS
   */
  std::string getNextFeatureUUID();

  // Protected member variables

  /*!
   * \brief \c vector of Feature-messages to call the Hello-service
   *
   * They are generated in the inherited classes and contain the information about the features.
   * \see \c Feature.msg
   * \see Tapi::TapiClient::connect
   */
  std::vector<tapi_lib::Feature> featureMsgs;

  //! Unique ID of this device
  std::string uuid;

private:
  // Private member functions

  /*!
   * \brief Sends regular Hello-service calls to keep the device state "alive"
   *
   * Started in an extra (heartbeat-) thread. On failure it gives another try every WAIT_MS_ON_ERROR ms, on success it
   * waits the configured heartbeat time for this device.
   * \see Tapi::TapiClient::heartbeatThread
   * \see Tapi::TapiClient::connect
   * \see Tapi::TapiClient::heartbeatInterval
   */
  void heartbeat();

  /*!
   * \brief Open the config file for this device and load the stored uuids
   * \see Tapi::TapiClient::featureUUIDs
   */
  void loadUUIDs();

  // Private member variables

  /*!
   * \brief Store the type of the device
   *
   * It's 1 for PUBLISHER_DEVICE, 2 for SUBSCRIBER_DEVICE, 1 (=PUBLISHER_DEVICE) for SERVICE_SERVER and 2
   * (=SUBSCRIBER_DEVICE) for SERVICE_CLIENT
   */
  uint8_t deviceType;

  /*!
   * \brief All currently generated/loaded unique IDs for features
   * \see Tapi::TapiClient::loadUUIDs
   */
  std::vector<std::string> featureUUIDs;

  /*!
   * \brief The filename (including the path) where the uuid of the device is stored
   * \see Tapi::TapiClient::loadUUIDs
   */
  std::string filenameDevUUID;

  /*!
   * \brief The filename (including the path) where the uuids of the device's features are stored
   * \see Tapi::TapiClient::loadUUIDs
   */
  std::string filenameFeatureUUIDs;

  /*!
   * \brief Store whether it's the first time the node connects to Hello-Service for less unnecessary output of the \c
   * connect function
   * \see Tapi::TapiClient::connect
   */
  bool firstRun;

  /*!
   * \brief Header for the Hello calls
   * \see Tapi::TapiClient::connect
   */
  std_msgs::Header header;

  /*!
   * \brief Heartbeat (delay the device shall wait between two Hello-calls) configured for this device
   * \see Tapi::TapiClient::heartbeat
   */
  unsigned long heartbeatInterval;

  /*!
   * \brief Thread to run the \c heartbeat-function
   * \see Tapi::TapiClient::heartbeat
   */
  std::thread* heartbeatThread;

  /*!
   * \brief \c ros::ServiceClient to connect to the \c Hello service
   * \see Tapi::TapiClient::connect
   */
  ros::ServiceClient helloClient;

  //! NodeHandle-pointer necessary to create subscribers.
  ros::NodeHandle* nh;

  //! Name of the node/device
  std::string nodename;
};
}
#endif  // TAPI_CLIENT_H
