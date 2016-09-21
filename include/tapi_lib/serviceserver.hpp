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
 * \file serviceserver.hpp
 * \ingroup tapi_lib
 * \author Tobias Holst
 * \date 06 Sep 2016
 * \brief Declaration of the Tapi::ServiceClient-class and definition of its member variables
 */

#ifndef SERVICESERVER_H
#define SERVICESERVER_H

#include <string>
#include <vector>
#include "ros/advertise_service_options.h"
#include "ros/node_handle.h"
#include "ros/service_server.h"
#include "tapi_client.hpp"

#define ServiceServerOptionsForTapi(type, callfctn)                                                                    \
  ros::AdvertiseServiceOptions::create<type>("", boost::bind((callfctn), this, _1, _2), ros::VoidPtr(this), NULL)

namespace Tapi
{
/*!
 * \brief Create Tapi-compliant serviceservers with the help of this class, inherits its base functions from
 * Tapi::TapiClient
 * \author Tobias Holst
 * \version 2.1.1
 */
class ServiceServer : public TapiClient
{
public:
  // Constructor/Destructor

  /*!
   * \brief Create a ServiceServer object as the helper to connect to the core of Tapi and create Tapi-compliant
   * serviceservers
   * \param nh Pointer to a \c ros::NodeHandle created outside of this class
   * \param nodename Name of the device
   */
  ServiceServer(ros::NodeHandle *nh, std::string nodename = "");

  //! Shutdown all serviceservers and free the memory
  ~ServiceServer();

  // Public member functions

  /*!
   * \brief Create a \c ros::ServiceServer by adding a feature
   *
   * Gets a saved uuid or generates a new one with the help of Tapi::TapiClient. Stores the \c pair of \c
   * ros::AdvertiseServiceOptions and a pointer to the created ros::ServiceServer in the \c services \c vector. The
   * AdvertiseServiceOptions should be created with the help of the \c ServiceServerOptionsForTapi \c macro. At last the
   * informations are stored in the Feature-messages \c vector of Tapi::TapiClient.
   * \param opt Options to create the service, easiest way is to use \c ServiceServerOptionsForTapi \c macro
   * \param featurename (Descriptive) name of the feature
   * \return Pointer to the created \c ros::ServiceServer object.
   * \see Tapi::ServiceServer::services
   * \see Tapi::TapiClient
   * \see Tapi::Feature.msg
   * \code{.cpp}
   * // Example code to use the Tapi::ServiceServer class
   * // nh is a pointer to a ros::NodeHandle generated outside (e.g. in int main())
   * Tapi::ServiceServer* tservice = new Tapi::ServiceServer(nh, "Test");
   * // Option 1 (longer, but maybe easier to read)
   * ros::AdvertiseServiceOptions opt;
   * // Usage: ServiceServerOptionsForTapi(servicetype, &callback_function)
   * opt2 = ServiceServerOptionsForTapi(tapi_lib::GetDeviceList, &My_Classname::myCallbackFunction);
   * ros::ServiceServer* server1 = tservice->AddFeature(opt, "Device List");
   * // Option 2 (shorter, but maybe harder to read)
   * ros::ServiceServer* server2 = tservice->AddFeature(ServiceServerOptionsForTapi(tapi_lib::GetDeviceList,
   *                                                    &My_Classname::myCallbackFunction), "Device List");
   * // Don't delete ros::ServiceServer objects, only delete the Tapi::ServiceServer object,
   * // it will shutdown and delete the serviceservers in its destructor
   * delete tservice;
   * \endcode
   */
  ros::ServiceServer *AddFeature(ros::AdvertiseServiceOptions opt, std::string featurename = "");

private:
  // Private member variables

  //! NodeHandle-pointer necessary to create subscribers and serviceclients.
  ros::NodeHandle *nh;

  //! Name of the node
  std::string nodename;

  /*!
   * \brief \c vector of \c pairs with all \c ros::AdvertiseServiceOptions and the pointer to the \c ros::ServiceServer
   * objects created with the help of this class
   * \see Tapi::Publisher::AddFeature
   */
  std::vector<std::pair<ros::AdvertiseServiceOptions, ros::ServiceServer *>> services;
};
}

#endif  // SERVICESERVER_H
