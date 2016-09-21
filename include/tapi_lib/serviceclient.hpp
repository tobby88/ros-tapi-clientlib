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
 * \file serviceclient.hpp
 * \ingroup tapi_lib
 * \author Tobias Holst
 * \date 06 Sep 2016
 * \brief Declaration of the Tapi::ServiceClient-class and definition of its member variables (also part of the
 * definition of member functions because of templates)
 */

#ifndef SERVICECLIENT_H
#define SERVICECLIENT_H

#include <string>
#include <vector>
#include "ros/node_handle.h"
#include "ros/service_client.h"
#include "ros/service_client_options.h"
#include "ros/subscriber.h"
#include "tapi_client.hpp"
#include "tapi_lib/Connection.h"
#include "tapi_lib/Feature.h"

namespace Tapi
{
/*!
 * \brief Create Tapi-compliant serviceclients with the help of this class, inherits its base functions from
 * Tapi::TapiClient
 * \author Tobias Holst
 * \version 2.1.1
 */
class ServiceClient : public TapiClient
{
public:
  // Constructor/Destructor

  /*!
   * \brief Create a serviceclient object as the helper to connect to the core of tapi and create Tapi-compliant
   * serviceclients
   * \param nh Pointer to a \c ros::NodeHandle created outside of this class
   * \param nodename Name of the device
   */
  ServiceClient(ros::NodeHandle *nh, std::string nodename = "");

  //! Shutdown all serviceclients and free the memory
  ~ServiceClient();

  // Public member functions
  // Templated functions
  template <typename T>
  /*!
   * \brief Create a ServiceClient by adding a feature
   *
   * Template. Gets a saved uuid or generates an new one with the help of Tapi::TapiClient, creates
   * ros::ServiceClientOptions with the templated type and the device uuid and feature uuid as topic name. Then it
   * stores a pointer to this options and the infos about this topic/feature in a Feature-message. The ServiceClient
   * itself isn't existing yet! It will be built with the help of the ServiceClientOptions only when there is an active
   * connection, provided by the core.
   * Also by the help of generating AdvertiseServiceOptions the string of the service-type (e.g. tapi_lib/Hello) is
   * extracted and saved in the feature message.
   * \param featurename (Descriptive) name of the feature
   * \return Doublepointer (has to be derefernced twice!) to the ros::ServiceClient
   * \warning When trying to use the ros::ServiceClient always dereference the pointer once first and check if it's a
   * nullpointer! This means there is no connection to any ServiceServer at the moment. Only if it's a valid pointer
   * dereference it twice!
   * \see \c Feature.msg
   * \see Tapi::TapiClient
   * \see Tapi::ServiceClient::clients
   * \code{.cpp}
   * // Example code to use the Tapi::ServiceClient class
   * // nh is a pointer to a ros::NodeHandle generated outside (e.g. in int main())
   * Tapi::ServiceClient* tclient = new Tapi::ServiceClient(nh, "Test");
   * // Usage: AddFeature<service_type>(descriptive_name)
   * ros::ServiceClient** client = tclient->AddFeature<tapi_lib::GetDeviceList>("Devicelist Getter");
   * while(ros::ok()
   * {
   *   // Check whether the ServiceClient is connected
   *   if (*tclient)
   *   {
   *     // Call the service
   *     tapi_lib::GetDeviceList msg;
   *     msg.request.get = true;
   *     if ((*client)->call(msg))
   *     {
   *       // Do something wit the response
   *     }
   *   }
   * }
   * // Don't delete ros::ServiceClient objects, only delete the Tapi::ServiceClient object, it will shutdown and delete
   * // the serviceclients in its destructor
   * delete tclient;
   * \endcode
   */
  ros::ServiceClient **AddFeature(std::string featurename = "")
  {
    // Create empty options for this topic
    std::string noTopic = "";
    serviceName.push_back(noTopic);
    ros::ServiceClientOptions opt;
    ros::M_string empty;
    ros::ServiceClient *client = 0;

    // Get the uuid (or a newly generated one) for this feature
    std::string featureUUID = getNextFeatureUUID();

    // Set an unsed topic name to create the ServiceClientOptions
    std::string temptopic = "/Tapi/" + uuid + "/" + featureUUID;
    opt.init<T>(temptopic, false, empty);

    // Not connected yes -> it should be a NUL-pointer
    ros::ServiceClient **clientptr = 0;

    // Save ServiceClientOptions and the empty ServiceClient in a pair
    std::pair<ros::ServiceClientOptions, ros::ServiceClient *> newEntry;
    newEntry = std::make_pair(opt, client);
    clients.push_back(newEntry);

    // Generate a feature message for the featureMsgs vector of Tapi::TapiClient
    tapi_lib::Feature feature;
    ros::AdvertiseServiceOptions temp;
    // We need to temporarily create AdvertiseServiceOptions so we can get a string of the topic/service type (only
    // available in AdvertiseServiceOptions but not in ServiceClientOptions)
    temp = ros::AdvertiseServiceOptions::create<T>("", NULL, ros::VoidPtr(), NULL);
    std::string temp2 = temp.datatype;
    feature.FeatureType = temp2;
    feature.Name = featurename;
    feature.UUID = featureUUID;
    featureMsgs.push_back(feature);

    // Give the caller of this funtion the pointer to the element in the vector so he can dereference it and see if
    // there currently is a connected service (then he can derefernce twice and use the service)
    clientptr = &clients.at(clients.size() - 1).second;
    connect();
    return clientptr;
  }

private:
  // Private member functions

  /*!
   * \brief Get Config-messages from the core to get commands to which ServiceServer we shall connect.
   *
   * On this topic all commands about connections are published. So in this function we first have to check if there is
   * one (or more) message with our uuid and a feature uuid of us in it. If yes we check who we shall subscribe/connect
   * or if we have to delete our current connection.
   * \param msg The message waiting in the ros message queue where the information about who has to connect to who is
   * stored.
   * \see \c Connection.msg
   */
  void readConfigMsg(const tapi_lib::Connection::ConstPtr &msg);

  // Private member variables

  /*!
   * \brief A \c vector of \c pair objects of \c ros::ServiceClientOptions and \c ros::ServiceClient to hold the
   * information about all serviceclients.
   *
   * The second part of the pair is zero if the feature isn't connected to a \c ServiceServer. If the core tells us to
   * connect, the \c ServiceClient is created with the help of the \c ServiceClientOptions object in the first part of
   * the \c pair.
   * \see Tapi::ServiceClient::AddFeature
   */
  std::vector<std::pair<ros::ServiceClientOptions, ros::ServiceClient *>> clients;

  /*!
   * \brief Subscriber to subscribe the Config-messages topic to get information of who shall connect to who. Listens on
   * /Tapi/Config
   * \see Tapi::ServiceClient::readConfigMsg
   */
  ros::Subscriber configSub;

  //! NodeHandle-pointer necessary to create subscribers and serviceclients.
  ros::NodeHandle *nh;

  //! Name of the node
  std::string nodename;

  /*!
   * \brief \c vector of current connected topics/services. This helps to check if we need to change or just stay, if it
   * didn't change.
   * \see Tapi::ServiceClient::readConfigMsg
   */
  std::vector<std::string> serviceName;
};
}

#endif  // SERVICECLIENT_H
