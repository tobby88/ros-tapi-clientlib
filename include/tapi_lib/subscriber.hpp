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
 * \file subscriber.hpp
 * \ingroup tapi_lib
 * \author Tobias Holst
 * \date 29 Aug 2016
 * \brief Declaration of the Tapi::Subscriber-class and definition of its member variables
 */

#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H

#include <string>
#include <vector>
#include "ros/node_handle.h"
#include "ros/subscribe_options.h"
#include "ros/subscriber.h"
#include "tapi_client.hpp"
#include "tapi_lib/Connection.h"

#define SubscribeOptionsForTapi(type, buffer, callfctn)                                                                \
  ros::SubscribeOptions::create<type>("", buffer, boost::bind((callfctn), this, _1), ros::VoidPtr(this), NULL)

namespace Tapi
{
/*!
 * \brief Create Tapi-compliant subscribers with the help of this class, inherits its base functions from
 * Tapi::TapiClient
 * \author Tobias Holst
 * \version 2.1.1
 */
class Subscriber : public TapiClient
{
public:
  // Constructor/Destructor

  /*!
   * \brief Create a Subscriber object as the helper to connect to the core of Tapi and create Tapi-compliant
   * subscribers
   * \param nh Pointer to a \c ros::NodeHandle created outside of this class
   * \param nodename Name of the device
   */
  Subscriber(ros::NodeHandle *nh, std::string nodename = "");

  //! Shutdown all subscribers and free the memory
  ~Subscriber();

  // Public member functions

  /*!
   * \brief Create a subscriber by adding a feature
   *
   * Gets a saved uuid or generates a new one with the help of Tapi::TapiClient. Stores the \c pair of \c
   * ros::SubscribeOptions and a pointer to the created ros::Subscriber in the \c subscribers \c vector. The
   * SubscriberOptions should be created with the help of the \c SubscribeOptionsForTapi \c macro. At last the
   * informations are stored in the Feature-messages \c vector of Tapi::TapiClient.
   * \param opt Options to create the subscriber, easiest way is to use \c SubscribeOptionsForTapi \c macro
   * \param featurename Descriptive) name of the feature
   * \return Pointer to the coefficient of the connection
   * \see Tapi::TapiClient
   * \see Tapi::Subscriber::subscribers
   * \see \c Feature.msg
   * \see Tapi::Subscriber::coefficients
   * \code{.cpp}
   * // Example code to use the Tapi::Subscriber class
   * // nh is a pointer to a ros::NodeHandle generated outside (e.g. in int main())
   * Tapi::Subscriber* tsub = new Tapi::Subscriber(nh, "Test");
   * // Option 1 (longer, but maybe easier to read)
   * ros::SubscribeOptions opt;
   * double* coefficient;
   * // Usage: SubscribeOptionsForTapi(topic_type, queue_lenght, &callback_function)
   * opt = SubscribeOptionsForTapi(std_msgs::Float64, 1, &My_Classname::myFloatCallback);
   * coefficient = tsub->AddFeature(opt, "Double/Float64");
   * // Option 2 (shorter, but maybe harder to read)
   * coefficient = tsub->AddFeature(SubscribeOptionsForTapi(std_msgs::Float64, 1, &My_Classname::myFloatCallback),
   * "Double/Float64");
   *
   * // In the callback function now don't forget to multiply with the coeffeicient, e.g.:
   * void My_Classname::myFloatCallback(const std_msgs::Float64::ConstPtr& msg)
   * {
   *   double value = msg->data * (*coefficient);
   * }
   *
   * // Don't delete \c ros::Subscriber objects, only delete the Tapi::Subscriber object,
   * // it will shutdown and delete the subscribers in its destructor
   * delete tsub;
   * \endcode
   */
  double *AddFeature(ros::SubscribeOptions opt, std::string featurename = "");

private:
  // Private member functions

  /*!
   * \brief Get Config-messages from the core to get commands to which Publisher we shall connect.
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
   * \brief coefficients of our subscribers
   *
   * Stores with which values our received messages have to be multiplied. Only valid for basic number types (e.g
   * std_msgs/Int8, std_msgs/Int16, std_msgs/Float32, ...)
   * \see Tapi::Subscriber::readConfigMsg
   */
  std::vector<double *> coefficients;

  /*!
   * \brief Subscriber to subscribe the Config-messages topic to get information of who shall connect to who. Listens on
   * /Tapi/Config
   * \see Tapi::Subscriber::readConfigMsg
   */
  ros::Subscriber configSub;

  //! NodeHandle-pointer necessary to create subscribers.
  ros::NodeHandle *nh;

  //! Name of the node
  std::string nodename;

  /*!
   * \brief A \c vector of \c pair objects of \c ros::SubscribeOptions and \c ros::Subscriber to hold the information
   * about all subscribers.
   *
   * The second part of the pair is zero if the \c Subscriber isn't connected to a \c Publisher. If the core tells us to
   * connect, the \c Subscriber is created with the help of the \c SubscribeOptions object in the first part of the \c
   * pair.
   * \see Tapi::Subscriber::AddFeature
   */
  std::vector<std::pair<ros::SubscribeOptions, ros::Subscriber *>> subscribers;

  /*!
   * \brief \c vector of current connected topics. This helps to check if we need to change or just stay, if it didn't
   * change.
   * \see Tapi::Subscriber::readConfigMsg
   */
  std::vector<std::string> topicNames;
};
}

#endif  // SUBSCRIBER_H
