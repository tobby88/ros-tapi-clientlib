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
 * \file publisher.hpp
 * \ingroup tapi_lib
 * \author Tobias Holst
 * \date 28 Aug 2016
 * \brief Declaration of the Tapi::Publisher-class and definition of its member variables (also part of the definition
 * of member functions because of templates)
 */

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
/*!
 * \brief Create Tapi-compliant publishers with the help of this class, inherits its base functions from
 * Tapi::TapiClient
 * \author Tobias Holst
 * \version 2.1.1
 */
class Publisher : public TapiClient
{
public:
  // Constructor/Destructor

  /*!
   * \brief Create a publisher object as the helper to connect to the core of tapi and create Tapi-compliant publishers
   * \param nh Pointer to a \c ros::NodeHandle created outside of this class
   * \param nodename Name of the device
   */
  Publisher(ros::NodeHandle *nh, std::string nodename = "");

  //! Shutdown all publishers and free the memory
  ~Publisher();

  // Public member functions
  // Templated functions
  template <typename T>
  /*!
   * \brief Create a publisher by adding a feature
   *
   * Template. Gets a saved uuid or generates an new one with the help of Tapi::TapiClient, create a ros::Publisher with
   * the templated type and the device uuid and feature uuid as topic name. Then it stores a pointer to this publisher
   * and the infos about this topic in a Feature message.
   * \param featurename (Descriptive) name of the feature
   * \param queuesize Size of the message queue for this publisher. If there are more messages waiting than this value
   * the messages will be discarded
   * \return Pointer to the publisher
   * \see \c Feature.msg
   * \see Tapi::TapiClient
   * \see Tapi::Publisher::publishers
   * \code{.cpp}
   * // Example code to use the Tapi::Publisher class
   * // nh is a pointer to a ros::NodeHandle generated outside (e.g. in int main())
   * Tapi::Publisher *tpub = new Tapi::Publisher(nh, "Test");
   * // Usage: AddFeature<topic_type>(name, queuesize)
   * ros::Publisher* pub = tpub->AddFeature<std_msgs::Bool>("testbutton", 10);
   * std_msgs::Bool test;
   * test.data = true;
   * pub->publish(test);
   * // Don't delete all ros::Publisher objects, only delete the Tapi::Publisher object,
   * // it will shutdown and delete the publishers in its destructor
   * delete tpub;
   * \endcode
   */
  ros::Publisher *AddFeature(std::string featurename = "", unsigned long queuesize = 1)
  {
    // Get the next UUID (already saved one or new one if not existing yet) and set the topic name
    std::string featureUUID = getNextFeatureUUID();
    ros::Publisher *publisher = 0;
    std::string publisherName = "/Tapi/" + uuid + "/" + featureUUID;

    // Create a publisher of the Template-type with the topic name generated above
    publisher = new ros::Publisher(nh->advertise<T>(publisherName, queuesize));

    // Publisher created successfully
    if (publisher)
    {
      publishers.push_back(publisher);
      tapi_lib::Feature feature;

      // Generate ros::SubscribeOptions with the same type as the publisher to save its message type (only available on
      // Subscribers/SubscribeOptions but not on Publishers/PublishOptions -> do this workaround)
      ros::SubscribeOptions temp;
      temp = ros::SubscribeOptions::create<T>("", 1, NULL, ros::VoidPtr(), NULL);
      feature.FeatureType = temp.datatype;

      // Save name and uuid and save the create message in the featureMsgs vector of Tapi::TapiClient
      feature.Name = featurename;
      feature.UUID = featureUUID;
      featureMsgs.push_back(feature);
    }
    connect();
    return publisher;
  }

private:
  // Private member variables

  //! NodeHandle-pointer necessary to create publishers.
  ros::NodeHandle *nh;

  //! Name of the node
  std::string nodename;

  /*!
   * \brief \c vector of pointers to all \c ros::publisher objects created with the help of this class
   * \see Tapi::Publisher::AddFeature
   */
  std::vector<ros::Publisher *> publishers;
};
}

#endif  // PUBLISHER_H
