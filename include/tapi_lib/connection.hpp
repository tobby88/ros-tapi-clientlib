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
 * \file connection.hpp
 * \ingroup tapi_lib
 * \author Tobias Holst
 * \date 28 Jul 2016
 * \brief Declaration of the Tapi::Connection-class and definition of its member variables
 */

#ifndef CONNECTION_H
#define CONNECTION_H

#include <string>

namespace Tapi
{
/*!
 * \brief Store information about a Tapi-connection
 * \author Tobias Holst
 * \version 2.1.1
 */
class Connection
{
public:
  // Constructor/Destructor

  /*!
   * \brief Create a connection object
   * \param publisherUUID Unique ID of the Publisher/ServiceServer device
   * \param publisherFeatureUUID Unique ID of the Publisher/ServiceServer feature
   * \param subscriberUUID Unique ID of the Subscriber/ServiceClient device
   * \param subscriberFeatureUUID Unique ID of the Subscriber/ServiceClient feature
   * \param coefficient Values going over this connection shall be multiplied with this coefficient
   */
  Connection(std::string publisherUUID, std::string publisherFeatureUUID, std::string subscriberUUID,
             std::string subscriberFeatureUUID, double coefficient = 1.0);

  //! Empty destructor
  ~Connection();

  // Public member functions

  /*!
   * \brief Get the coefficient of the connection
   * \return coefficient
   * \see Tapi::Connection::coefficient
   */
  double GetCoefficient();

  /*!
   * \brief Get the unique ID of the connected feature on publisher-/serviceserver-side
   * \return publisherFeatureUUID
   * \see Tapi::Connection::publisherFeatureUUID
   */
  std::string GetPublisherFeatureUUID();

  /*!
   * \brief Get the unique ID of the publisher/serviceserver device
   * \return publisherUUID
   * \see Tapi::Connection::publisherUUID
   */
  std::string GetPublisherUUID();

  /*!
   * \brief Get the unique ID of the connected feature on subscriber-/serviceclient-side
   * \return subscriberFeatureUUID
   * \see Tapi::Connection::subscriberFeatureUUID
   */
  std::string GetSubscriberFeatureUUID();

  /*!
   * \brief Get the unique ID of the subscriber/serviceclient device
   * \return subscriberUUID
   * \see Tapi::Connection::subscriberUUID
   */
  std::string GetSubscriberUUID();

private:
  // Private member variables

  //! Values going over this connection shall be multiplied with this coefficient
  double coefficient;

  //! Unique ID of the Publisher/ServiceServer feature
  std::string publisherFeatureUUID;

  //! Unique ID of the Publisher/ServiceServer device
  std::string publisherUUID;

  //! subscriberFeatureUUID Unique ID of the Subscriber/ServiceClient feature
  std::string subscriberFeatureUUID;

  //! Unique ID of the Subscriber/ServiceClient device
  std::string subscriberUUID;
};
}

#endif  // CONNECTION_H
