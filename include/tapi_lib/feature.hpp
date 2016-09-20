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
 * \file feature.hpp
 * \ingroup tapi_lib
 * \author Tobias Holst
 * \date 20 Nov 2015
 * \brief Declaration of the Tapi::Feature-class and definition of its member variables
 */

#ifndef FEATURE_H
#define FEATURE_H

#include <string>

namespace Tapi
{
/*!
 * \brief Store information about a feature of a device
 * \author Tobias Holst
 * \version 2.1.1
 */
class Feature
{
public:
  // Constructor/Destructor

  /*!
   * \brief Create an object which stores information about a device's feature
   * \param type Type of the feature (topic type like \c "std_msgs/Bool")
   * \param name Name of the feature
   * \param uuid Unique ID of the feature
   */
  Feature(std::string type, std::string name, std::string uuid);

  //! Empty destructor
  ~Feature();

  // Public member functions

  /*!
   * \brief Get the name of the feature
   * \return Name of the feature
   * \see Tapi::Feature::name
   */
  std::string GetName() const;

  /*!
   * \brief Get the type of the feature
   * \return Type of the feature (topic type like \c "std_msgs/Bool")
   * \see Tapi::Feature::type
   */
  std::string GetType() const;

  /*!
   * \brief Get the unique ID of the feature
   * \return Unique ID of the feature
   * \see Tapi::Feature::uuid
   */
  std::string GetUUID() const;

  /*!
   * \brief Override the \c operator \c == to compare two features by \c ==
   * \param other The \c Feature behind the \c ==
   * \return \c true if both features contain the same data in its member variables, \c false if not
   */
  bool operator==(const Feature &other) const;

  /*!
   * \brief Update the data of the Feature
   * \param type Type of the feature (topic type like \c "std_msgs/Bool")
   * \param name Name of the feature
   */
  void Update(std::string type, std::string name);

private:
  // Private member variables

  //! Type of the feature (topic type like \c "std_msgs/Bool")
  std::string type;

  //! Name of the feature
  std::string name;

  //! Unique ID of the feature
  std::string uuid;
};
}

#endif  // FEATURE_H
