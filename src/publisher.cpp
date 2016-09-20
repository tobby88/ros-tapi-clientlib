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
 * \file publisher.cpp
 * \ingroup tapi_lib
 * \author Tobias Holst
 * \date 28 Aug 2016
 * \brief Definition of the Tapi::Publisher-class and its member functions
 */

#include "include/tapi_lib/publisher.hpp"

using namespace std;

namespace Tapi
{
// Constructor/Destructor

Publisher::Publisher(ros::NodeHandle* nh, string nodename)
  : TapiClient(nh, nodename, PUBLISHER_DEVICE), nh(nh), nodename(nodename)
{
}

Publisher::~Publisher()
{
  for (int i = 0; i < publishers.size(); i++)
  {
    publishers[i]->shutdown();
    delete publishers[i];
  }
}
}
