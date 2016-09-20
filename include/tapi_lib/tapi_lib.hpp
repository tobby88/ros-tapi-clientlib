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
 * \defgroup tapi_lib tapi_lib
 * \file tapi_lib.hpp
 * \ingroup tapi_lib
 * \author Tobias Holst
 * \date 07 Sep 2016
 * \brief Includes everything of tapi_lib
 */

// Server-Libraries
#include "connection.hpp"
#include "device.hpp"
#include "feature.hpp"

// Client-Libraries
#include "publisher.hpp"
#include "serviceclient.hpp"
#include "serviceserver.hpp"
#include "subscriber.hpp"

// "Helper"-Library
#include "tapi_client.hpp"

// Messages
#include "tapi_lib/Connect.h"
#include "tapi_lib/Connection.h"
#include "tapi_lib/Device.h"
#include "tapi_lib/Feature.h"
#include "tapi_lib/GetConnectionList.h"
#include "tapi_lib/GetDeviceList.h"
#include "tapi_lib/GetStateMachine.h"
#include "tapi_lib/Hello.h"
#include "tapi_lib/State.h"
#include "tapi_lib/Transition.h"
