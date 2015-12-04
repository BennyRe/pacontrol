/*
 * Copyright (C) 2015 Benjamin Reiner <reiner@hs-weingarten.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "pacontrol/pacontrol.h"

PaControl::PaControl() :  node_handle_("~"),
                          pulse_("pacontrol"),
                          sources_(pulse_.get_sources()),
                          get_mute_service_(node_handle_.advertiseService("get_mute", &PaControl::getMuteCb, this)),
                          set_mute_service_(node_handle_.advertiseService("set_mute", &PaControl::setMuteCb, this))
{
  // TODO Auto-generated constructor stub

}

PaControl::~PaControl()
{
  // TODO Auto-generated destructor stub
}

void PaControl::printSources()
{
  std::list<Device>::iterator it;
  ROS_INFO("Available sources:");
  for (it = sources_.begin(); it != sources_.end(); ++it)
  {
    ROS_INFO_STREAM(it->index << " \"" << it->name << "\" \"" << it->description << "\"");
  }
}

bool PaControl::getMuteCb(pacontrol::GetMute::Request& req, pacontrol::GetMute::Response& res)
{
  return false;
}

bool PaControl::setMuteCb(pacontrol::SetMute::Request& req, pacontrol::SetMute::Response& res)
{
  return false;
}
