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

#include <list>
#include <boost/foreach.hpp>

PaControl::PaControl() :  node_handle_("~"),
                          pulse_("pacontrol"),
                          get_mute_service_(node_handle_.advertiseService("get_mute", &PaControl::getMuteCb, this)),
                          set_mute_service_(node_handle_.advertiseService("set_mute", &PaControl::setMuteCb, this)),
                          dyn_reconf_server_(node_handle_),
                          dyn_reconf_callback_(boost::bind(&PaControl::dynReconfCallback, this, _1, _2)),
                          muted_at_start_(false)
{
  dyn_reconf_server_.setCallback(dyn_reconf_callback_);

  if(!device_.empty())
  {
    Device dev = pulse_.get_source(device_);
    pulse_.set_mute(dev, muted_at_start_);
  }
}

PaControl::~PaControl()
{

}

void PaControl::printSources()
{
  std::list<Device> sources = pulse_.get_sources();
  ROS_INFO("Available sources:");
  BOOST_FOREACH(Device dev, sources)
  {
    ROS_INFO_STREAM(dev.index << " \"" << dev.name << "\" \"" << dev.description << "\"");
  }
}

bool PaControl::getMuteCb(pacontrol::GetMute::Request& req, pacontrol::GetMute::Response& res)
{
  bool success = false;
  try
    {
      Device dev = pulse_.get_source(req.device_name);
    res.mute = dev.mute;
    success = true;
  }
  catch(std::string& e)
  {
    ROS_ERROR("Device '%s' not available: %s", req.device_name.c_str(), e.c_str());
    printSources();
  }

  return success;
}

bool PaControl::setMuteCb(pacontrol::SetMute::Request& req, pacontrol::SetMute::Response& res)
{
  bool success = false;

  try
  {
    Device dev = pulse_.get_source(req.device_name);
    ROS_INFO("%s device '%s.", req.mute ? "Mute" : "Unmute", req.device_name.c_str());
    pulse_.set_mute(dev, req.mute);
    res.success = true;
    success = true;
  }
  catch(std::string& e)
  {
    ROS_ERROR("Device '%s' not available: %s", req.device_name.c_str(), e.c_str());
    printSources();
  }

  return success;
}

void PaControl::dynReconfCallback(pacontrol::pacontrolConfig& config, uint32_t level)
{
  if(!config.default_device.empty())
  {
    ROS_INFO("Set device name to '%s'", config.default_device.c_str());
    device_ = config.default_device;
  }

  muted_at_start_ = config.muted_at_start;
}
