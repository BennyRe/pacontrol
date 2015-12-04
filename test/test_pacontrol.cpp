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

#include <gtest/gtest.h>
#include "ros/ros.h"
#include "pacontrol/GetMute.h"
#include "pacontrol/SetMute.h"

TEST(PaControlTestSuite, testServices)
{
  const std::string DEVICE_NAME = "alsa_output.pci-0000_00_1b.0.analog-stereo.monitor";

  ros::NodeHandle n;
  ros::ServiceClient get_mute_client = n.serviceClient<pacontrol::GetMute>("pacontrol/get_mute");
  ros::ServiceClient set_mute_client = n.serviceClient<pacontrol::SetMute>("pacontrol/set_mute");

  pacontrol::GetMute get_mute_srv;
  pacontrol::SetMute set_mute_srv;

  get_mute_srv.request.device_name = DEVICE_NAME;
  set_mute_srv.request.device_name = DEVICE_NAME;

  // get the initial mute state of the device
  ASSERT_TRUE(get_mute_client.call(get_mute_srv))
    << "Calling the get mute client with a valid device the first time failed!";

  bool original_mute_state = get_mute_srv.response.mute;

  // test set mute
  set_mute_srv.request.mute = true;
  ASSERT_TRUE(set_mute_client.call(set_mute_srv))
      << "Muting a valid device the first time failed!";
  EXPECT_TRUE(set_mute_srv.response.success) << "Muting returned unsuccessfully";

  // did it work?
  ASSERT_TRUE(get_mute_client.call(get_mute_srv));
  EXPECT_TRUE(get_mute_srv.response.mute) << "After muting the device should be muted.";

  // unmute
  set_mute_srv.request.mute = false;
  ASSERT_TRUE(set_mute_client.call(set_mute_srv));
  EXPECT_TRUE(set_mute_srv.response.success) << "Unmuting returned unsuccessfully";

  // did it work?
  ASSERT_TRUE(get_mute_client.call(get_mute_srv));
  EXPECT_FALSE(get_mute_srv.response.mute) << "After unmuting the device should be unmuted.";

  // restore original state
  set_mute_srv.request.mute = original_mute_state;
  ASSERT_TRUE(set_mute_client.call(set_mute_srv));
  EXPECT_TRUE(set_mute_srv.response.success);

  // did it work?
  ASSERT_TRUE(get_mute_client.call(get_mute_srv));
  EXPECT_EQ(original_mute_state, get_mute_srv.response.mute);
}

int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
ros::init(argc, argv, "test_pacontrol");

return RUN_ALL_TESTS();
}
