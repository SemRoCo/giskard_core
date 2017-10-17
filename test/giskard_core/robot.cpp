/*
 * Copyright (C) 2017 Georg Bartels <georg.bartels@cs.uni-bremen.de>
 * 
 * This file is part of giskard.
 * 
 * giskard is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <gtest/gtest.h>
#include <giskard_core/giskard_core.hpp>


class RobotTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      ASSERT_TRUE(urdf.initFile("pr2.urdf"));
      root_link = "base_link";
      wrong_root_link = "foo";
      tip_links = {"torso_lift_link", "l_wrist_roll_link"};
      empty_tip_links = {};
      wrong_tip_links = {"bar"};
      root_as_only_tip_link = { root_link };
    }

    virtual void TearDown(){}

    urdf::Model urdf;
    std::string root_link, wrong_root_link;
    std::vector<std::string> tip_links, empty_tip_links, wrong_tip_links,
      root_as_only_tip_link;
};

TEST_F(RobotTest, SaneConstructor)
{
  EXPECT_NO_THROW(giskard_core::Robot(urdf, root_link, tip_links));
  EXPECT_NO_THROW(giskard_core::Robot(urdf, root_link, root_as_only_tip_link));
  EXPECT_ANY_THROW(giskard_core::Robot(urdf, wrong_root_link, tip_links));
  EXPECT_NO_THROW(giskard_core::Robot(urdf, root_link, empty_tip_links));
  EXPECT_ANY_THROW(giskard_core::Robot(urdf, root_link, wrong_tip_links));
}

TEST_F(RobotTest, GetEmptyScope)
{
  ASSERT_NO_THROW(giskard_core::Robot(urdf, root_link, empty_tip_links));
  giskard_core::Robot my_robot(urdf, root_link, empty_tip_links);
  ASSERT_NO_THROW(my_robot.get_scope());
  EXPECT_EQ(my_robot.get_scope().size(), 0);
}

TEST_F(RobotTest, ChainJointNames)
{
  // TODO: find a way to directly unit test protected method Robot::chain_joint_names
}

TEST_F(RobotTest, GetScope)
{
  ASSERT_NO_THROW(giskard_core::Robot(urdf, root_link, tip_links));
  giskard_core::Robot my_robot(urdf, root_link, tip_links);
  ASSERT_NO_THROW(my_robot.get_scope());
  EXPECT_EQ(my_robot.get_scope().size(), 2);
  // TODO: complete me
}
