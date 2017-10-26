//
// Created by georg on 25.10.17.
//

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

using namespace giskard_core;

class WholeBodyControlParamsTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
      ASSERT_TRUE(urdf.initFile("pr2.urdf"));
      root_link = "base_link";
      tip_links = {"torso_lift_link", "r_gripper_tool_frame"};
      weights = {{"default-joint-weight", 0.001}, {"torso_lift_joint", 0.01}};
      thresholds = {{"default-joint-velocity", 0.5}, {"torso_lift_joint", 0.01}};
    }

    virtual void TearDown(){}


    urdf::Model urdf;
    std::map<std::string, double> weights, thresholds;
    std::vector<std::string> tip_links;
    std::string root_link;

};

TEST_F(WholeBodyControlParamsTest, NoControl)
{
  ASSERT_NO_THROW(Robot(urdf, root_link, tip_links, weights, thresholds));
}