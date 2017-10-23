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

class TestRobot : public giskard_core::Robot
{
  public:
     TestRobot(const urdf::Model& robot_model, const std::string& root_link,
          const std::vector<std::string>& tip_links) :
       giskard_core::Robot(robot_model, root_link, tip_links) {}

     std::vector<std::string> test_chain_joint_names(const std::string& root, 
        const std::string& tip, bool add_fixed_joints=true)
    {
      return chain_joint_names(root, tip, add_fixed_joints);
    }
};

class RobotTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      ASSERT_TRUE(urdf.initFile("pr2.urdf"));
      root_link = "base_link";
      tip_link = "l_gripper_tool_frame";
      wrong_root_link = "foo";
      tip_links = {"torso_lift_link", "l_wrist_roll_link"};
      empty_tip_links = {};
      wrong_tip_links = {"bar"};
      root_as_only_tip_link = { root_link };
      moveable_joints_names = {
        "torso_lift_joint", "l_shoulder_pan_joint", "l_shoulder_lift_joint",
        "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint",
        "l_wrist_flex_joint", "l_wrist_roll_joint"};
      all_joint_names = {
        "torso_lift_joint", "l_shoulder_pan_joint", "l_shoulder_lift_joint", 
        "l_upper_arm_roll_joint", "l_upper_arm_joint", "l_elbow_flex_joint", 
        "l_forearm_roll_joint", "l_forearm_joint", "l_wrist_flex_joint", 
        "l_wrist_roll_joint", "l_gripper_palm_joint", "l_gripper_tool_joint"};
    }

    virtual void TearDown(){}

    urdf::Model urdf;
    std::string root_link, tip_link, wrong_root_link;
    std::vector<std::string> tip_links, empty_tip_links, wrong_tip_links,
      root_as_only_tip_link, moveable_joints_names, all_joint_names;
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

TEST_F(RobotTest, GetJointEmpty)
{
  ASSERT_NO_THROW(giskard_core::Robot(urdf, root_link, empty_tip_links));
  giskard_core::Robot my_robot(urdf, root_link, empty_tip_links);
  EXPECT_EQ(0, my_robot.get_number_of_joints());
  for (size_t i=0; i<all_joint_names.size(); ++i)
    EXPECT_ANY_THROW(my_robot.get_joint(all_joint_names[i]));
}

TEST_F(RobotTest, GetJointAll)
{
  ASSERT_NO_THROW(giskard_core::Robot(urdf, root_link, tip_links));
  giskard_core::Robot my_robot(urdf, root_link, tip_links);
  EXPECT_EQ(moveable_joints_names.size(), my_robot.get_number_of_joints());
  for (size_t i=0; i<moveable_joints_names.size(); ++i)
    ASSERT_NO_THROW(my_robot.get_joint(moveable_joints_names[i]));
}

TEST_F(RobotTest, ChainJointNames)
{
  ASSERT_NO_THROW(TestRobot(urdf, root_link, empty_tip_links));
  TestRobot robot(urdf, root_link, empty_tip_links);
  ASSERT_NO_THROW(robot.test_chain_joint_names(root_link, tip_link, false));
  std::vector<std::string> joint_names = 
    robot.test_chain_joint_names(root_link, tip_link, false);
  EXPECT_EQ(moveable_joints_names.size(), joint_names.size());
  for (size_t i=0; i<joint_names.size(); ++i)
    EXPECT_STREQ(moveable_joints_names[i].c_str(), joint_names[i].c_str());
  ASSERT_NO_THROW(robot.test_chain_joint_names(root_link, tip_link, true));
  joint_names = robot.test_chain_joint_names(root_link, tip_link, true);
  EXPECT_EQ(all_joint_names.size(), joint_names.size());
  for (size_t i=0; i<joint_names.size(); ++i)
    EXPECT_STREQ(all_joint_names[i].c_str(), joint_names[i].c_str());
}

TEST_F(RobotTest, GetScope)
{
  ASSERT_NO_THROW(giskard_core::Robot(urdf, root_link, tip_links));
  giskard_core::Robot my_robot(urdf, root_link, tip_links);
  ASSERT_NO_THROW(my_robot.get_scope());
  EXPECT_EQ(my_robot.get_scope().size(), 2);
  // TODO: complete me
}
