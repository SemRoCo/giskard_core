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
      root_link = "base_footprint";
      weights = {{Robot::default_joint_weight_key(), 0.001}, {"torso_lift_joint", 0.01}};
      thresholds = {{Robot::default_joint_velocity_key(), 0.5}, {"torso_lift_joint", 0.01}};
    }

    virtual void TearDown(){}


    urdf::Model urdf;
    std::map<std::string, double> weights, thresholds;
    std::string root_link;

};

TEST_F(WholeBodyControlParamsTest, NoControl)
{
  WholeBodyControlParams params(urdf, root_link, weights, thresholds, {});
  ASSERT_NO_THROW(ControllerSpecGenerator gen(params));
  ControllerSpecGenerator gen(params);
  ASSERT_NO_THROW(gen.get_control_params());
  EXPECT_ANY_THROW(gen.get_goal_inputs(""));
  ASSERT_NO_THROW(gen.get_spec());
  // TODO: complete me
}

TEST_F(WholeBodyControlParamsTest, SingleJointControl)
{
  ControlParams single_joint_params;
  single_joint_params.root_link = "base_link";
  single_joint_params.tip_link = "torso_lift_link";
  single_joint_params.p_gain = 1.0;
  single_joint_params.threshold_error = false;
  single_joint_params.threshold = 0.2;
  single_joint_params.weight = 1.0;
  single_joint_params.type = ControlParams::ControlType::JOINT;
  std::string control_name = "torso_controller";
  WholeBodyControlParams params(urdf, root_link, weights, thresholds, {{control_name, single_joint_params}});
  ASSERT_NO_THROW(ControllerSpecGenerator gen(params));
  ControllerSpecGenerator gen(params);
  ASSERT_NO_THROW(gen.get_control_params());
  ASSERT_NO_THROW(gen.get_goal_inputs(control_name));
  ASSERT_EQ(gen.get_goal_inputs(control_name).size(), 1);
  EXPECT_TRUE(gen.get_goal_inputs(control_name)[0]->equals(*(input(1))));
  ASSERT_NO_THROW(gen.get_spec());
  QPControllerSpec spec = gen.get_spec();
  ASSERT_EQ(spec.controllable_constraints_.size(), 1);
  ASSERT_EQ(spec.hard_constraints_.size(), 1);
  ASSERT_EQ(spec.scope_.size(), 0);
  ASSERT_EQ(spec.soft_constraints_.size(), 1);
  // TODO: more about comparing the soft constraint specs
  ASSERT_NO_THROW(generate(spec));
  // TODO: test that the controller is usable
}

// MULTI-DOF CONTROL

// CARTPOS CONTROL

// CARTROT CONTROL
