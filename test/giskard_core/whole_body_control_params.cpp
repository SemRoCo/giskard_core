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

TEST_F(WholeBodyControlParamsTest, TorsoLiftJointControl)
{
  // prepare necessary data
  using Eigen::operator<<;
  Eigen::VectorXd state;
  state.resize(2);
  state << 0.2, 0.05; // start joint state and goal joint state for torso joint
  int nWSR = 20;
  ControlParams single_joint_params;
  single_joint_params.root_link = "base_link";
  single_joint_params.tip_link = "torso_lift_link";
  single_joint_params.p_gain = 1.7;
  single_joint_params.threshold_error = false;
  single_joint_params.threshold = 0.2;
  single_joint_params.weight = 1.0;
  single_joint_params.type = ControlParams::ControlType::JOINT;
  std::string control_name = "torso_controller";
  std::string joint_name = "torso_lift_joint";
  std::string autgen_name = control_name + "_" + joint_name;
  WholeBodyControlParams params(urdf, root_link, weights, thresholds, {{control_name, single_joint_params}});
  // check that spec generation is ok
  ASSERT_NO_THROW(ControllerSpecGenerator gen(params));
  ControllerSpecGenerator gen(params);
  ASSERT_NO_THROW(gen.get_control_params());
  ASSERT_NO_THROW(gen.get_goal_inputs(control_name));
  ASSERT_EQ(gen.get_goal_inputs(control_name).size(), 1);
  ASSERT_TRUE(gen.get_goal_inputs(control_name).find(joint_name) != gen.get_goal_inputs(control_name).end());
  EXPECT_TRUE(gen.get_goal_inputs(control_name).find(joint_name)->second->equals(*(input(1))));
  ASSERT_NO_THROW(gen.get_spec());
  QPControllerSpec spec = gen.get_spec();
  ASSERT_EQ(spec.controllable_constraints_.size(), 1);
  ASSERT_EQ(spec.hard_constraints_.size(), 1);
  ASSERT_EQ(spec.scope_.size(), 0);
  ASSERT_EQ(spec.soft_constraints_.size(), 1);
  EXPECT_STREQ(spec.soft_constraints_[0].name_.c_str(), autgen_name.c_str());
  EXPECT_TRUE(spec.soft_constraints_[0].weight_->equals(*(double_const_spec(single_joint_params.weight))));
  EXPECT_TRUE(spec.soft_constraints_[0].expression_->equals(*(input(0))));
  EXPECT_TRUE(spec.soft_constraints_[0].lower_->equals(*(spec.soft_constraints_[0].upper_)));
  if (single_joint_params.threshold_error)
    EXPECT_TRUE(false);
  else
    EXPECT_TRUE(spec.soft_constraints_[0].lower_->equals(*(double_mul_spec({double_const_spec(single_joint_params.p_gain),
                                                                       double_sub_spec({input(1), input(0)})}))));
  // check that resulting controller is ok
  ASSERT_NO_THROW(generate(spec));
  QPController control = generate(spec);
  ASSERT_TRUE(control.start(state, nWSR));
  for (size_t i=0; i<15; ++i)
  {
    ASSERT_TRUE(control.update(state, nWSR));
    ASSERT_EQ(control.get_command().rows(), 1);
    EXPECT_DOUBLE_EQ(control.get_command()[0], -thresholds["torso_lift_joint"]); // commanding max velocity
    state[0] += control.get_command()[0]; // simulating kinematics
  }
  EXPECT_DOUBLE_EQ(state[0], state[1]); // goal reached
}
TEST_F(WholeBodyControlParamsTest, LWristRollJoint)
{
  // prepare necessary data
  using Eigen::operator<<;
  Eigen::VectorXd state;
  state.resize(2);
  state << -22, 0.1; // start joint state and goal joint state for joint
  int nWSR = 20;
  ControlParams single_joint_params;
  single_joint_params.root_link = "l_wrist_flex_link";
  single_joint_params.tip_link = "l_wrist_roll_link";
  single_joint_params.p_gain = 1;
  single_joint_params.threshold_error = false;
  single_joint_params.threshold = 0.2;
  single_joint_params.weight = 1.0;
  single_joint_params.type = ControlParams::ControlType::JOINT;
  std::string control_name = "arm_controller";
  std::string joint_name = "l_wrist_roll_joint";
  std::string autgen_name = control_name + "_" + joint_name;
  WholeBodyControlParams params(urdf, root_link, weights, thresholds, {{control_name, single_joint_params}});
  // check that spec generation is ok
  ASSERT_NO_THROW(ControllerSpecGenerator gen(params));
  ControllerSpecGenerator gen(params);
  ASSERT_NO_THROW(gen.get_control_params());
  ASSERT_NO_THROW(gen.get_goal_inputs(control_name));
  ASSERT_EQ(gen.get_goal_inputs(control_name).size(), 1);
  ASSERT_TRUE(gen.get_goal_inputs(control_name).find(joint_name) != gen.get_goal_inputs(control_name).end());
  EXPECT_TRUE(gen.get_goal_inputs(control_name).find(joint_name)->second->equals(*(input(1))));
  ASSERT_NO_THROW(gen.get_spec());
  QPControllerSpec spec = gen.get_spec();
  ASSERT_EQ(spec.controllable_constraints_.size(), 1);
  ASSERT_EQ(spec.hard_constraints_.size(), 1);
  ASSERT_EQ(spec.scope_.size(), 0);
  ASSERT_EQ(spec.soft_constraints_.size(), 1);
  EXPECT_STREQ(spec.soft_constraints_[0].name_.c_str(), autgen_name.c_str());
  EXPECT_TRUE(spec.soft_constraints_[0].weight_->equals(*(double_const_spec(single_joint_params.weight))));
  EXPECT_TRUE(spec.soft_constraints_[0].expression_->equals(*(input(0))));
  EXPECT_TRUE(spec.soft_constraints_[0].lower_->equals(*(spec.soft_constraints_[0].upper_)));
  if (single_joint_params.threshold_error)
    EXPECT_TRUE(false);
  else
    EXPECT_TRUE(spec.soft_constraints_[0].lower_->equals(*(double_mul_spec({double_const_spec(single_joint_params.p_gain),
                                                                       double_sub_spec({input(1), input(0)})}))));
  // check that resulting controller is ok
  ASSERT_NO_THROW(generate(spec));
  QPController control = generate(spec);
  ASSERT_TRUE(control.start(state, nWSR));
  for (size_t i=0; i<15; ++i)
  {
    ASSERT_TRUE(control.update(state, nWSR));
    ASSERT_EQ(control.get_command().rows(), 1);
    EXPECT_DOUBLE_EQ(control.get_command()[0], -thresholds[Robot::default_joint_velocity_key()]); // commanding max velocity
    state[0] += control.get_command()[0]; // simulating kinematics
  }
  EXPECT_DOUBLE_EQ(state[0], state[1]); // goal reached
}

// MULTI-DOF CONTROL

// CARTPOS CONTROL

// CARTROT CONTROL
