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
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

using namespace giskard_core;

class QPControllerSpecGeneratorTest : public ::testing::Test
{
protected:
    // TODO: move these conversions to somewhere usable
    Eigen::Vector3d to_eigen(const KDL::Vector& p)
    {
      Eigen::Vector3d result;
      result[0] = p.x();
      result[1] = p.y();
      result[2] = p.z();
      return result;
    }

    Eigen::Vector4d to_eigen(const KDL::Rotation& M)
    {
      Eigen::Vector4d result;
      KDL::Vector vec = M.GetRot();
      result[0] = vec.x();
      result[1] = vec.y();
      result[2] = vec.z();
      result[3] = vec.Norm();
      return result;
    }

    typedef Eigen::Matrix< double, 7, 1 > Vector7d;

    Vector7d to_eigen(const KDL::Frame& f)
    {
      Vector7d result;
      result.segment(0, 4) = to_eigen(f.M);
      result.segment(4, 3) = to_eigen(f.p);
      return result;
    }

    virtual void SetUp()
    {
      ASSERT_TRUE(urdf.initFile("pr2.urdf"));
      ASSERT_TRUE(kdl_parser::treeFromUrdfModel(urdf, tree));
      root_link = "base_footprint";
      weights = {{Robot::default_joint_weight_key(), 0.001}, {"torso_lift_joint", 0.01}};
      thresholds = {{Robot::default_joint_velocity_key(), 0.5}, {"torso_lift_joint", 0.01}};
    }

    virtual void TearDown(){}

    urdf::Model urdf;
    KDL::Tree tree;
    std::map<std::string, double> weights, thresholds;
    std::string root_link;

};

TEST_F(QPControllerSpecGeneratorTest, NoControl)
{
  QPControllerParams params(urdf, root_link, weights, thresholds, {});
  ASSERT_NO_THROW(QPControllerSpecGenerator gen(params));
  QPControllerSpecGenerator gen(params);
  ASSERT_NO_THROW(gen.get_control_params());
  EXPECT_ANY_THROW(gen.get_goal_inputs(""));
  ASSERT_NO_THROW(gen.get_spec());
  // TODO: complete me
}

// REGULAR 1-DOF CONTROL
TEST_F(QPControllerSpecGeneratorTest, TorsoLiftJointControl)
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
  single_joint_params.type = ControlParams::ControlType::Joint;
  std::string control_name = "torso_controller";
  std::string joint_name = "torso_lift_joint";
  std::string autgen_name = create_input_name(control_name, joint_name);
  QPControllerParams params(urdf, root_link, weights, thresholds, {{control_name, single_joint_params}});
  // check that spec generation is ok
  ASSERT_NO_THROW(QPControllerSpecGenerator gen(params));
  QPControllerSpecGenerator gen(params);
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
  ASSERT_EQ(1, gen.get_controllable_names().size());
  EXPECT_STREQ(joint_name.c_str(), gen.get_controllable_names()[0].c_str());
  ASSERT_EQ(2, gen.get_observable_names().size());
  EXPECT_STREQ(joint_name.c_str(), gen.get_observable_names()[0].c_str());
  EXPECT_STREQ(autgen_name.c_str(), gen.get_observable_names()[1].c_str());
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

// LIMIT-LESS 1-DOF CONTROL
TEST_F(QPControllerSpecGeneratorTest, LWristRollJoint)
{
  // prepare necessary data
  using Eigen::operator<<;
  Eigen::VectorXd state;
  state.resize(2);
  state << 2.0*QPControllerSpecGenerator::pi() - 0.9, 0.1; // start joint state and goal joint state for joint
  int nWSR = 20;
  ControlParams single_joint_params;
  single_joint_params.root_link = "l_wrist_flex_link";
  single_joint_params.tip_link = "l_wrist_roll_link";
  single_joint_params.p_gain = 1;
  single_joint_params.threshold_error = false;
  single_joint_params.threshold = 0.2;
  single_joint_params.weight = 1.0;
  single_joint_params.type = ControlParams::ControlType::Joint;
  std::string control_name = "arm_controller";
  std::string joint_name = "l_wrist_roll_joint";
  std::string autgen_name = create_input_name(control_name, joint_name);
  QPControllerParams params(urdf, single_joint_params.root_link, weights, thresholds, {{control_name, single_joint_params}});
  // check that spec generation is ok
  ASSERT_NO_THROW(QPControllerSpecGenerator gen(params));
  QPControllerSpecGenerator gen(params);
  ASSERT_NO_THROW(gen.get_control_params());
  ASSERT_NO_THROW(gen.get_goal_inputs(control_name));
  ASSERT_EQ(gen.get_goal_inputs(control_name).size(), 1);
  ASSERT_TRUE(gen.get_goal_inputs(control_name).find(joint_name) != gen.get_goal_inputs(control_name).end());
  EXPECT_TRUE(gen.get_goal_inputs(control_name).find(joint_name)->second->equals(*(input(1))));
  ASSERT_NO_THROW(gen.get_spec());
  QPControllerSpec spec = gen.get_spec();
  ASSERT_EQ(spec.controllable_constraints_.size(), 1);
  ASSERT_EQ(spec.hard_constraints_.size(), 0);
  ASSERT_EQ(spec.scope_.size(), 0);
  ASSERT_EQ(spec.soft_constraints_.size(), 1);
  EXPECT_STREQ(spec.soft_constraints_[0].name_.c_str(), autgen_name.c_str());
  EXPECT_TRUE(spec.soft_constraints_[0].weight_->equals(*(double_const_spec(single_joint_params.weight))));
  EXPECT_TRUE(spec.soft_constraints_[0].expression_->equals(*(input(0))));
  EXPECT_TRUE(spec.soft_constraints_[0].lower_->equals(*(spec.soft_constraints_[0].upper_)));
  ASSERT_EQ(1, gen.get_controllable_names().size());
  EXPECT_STREQ(joint_name.c_str(), gen.get_controllable_names()[0].c_str());
  ASSERT_EQ(2, gen.get_observable_names().size());
  EXPECT_STREQ(joint_name.c_str(), gen.get_observable_names()[0].c_str());
  EXPECT_STREQ(autgen_name.c_str(), gen.get_observable_names()[1].c_str());
  // check that resulting controller is ok
  ASSERT_NO_THROW(generate(spec));
  QPController control = generate(spec);
  ASSERT_TRUE(control.start(state, nWSR));
  for (size_t i=0; i<2; ++i)
  {
    ASSERT_TRUE(control.update(state, nWSR));
    ASSERT_EQ(control.get_command().rows(), 1);
    EXPECT_NEAR(control.get_command()[0], thresholds[Robot::default_joint_velocity_key()], 0.001); // commanding max velocity
    state[0] += control.get_command()[0]; // simulating kinematics
  }
  EXPECT_NEAR(state[0], 2.0*QPControllerSpecGenerator::pi() +state[1], 0.001); // goal reached
}

// MULTI-DOF CONTROL
TEST_F(QPControllerSpecGeneratorTest, RightArmJoint)
{
  // prepare necessary data

  int nWSR = 100;
  ControlParams single_joint_params;
  single_joint_params.root_link = "torso_lift_link";
  single_joint_params.tip_link = "r_gripper_tool_frame";
  single_joint_params.p_gain = 1;
  single_joint_params.threshold_error = false;
  single_joint_params.threshold = 0.2;
  single_joint_params.weight = 1.0;
  single_joint_params.type = ControlParams::ControlType::Joint;
  std::string control_name = "right_arm_controller";
  std::vector<std::string> joint_names = {
        "torso_lift_joint", "r_shoulder_pan_joint", "r_shoulder_lift_joint","r_upper_arm_roll_joint",
        "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"};
  std::vector<std::string> controlled_joint_names = {
        "r_shoulder_pan_joint", "r_shoulder_lift_joint","r_upper_arm_roll_joint",
        "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"};
  std::set<std::string> limitless_joints = {"r_forearm_roll_joint", "r_wrist_roll_joint"};
  QPControllerParams params(urdf, root_link, weights, thresholds, {{control_name, single_joint_params}});
  using Eigen::operator<<;
  Eigen::VectorXd state;
  state.resize(joint_names.size() + controlled_joint_names.size());
  state << 0.1, 0.2,  0.4,  0.6,  -0.8,  1.0,  -1.2,  1.4, // start joint states
               -0.7, -0.5, -0.5, -0.4, -0.3, -0.2, -0.1; // goal joint states
  // check that spec generation is ok
  ASSERT_NO_THROW(QPControllerSpecGenerator gen(params));
  QPControllerSpecGenerator gen(params);
  ASSERT_NO_THROW(gen.get_control_params());
  ASSERT_NO_THROW(gen.get_goal_inputs(control_name));
  ASSERT_EQ(gen.get_goal_inputs(control_name).size(), controlled_joint_names.size());
  for (size_t i=0; i<controlled_joint_names.size(); ++i)
  {
    ASSERT_TRUE(gen.get_goal_inputs(control_name).find(controlled_joint_names[i]) != gen.get_goal_inputs(control_name).end());
    EXPECT_TRUE(gen.get_goal_inputs(control_name).find(controlled_joint_names[i])->second->equals(*(input(joint_names.size() + i))));
  }
  ASSERT_NO_THROW(gen.get_spec());
  QPControllerSpec spec = gen.get_spec();
  ASSERT_EQ(spec.controllable_constraints_.size(), joint_names.size());
  ASSERT_EQ(spec.hard_constraints_.size(), joint_names.size() - limitless_joints.size());
  ASSERT_EQ(spec.scope_.size(), 0);
  ASSERT_EQ(spec.soft_constraints_.size(), controlled_joint_names.size());
  for (size_t i=0; i<controlled_joint_names.size(); ++i)
  {
    std::string autogen_name = create_input_name(control_name, controlled_joint_names[i]);
    EXPECT_STREQ(spec.soft_constraints_[i].name_.c_str(), autogen_name.c_str());
    EXPECT_TRUE(spec.soft_constraints_[i].weight_->equals(*(double_const_spec(single_joint_params.weight))));
    EXPECT_TRUE(spec.soft_constraints_[i].expression_->equals(*(input(i+1))));
    EXPECT_TRUE(spec.soft_constraints_[i].lower_->equals(*(spec.soft_constraints_[i].upper_)));
  }

  ASSERT_EQ(joint_names.size(), gen.get_controllable_names().size());
  ASSERT_EQ(joint_names.size() + controlled_joint_names.size(), gen.get_observable_names().size());
  for (size_t i=0; i<joint_names.size(); ++i)
  {
    EXPECT_STREQ(joint_names[i].c_str(), gen.get_controllable_names()[i].c_str());
    EXPECT_STREQ(joint_names[i].c_str(), gen.get_observable_names()[i].c_str());
  }
  for (size_t i=0; i<controlled_joint_names.size(); ++i)
    EXPECT_STREQ(create_input_name(control_name, controlled_joint_names[i]).c_str(),
              gen.get_observable_names()[joint_names.size() + i].c_str());

  // check that resulting controller is ok
  ASSERT_NO_THROW(generate(spec));
  QPController control = generate(spec);
  EXPECT_TRUE(control.start(state, nWSR));

  for (size_t i=0; i<4; ++i) {
    ASSERT_TRUE(control.update(state, nWSR));
    ASSERT_EQ(control.get_command().rows(), joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i)
      state[i] += control.get_command()[i]; // simulating kinematics
  }


  for (size_t i=0; i<controlled_joint_names.size(); ++i)
    EXPECT_NEAR(state[1+i], state[joint_names.size() + i], 0.0001); // goal reached
}

// Translation3D CONTROL
TEST_F(QPControllerSpecGeneratorTest, LeftArmTranslation3D)
{
  // prepare necessary data
  int nWSR = 100;
  ControlParams single_joint_params;
  single_joint_params.root_link = root_link;
  single_joint_params.tip_link = "l_gripper_tool_frame";
  single_joint_params.p_gain = 1;
  single_joint_params.threshold_error = true;
  single_joint_params.threshold = 0.05;
  single_joint_params.weight = 1.0;
  single_joint_params.type = ControlParams::ControlType::Translation3D;
  std::string control_name = "left_arm_controller";
  std::vector<std::string> joint_names = {
        "torso_lift_joint", "l_shoulder_pan_joint", "l_shoulder_lift_joint","l_upper_arm_roll_joint",
        "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"};
  std::set<std::string> limitless_joints = {"r_forearm_roll_joint", "r_wrist_roll_joint"};
  QPControllerParams params(urdf, root_link, weights, thresholds, {{control_name, single_joint_params}});
  std::map<std::string, double> q_map;
  q_map["l_elbow_flex_joint"] = -0.150245;
  q_map["l_forearm_roll_joint"] = 2.05374;
  q_map["l_shoulder_lift_joint"] = 1.29519;
  q_map["l_shoulder_pan_joint"] = 1.85677;
  q_map["l_upper_arm_roll_joint"] = 1.49425;
  q_map["l_wrist_flex_joint"] = -0.240708;
  q_map["l_wrist_roll_joint"] = 6.11928;
  q_map["torso_lift_joint"] = 0.300026;
  KDL::Vector goal = KDL::Vector(0.0834884427791, 0.505313166742, 0.176484549611);
  Eigen::VectorXd state;
  state.resize(joint_names.size() + translation3d_names().size());
  for (size_t i=0; i<joint_names.size(); ++i)
    state(i) = q_map.find(joint_names[i])->second;
  state.block<3,1>(joint_names.size(), 0) = to_eigen(goal);
  // check that spec generation is ok
  ASSERT_NO_THROW(QPControllerSpecGenerator gen(params));
  QPControllerSpecGenerator gen(params);
  ASSERT_NO_THROW(gen.get_control_params());
  ASSERT_NO_THROW(gen.get_goal_inputs(control_name));
  ASSERT_EQ(gen.get_goal_inputs(control_name).size(), translation3d_names().size());
  for (size_t i=0; i<translation3d_names().size(); ++i)
  {
    ASSERT_TRUE(gen.get_goal_inputs(control_name).find(translation3d_names()[i]) !=
                        gen.get_goal_inputs(control_name).end());
    EXPECT_TRUE(gen.get_goal_inputs(control_name).find(translation3d_names()[i])->second->equals(*(input(joint_names.size() + i))));
  }
  ASSERT_NO_THROW(gen.get_spec());
  QPControllerSpec spec = gen.get_spec();
  ASSERT_EQ(spec.controllable_constraints_.size(), joint_names.size());
  ASSERT_EQ(spec.hard_constraints_.size(), joint_names.size() - limitless_joints.size());
  ASSERT_EQ(spec.scope_.size(), 0);
  ASSERT_EQ(spec.soft_constraints_.size(), translation3d_names().size());
  for (size_t i=0; i<translation3d_names().size(); ++i)
  {
    std::string autogen_name = create_input_name(control_name, translation3d_names()[i]);
    EXPECT_STREQ(spec.soft_constraints_[i].name_.c_str(), autogen_name.c_str());
    EXPECT_TRUE(spec.soft_constraints_[i].weight_->equals(*(double_const_spec(single_joint_params.weight))));
    // TODO: check expression
    // EXPECT_TRUE(spec.soft_constraints_[i].expression_->equals(*(input(i+1))));
    EXPECT_TRUE(spec.soft_constraints_[i].lower_->equals(*(spec.soft_constraints_[i].upper_)));
    // TODO: check upper
  }

  // check that names of controllables and observables are fine
  ASSERT_EQ(joint_names.size(), gen.get_controllable_names().size());
  ASSERT_EQ(joint_names.size() + translation3d_names().size(), gen.get_observable_names().size());
  for (size_t i=0; i<joint_names.size(); ++i)
  {
    EXPECT_STREQ(joint_names[i].c_str(), gen.get_controllable_names()[i].c_str());
    EXPECT_STREQ(joint_names[i].c_str(), gen.get_observable_names()[i].c_str());
  }
  for (size_t i=0; i<translation3d_names().size(); ++i)
    EXPECT_STREQ(create_input_name(control_name, translation3d_names()[i]).c_str(),
              gen.get_observable_names()[joint_names.size() + i].c_str());

  // check that resulting controller is ok
  ASSERT_NO_THROW(generate(spec));
  QPController control = generate(spec);
  ASSERT_TRUE(control.start(state, nWSR));

  for (size_t i=0; i<10; ++i) {
    ASSERT_TRUE(control.update(state, nWSR));
    ASSERT_EQ(control.get_command().rows(), joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i)
      state[i] += control.get_command()[i]; // simulating kinematics
  }

  KDL::Chain chain;
  ASSERT_TRUE(tree.getChain(single_joint_params.root_link, single_joint_params.tip_link, chain));
  ASSERT_EQ(chain.getNrOfJoints(), joint_names.size());

  KDL::ChainFkSolverPos_recursive fk_solver(chain);
  KDL::JntArray q_in(joint_names.size());

  for (size_t i=0; i<joint_names.size(); ++i)
    q_in(i) = state(i);
  KDL::Frame solver_frame;
  ASSERT_GE(fk_solver.JntToCart(q_in, solver_frame), 0);
  EXPECT_TRUE(KDL::Equal(solver_frame.p, goal));
}

// Rotation3D CONTROL
TEST_F(QPControllerSpecGeneratorTest, LeftArmRotation3D)
{
  // prepare necessary parameters
  int nWSR = 100;
  ControlParams single_joint_params;
  single_joint_params.root_link = root_link;
  single_joint_params.tip_link = "l_gripper_tool_frame";
  single_joint_params.p_gain = 1;
  single_joint_params.threshold_error = true;
  single_joint_params.threshold = 0.05;
  single_joint_params.weight = 1.0;
  single_joint_params.type = ControlParams::ControlType::Rotation3D;
  std::string control_name = "left_arm_controller";
  std::vector<std::string> joint_names = {
        "torso_lift_joint", "l_shoulder_pan_joint", "l_shoulder_lift_joint","l_upper_arm_roll_joint",
        "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"};
  std::set<std::string> limitless_joints = {"r_forearm_roll_joint", "r_wrist_roll_joint"};
  QPControllerParams params(urdf, root_link, weights, thresholds, {{control_name, single_joint_params}});

  // prepare initial state and goal
  std::map<std::string, double> q_map;
  q_map["l_elbow_flex_joint"] = -0.150245;
  q_map["l_forearm_roll_joint"] = 2.05374;
  q_map["l_shoulder_lift_joint"] = 1.29519;
  q_map["l_shoulder_pan_joint"] = 1.85677;
  q_map["l_upper_arm_roll_joint"] = 1.49425;
  q_map["l_wrist_flex_joint"] = -0.240708;
  q_map["l_wrist_roll_joint"] = 6.11928;
  q_map["torso_lift_joint"] = 0.300026;
  KDL::Rotation goal = KDL::Rotation::Quaternion(0.293821000071, 0.27801803703, -0.635756695443, 0.657410552874);
  Eigen::VectorXd state;
  state.resize(joint_names.size() + rotation3d_names().size());
  for (size_t i=0; i<joint_names.size(); ++i)
    state(i) = q_map.find(joint_names[i])->second;
  state.segment(joint_names.size(), rotation3d_names().size()) = to_eigen(goal);

  // check that spec generation is ok
  ASSERT_NO_THROW(QPControllerSpecGenerator gen(params));
  QPControllerSpecGenerator gen(params);
  ASSERT_NO_THROW(gen.get_control_params());
  ASSERT_NO_THROW(gen.get_goal_inputs(control_name));
  ASSERT_EQ(gen.get_goal_inputs(control_name).size(), rotation3d_names().size());
  for (size_t i=0; i<rotation3d_names().size(); ++i)
  {
    ASSERT_TRUE(gen.get_goal_inputs(control_name).find(rotation3d_names()[i]) !=
                        gen.get_goal_inputs(control_name).end());
    EXPECT_TRUE(gen.get_goal_inputs(control_name).find(rotation3d_names()[i])->second->equals(*(input(joint_names.size() + i))));
  }
  ASSERT_NO_THROW(gen.get_spec());
  QPControllerSpec spec = gen.get_spec();
  ASSERT_EQ(spec.controllable_constraints_.size(), joint_names.size());
  ASSERT_EQ(spec.hard_constraints_.size(), joint_names.size() - limitless_joints.size());
  ASSERT_EQ(spec.scope_.size(), 0);
  ASSERT_EQ(spec.soft_constraints_.size(), rotation3d_names().size() - 1);
  for (size_t i=0; i<(rotation3d_names().size()-1); ++i)
  {
    std::string autogen_name = create_input_name(control_name, rotation3d_names()[i]);
    EXPECT_STREQ(spec.soft_constraints_[i].name_.c_str(), autogen_name.c_str());
    EXPECT_TRUE(spec.soft_constraints_[i].weight_->equals(*(double_const_spec(single_joint_params.weight))));
    // TODO: check expression
    // EXPECT_TRUE(spec.soft_constraints_[i].expression_->equals(*(input(i+1))));
    EXPECT_TRUE(spec.soft_constraints_[i].lower_->equals(*(spec.soft_constraints_[i].upper_)));
    // TODO: check upper
  }

  // check that names of controllables and observables are ok
  ASSERT_EQ(joint_names.size(), gen.get_controllable_names().size());
  ASSERT_EQ(joint_names.size() + rotation3d_names().size(), gen.get_observable_names().size());
  for (size_t i=0; i<joint_names.size(); ++i)
  {
    EXPECT_STREQ(joint_names[i].c_str(), gen.get_controllable_names()[i].c_str());
    EXPECT_STREQ(joint_names[i].c_str(), gen.get_observable_names()[i].c_str());
  }
  for (size_t i=0; i<rotation3d_names().size(); ++i)
    EXPECT_STREQ(create_input_name(control_name, rotation3d_names()[i]).c_str(),
              gen.get_observable_names()[joint_names.size() + i].c_str());

  // check that resulting controller is ok
  ASSERT_NO_THROW(generate(spec));
  QPController control = generate(spec);
  ASSERT_TRUE(control.start(state, nWSR));

  for (size_t i=0; i<30; ++i) {
    ASSERT_TRUE(control.update(state, nWSR));
    ASSERT_EQ(control.get_command().rows(), joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i)
      state[i] += control.get_command()[i]; // simulating kinematics
  }

  KDL::Chain chain;
  ASSERT_TRUE(tree.getChain(single_joint_params.root_link, single_joint_params.tip_link, chain));
  ASSERT_EQ(chain.getNrOfJoints(), joint_names.size());

  KDL::ChainFkSolverPos_recursive fk_solver(chain);
  KDL::JntArray q_in(joint_names.size());
  for (size_t i=0; i<joint_names.size(); ++i)
    q_in(i) = state(i);
  KDL::Frame solver_frame;
  ASSERT_GE(fk_solver.JntToCart(q_in, solver_frame), 0);
  EXPECT_TRUE(KDL::Equal(solver_frame.M, goal));
}

// Translation3DAndRotation3D
TEST_F(QPControllerSpecGeneratorTest, LeftArmTranslation3DAndRotation3D)
{
  // prepare necessary parameters
  int nWSR = 100;
  ControlParams trans3d_params;
  trans3d_params.root_link = root_link;
  trans3d_params.tip_link = "l_gripper_tool_frame";
  trans3d_params.p_gain = 1;
  trans3d_params.threshold_error = true;
  trans3d_params.threshold = 0.05;
  trans3d_params.weight = 1.0;
  trans3d_params.type = ControlParams::ControlType::Translation3D;
  std::string control_name_trans3d = "left_arm_trans3D";
  std::string control_name_rot3d = "left_arm_rot3D";
  ControlParams rot3d_params = trans3d_params;
  rot3d_params.threshold = 0.1;
  rot3d_params.type = ControlParams::ControlType::Rotation3D;
  QPControllerParams params(urdf, root_link, weights, thresholds,
      {{control_name_trans3d, trans3d_params}, {control_name_rot3d, rot3d_params}});
  std::vector<std::string> joint_names = {
        "torso_lift_joint", "l_shoulder_pan_joint", "l_shoulder_lift_joint","l_upper_arm_roll_joint",
        "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"};
  std::set<std::string> limitless_joints = {"r_forearm_roll_joint", "r_wrist_roll_joint"};

  // prepare initial state and goal state
  std::map<std::string, double> q_map;
  q_map["l_elbow_flex_joint"] = -0.150245;
  q_map["l_forearm_roll_joint"] = 2.05374;
  q_map["l_shoulder_lift_joint"] = 1.29519;
  q_map["l_shoulder_pan_joint"] = 1.85677;
  q_map["l_upper_arm_roll_joint"] = 1.49425;
  q_map["l_wrist_flex_joint"] = -0.240708;
  q_map["l_wrist_roll_joint"] = 6.11928;
  q_map["torso_lift_joint"] = 0.300026;
  KDL::Frame goal = KDL::Frame(KDL::Rotation::Quaternion(0.293821000071, 0.27801803703, -0.635756695443, 0.657410552874),
                               KDL::Vector(0.0834884427791, 0.505313166742, 0.176484549611));
  Eigen::VectorXd state;
  state.resize(joint_names.size() + translation3d_names().size() + rotation3d_names().size());
  for (size_t i=0; i<joint_names.size(); ++i)
    state(i) = q_map.find(joint_names[i])->second;
  state.segment(joint_names.size(), translation3d_names().size() + rotation3d_names().size()) = to_eigen(goal);

  // check that constructor is OK
  ASSERT_NO_THROW(QPControllerSpecGenerator gen(params));
  QPControllerSpecGenerator gen(params);
  ASSERT_NO_THROW(gen.get_control_params());

  // check that names of controllables and observables are ok
  ASSERT_EQ(joint_names.size(), gen.get_controllable_names().size());
  ASSERT_EQ(joint_names.size() + translation3d_names().size() +
            rotation3d_names().size(), gen.get_observable_names().size());
  for (size_t i=0; i<joint_names.size(); ++i)
  {
    EXPECT_STREQ(joint_names[i].c_str(), gen.get_controllable_names()[i].c_str());
    EXPECT_STREQ(joint_names[i].c_str(), gen.get_observable_names()[i].c_str());
  }
  for (size_t i=0; i<rotation3d_names().size(); ++i)
    EXPECT_STREQ(create_input_name(control_name_rot3d,
        rotation3d_names()[i]).c_str(),
        gen.get_observable_names()[joint_names.size() + i].c_str());

  for (size_t i=0; i<translation3d_names().size(); ++i)
    EXPECT_STREQ(create_input_name(control_name_trans3d,
        translation3d_names()[i]).c_str(),
        gen.get_observable_names()[joint_names.size() + rotation3d_names().size() + i].c_str());

  // check that size getters are OK
  EXPECT_EQ(joint_names.size(), gen.num_controllables());
  EXPECT_EQ(translation3d_names().size() + rotation3d_names().size(), gen.num_goal_inputs());
  EXPECT_EQ(gen.num_observables(), gen.num_controllables() + gen.num_goal_inputs());

  // check that goal inputs are OK
  ASSERT_NO_THROW(gen.get_goal_inputs(control_name_rot3d));
  ASSERT_EQ(gen.get_goal_inputs(control_name_rot3d).size(), rotation3d_names().size());
  for (size_t i=0; i<rotation3d_names().size(); ++i)
  {
    ASSERT_NO_THROW(gen.get_goal_input(control_name_rot3d, rotation3d_names()[i]));
    DoubleInputSpecPtr goal_input = gen.get_goal_input(control_name_rot3d, rotation3d_names()[i]);
    EXPECT_EQ(goal_input->get_input_num(), joint_names.size() + i);
    EXPECT_TRUE(gen.get_goal_inputs(control_name_rot3d).find(rotation3d_names()[i])->second->equals(*goal_input));
  }
  ASSERT_NO_THROW(gen.get_goal_inputs(control_name_trans3d));
  ASSERT_EQ(gen.get_goal_inputs(control_name_trans3d).size(), translation3d_names().size());
  for (size_t i=0; i<translation3d_names().size(); ++i)
  {
    ASSERT_NO_THROW(gen.get_goal_input(control_name_trans3d, translation3d_names()[i]));
    DoubleInputSpecPtr goal_input = gen.get_goal_input(control_name_trans3d, translation3d_names()[i]);
    EXPECT_EQ(goal_input->get_input_num(), joint_names.size() + rotation3d_names().size() + i);
    EXPECT_TRUE(gen.get_goal_inputs(control_name_trans3d).find(translation3d_names()[i])->second->equals(*goal_input));
  }

  // check that the actual generation is OK
  ASSERT_NO_THROW(gen.get_spec());
  QPControllerSpec spec = gen.get_spec();
  ASSERT_EQ(spec.controllable_constraints_.size(), joint_names.size());
  ASSERT_EQ(spec.hard_constraints_.size(), joint_names.size() - limitless_joints.size());
  ASSERT_EQ(spec.scope_.size(), 0);
  ASSERT_EQ(spec.soft_constraints_.size(), translation3d_names().size() + rotation3d_names().size() - 1);
  for (size_t i=0; i<(rotation3d_names().size()-1); ++i)
  {
    std::string autogen_name = create_input_name(control_name_rot3d, rotation3d_names()[i]);
    EXPECT_STREQ(spec.soft_constraints_[i].name_.c_str(), autogen_name.c_str());
    EXPECT_TRUE(spec.soft_constraints_[i].weight_->equals(*(double_const_spec(trans3d_params.weight))));
    // TODO: check expression
    // EXPECT_TRUE(spec.soft_constraints_[i].expression_->equals(*(input(i+1))));
    EXPECT_TRUE(spec.soft_constraints_[i].lower_->equals(*(spec.soft_constraints_[i].upper_)));
    // TODO: check upper
  }

  // check that resulting controller is ok
  ASSERT_NO_THROW(generate(spec));
  QPController control = generate(spec);
  ASSERT_TRUE(control.start(state, nWSR));

  for (size_t i=0; i<25; ++i) {
    ASSERT_TRUE(control.update(state, nWSR));
    ASSERT_EQ(control.get_command().rows(), joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i)
      state[i] += control.get_command()[i]; // simulating kinematics
  }

  // check that we reached goal in the end
  KDL::Chain chain;
  ASSERT_TRUE(tree.getChain(trans3d_params.root_link, trans3d_params.tip_link, chain));
  ASSERT_EQ(chain.getNrOfJoints(), joint_names.size());

  KDL::ChainFkSolverPos_recursive fk_solver(chain);
  KDL::JntArray q_in(joint_names.size());
  for (size_t i=0; i<joint_names.size(); ++i)
    q_in(i) = state(i);
  KDL::Frame solver_frame;
  ASSERT_GE(fk_solver.JntToCart(q_in, solver_frame), 0);
  EXPECT_TRUE(KDL::Equal(solver_frame, goal));
}
