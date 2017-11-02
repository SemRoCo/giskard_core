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

class TestRobot : public Robot
{
  public:
     TestRobot(const urdf::Model& robot_model, const std::string& root_link,
          const std::vector<std::string>& tip_links, const std::map<std::string, double>& weights,
          const std::map<std::string, double>& thresholds) :
       Robot(robot_model, root_link, tip_links, weights, thresholds) {}

    const std::map<std::string, ControllableConstraintSpec>& get_controllables_map() const
    {
      return controllable_map_;
    }

    const std::map<std::string, HardConstraintSpec>& get_hard_constraints_map() const
    {
      return hard_map_;
    }

    const std::map<std::string, DoubleInputSpecPtr>& get_joints_map() const
    {
      return joint_map_;
    }

    double test_get_velocity_limit(const std::string& joint_name) const
    {
      return get_velocity_limit(joint_name);
    }

    double test_get_weight(const std::string& joint_name) const
    {
      return get_weight(joint_name);
    }
};

class RobotTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      ASSERT_TRUE(urdf.initFile("pr2.urdf"));
      ASSERT_TRUE(kdl_parser::treeFromUrdfModel(urdf, tree));
      root_link = "base_link";
      tip_link = "l_gripper_tool_frame";
      wrong_root_link = "foo";
      tip_links = {"torso_lift_link", "l_gripper_tool_frame"};
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
      limitless_joints = {"l_forearm_roll_joint", "l_wrist_roll_joint"};
      weights = {{"default-joint-weight", 0.001}, {"torso_lift_joint", 0.01}};
      thresholds = {{"default-joint-velocity", 0.6}, {"torso_lift_joint", 0.01}};
    }

    virtual void TearDown(){}

    virtual void TestFrameExpression(const KDL::Expression<KDL::Frame>::Ptr& exp, 
        const std::string& start_link, const std::string& end_link)
    {
      ASSERT_TRUE(exp.get());

      KDL::Chain chain;
      ASSERT_TRUE(tree.getChain(start_link, end_link, chain));
      ASSERT_EQ(chain.getNrOfJoints(), exp->number_of_derivatives());

      boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
      fk_solver = boost::shared_ptr<KDL::ChainFkSolverPos_recursive>(
          new KDL::ChainFkSolverPos_recursive(chain));

      for(int i=0; i<12; ++i)
      {
        std::vector<double> exp_in;
        KDL::JntArray solver_in(exp->number_of_derivatives());
        for(size_t j=0; j<exp->number_of_derivatives(); ++j)
        {
          double value = 0.1*i;
          exp_in.push_back(value);
          solver_in(j) = value;
        }

        exp->setInputValues(exp_in);
        KDL::Frame exp_frame = exp->value();

        KDL::Frame solver_frame;
        ASSERT_GE(fk_solver->JntToCart(solver_in, solver_frame), 0);

        EXPECT_TRUE(KDL::Equal(exp_frame, solver_frame));
      }
    }

    urdf::Model urdf;
    KDL::Tree tree;
    std::string root_link, tip_link, wrong_root_link;
    std::vector<std::string> tip_links, empty_tip_links, wrong_tip_links,
      root_as_only_tip_link, moveable_joints_names, all_joint_names;
    std::map<std::string, double> weights, thresholds;
    std::set<std::string> limitless_joints;
};

TEST_F(RobotTest, SaneConstructor)
{
  EXPECT_NO_THROW(Robot(urdf, root_link, tip_links, weights, thresholds));
  EXPECT_NO_THROW(Robot(urdf, root_link, root_as_only_tip_link, weights, thresholds));
  EXPECT_ANY_THROW(Robot(urdf, wrong_root_link, tip_links, weights, thresholds));
  EXPECT_NO_THROW(Robot(urdf, root_link, empty_tip_links, weights, thresholds));
  EXPECT_ANY_THROW(Robot(urdf, root_link, wrong_tip_links, weights, thresholds));
  EXPECT_ANY_THROW(Robot(urdf, root_link, wrong_tip_links, {}, thresholds));
  EXPECT_NO_THROW(Robot(urdf, root_link, tip_links, weights, {}));
  EXPECT_NO_THROW(Robot(urdf, root_link, empty_tip_links, {}, {}));
}

TEST_F(RobotTest, GetEmptyScope)
{
  ASSERT_NO_THROW(Robot(urdf, root_link, empty_tip_links, weights, thresholds));
  Robot my_robot(urdf, root_link, empty_tip_links, weights, thresholds);
  ASSERT_NO_THROW(my_robot.get_scope());
  EXPECT_EQ(my_robot.get_scope().size(), 0);
}

TEST_F(RobotTest, GetJointEmpty)
{
  ASSERT_NO_THROW(Robot(urdf, root_link, empty_tip_links, weights, thresholds));
  Robot my_robot(urdf, root_link, empty_tip_links, weights, thresholds);
  EXPECT_EQ(0, my_robot.get_number_of_joints());
  for (size_t i=0; i<all_joint_names.size(); ++i)
    EXPECT_ANY_THROW(my_robot.get_joint(all_joint_names[i]));
}

TEST_F(RobotTest, GetJointAll)
{
  ASSERT_NO_THROW(Robot(urdf, root_link, tip_links, weights, thresholds));
  Robot my_robot(urdf, root_link, tip_links, weights, thresholds);
  EXPECT_EQ(moveable_joints_names.size(), my_robot.get_number_of_joints());
  for (size_t i=0; i<moveable_joints_names.size(); ++i)
    ASSERT_NO_THROW(my_robot.get_joint(moveable_joints_names[i]));
}

TEST_F(RobotTest, ChainJointNames)
{
  ASSERT_NO_THROW(TestRobot(urdf, root_link, empty_tip_links, weights, thresholds));
  TestRobot robot(urdf, root_link, empty_tip_links, weights, thresholds);
  ASSERT_NO_THROW(robot.chain_joint_names(root_link, tip_link, false));
  std::vector<std::string> joint_names = 
    robot.chain_joint_names(root_link, tip_link, false);
  EXPECT_EQ(moveable_joints_names.size(), joint_names.size());
  for (size_t i=0; i<joint_names.size(); ++i)
    EXPECT_STREQ(moveable_joints_names[i].c_str(), joint_names[i].c_str());
  ASSERT_NO_THROW(robot.chain_joint_names(root_link, tip_link, true));
  joint_names = robot.chain_joint_names(root_link, tip_link, true);
  EXPECT_EQ(all_joint_names.size(), joint_names.size());
  for (size_t i=0; i<joint_names.size(); ++i)
    EXPECT_STREQ(all_joint_names[i].c_str(), joint_names[i].c_str());
}

TEST_F(RobotTest, GetScope)
{
  ASSERT_NO_THROW(Robot(urdf, root_link, tip_links, weights, thresholds));
  Robot my_robot(urdf, root_link, tip_links, weights, thresholds);
  ASSERT_NO_THROW(my_robot.get_scope());
  ASSERT_NO_THROW(generate(my_robot.get_scope()));
    Scope my_scope = generate(my_robot.get_scope());
  EXPECT_EQ(0, my_scope.get_double_names().size());
  EXPECT_EQ(0, my_scope.get_vector_names().size());
  EXPECT_EQ(0, my_scope.get_rotation_names().size());
  ASSERT_EQ(tip_links.size(), my_scope.get_frame_names().size());
  for (size_t i=0; i<tip_links.size(); ++i)
  {
    ASSERT_NO_THROW(my_scope.find_frame_expression(tip_links[i]));
    TestFrameExpression(my_scope.find_frame_expression(tip_links[i]), root_link, tip_links[i]);
  }
}

TEST_F(RobotTest, GetFkExpression)
{
  ASSERT_NO_THROW(Robot(urdf, root_link, tip_links, weights, thresholds));
  Robot my_robot(urdf, root_link, tip_links, weights, thresholds);
  for (size_t i=0; i<tip_links.size(); ++i)
    EXPECT_ANY_THROW(my_robot.get_fk_spec(wrong_root_link, tip_links[i]));

  for (size_t i=0; i<wrong_tip_links.size(); ++i)
    EXPECT_ANY_THROW(my_robot.get_fk_spec(root_link, wrong_tip_links[i]));

  for (size_t i=0; i<tip_links.size(); ++i)
  {
    ASSERT_NO_THROW(my_robot.get_fk_spec(root_link, tip_links[i]));
    FrameSpecPtr spec = my_robot.get_fk_spec(root_link, tip_links[i]);
    ASSERT_NO_THROW(spec->get_expression(Scope()));
    TestFrameExpression(spec->get_expression(Scope()), root_link, tip_links[i]);
  }
}

TEST_F(RobotTest, GetRootLink)
{
  ASSERT_NO_THROW(Robot(urdf, root_link, tip_links, weights, thresholds));
  Robot my_robot(urdf, root_link, tip_links, weights, thresholds);
  EXPECT_STREQ(root_link.c_str(), my_robot.get_root_link().c_str());
}

TEST_F(RobotTest, GetHardConstraints)
{
  // check that constructor is sane
  ASSERT_NO_THROW(TestRobot(urdf, root_link, tip_links, weights, thresholds));
  TestRobot my_robot(urdf, root_link, tip_links, weights, thresholds);
  EXPECT_NO_THROW(my_robot.get_hard_constraints());
  // check that numbers are correct
  EXPECT_EQ(my_robot.get_hard_constraints().size(), moveable_joints_names.size() - limitless_joints.size());
  EXPECT_EQ(my_robot.get_hard_constraints().size(), my_robot.get_hard_constraints_map().size());
  // check that we have no constraints for limitless joints in the amp
  for (auto const & limitless_joint: limitless_joints)
    EXPECT_TRUE(my_robot.get_hard_constraints_map().find(limitless_joint) == my_robot.get_hard_constraints_map().end());
  // check that we find all limited joints in the map
  for (auto const & joint_name: moveable_joints_names)
    ASSERT_TRUE((limitless_joints.find(joint_name) != limitless_joints.end()) ||
                        (my_robot.get_hard_constraints_map().find(joint_name) != my_robot.get_hard_constraints_map().end()));
  // check contents of the constraints themselves
  for (auto const & pair: my_robot.get_hard_constraints_map())
  {
    ASSERT_NO_THROW(my_robot.get_joint(pair.first));
    ASSERT_TRUE(my_robot.get_joint(pair.first)->equals(*(pair.second.expression_)));
    EXPECT_TRUE(double_sub_spec({double_const_spec(urdf.getJoint(pair.first)->limits->lower),
                                 my_robot.get_joint(pair.first)})->equals(*(pair.second.lower_)));
    EXPECT_TRUE(double_sub_spec({double_const_spec(urdf.getJoint(pair.first)->limits->upper),
                                 my_robot.get_joint(pair.first)})->equals(*(pair.second.upper_)));
  }
  // TODO: compare get_hard_constraints() and get_hard_constraints_map() this using HardConstraintSpec::equals(other)
}

TEST_F(RobotTest, GetControllableConstraints)
{
  // check that stuff does not break upon construction
  ASSERT_NO_THROW(TestRobot(urdf, root_link, tip_links, weights, thresholds));
  TestRobot my_robot(urdf, root_link, tip_links, weights, thresholds);
  // check that number of constraints fit
  ASSERT_NO_THROW(my_robot.get_controllable_constraints());
  EXPECT_EQ(my_robot.get_controllable_constraints().size(), moveable_joints_names.size());
  EXPECT_EQ(my_robot.get_controllable_constraints().size(), my_robot.get_controllables_map().size());
  // check that we can find all constraints by name in the map
  for (auto const & joint_name: moveable_joints_names)
    ASSERT_TRUE(my_robot.get_controllables_map().find(joint_name) != my_robot.get_controllables_map().end());
  // check the content of the individual constraints
  for (auto const & pair: my_robot.get_controllables_map())
  {
    ASSERT_NO_THROW(my_robot.get_joint(pair.first));
    EXPECT_STREQ(pair.first.c_str(), pair.second.name_.c_str());
    EXPECT_EQ(my_robot.get_joint(pair.first)->get_input_num(), pair.second.input_number_);
    ASSERT_NO_THROW(my_robot.test_get_velocity_limit(pair.first));
    double vel_limit = my_robot.test_get_velocity_limit(pair.first);
    EXPECT_TRUE(double_const_spec(-vel_limit)->equals(*(pair.second.lower_)));
    EXPECT_TRUE(double_const_spec(vel_limit)->equals(*(pair.second.upper_)));
    EXPECT_TRUE(double_const_spec(my_robot.test_get_weight(pair.first))->equals(*(pair.second.weight_)));
  }
  // check that controllable constraints appear ordered by input number
  for (size_t i=0; i<my_robot.get_controllable_constraints().size(); ++i)
    EXPECT_EQ(i, my_robot.get_controllable_constraints()[i].input_number_);
  // TODO: compare get_controllable_constraints() and get_controllables_map() using ControllableConstraintSpec::equals(other)
}

TEST_F(RobotTest, GetVelocityLimit) {
  ASSERT_NO_THROW(TestRobot(urdf, root_link, tip_links, weights, thresholds));
  TestRobot my_robot(urdf, root_link, tip_links, weights, thresholds);
  ASSERT_NO_THROW(TestRobot(urdf, root_link, tip_links, weights, {}));
  TestRobot no_thresholds_robot(urdf, root_link, tip_links, weights, {});
  for (auto const & joint_name: moveable_joints_names)
  {
    ASSERT_NO_THROW(my_robot.test_get_velocity_limit(joint_name));
    if (joint_name.compare("torso_lift_joint") == 0)
      EXPECT_DOUBLE_EQ(thresholds.find(joint_name)->second, my_robot.test_get_velocity_limit(joint_name));
    else
      EXPECT_DOUBLE_EQ(thresholds.find(Robot::default_joint_velocity_key())->second, my_robot.test_get_velocity_limit(joint_name));

    ASSERT_NO_THROW(no_thresholds_robot.test_get_velocity_limit(joint_name));
    EXPECT_DOUBLE_EQ(no_thresholds_robot.test_get_velocity_limit(joint_name), urdf.getJoint(joint_name)->limits->velocity);
  }
}

TEST_F(RobotTest, GetVelocityLimitNoDefaults) {
  ASSERT_NO_THROW(TestRobot(urdf, root_link, tip_links, weights, {}));
  TestRobot my_robot(urdf, root_link, tip_links, weights, {});
  for (auto const & joint_name: moveable_joints_names)
  {
    ASSERT_NO_THROW(my_robot.test_get_velocity_limit(joint_name));
    EXPECT_DOUBLE_EQ(my_robot.test_get_velocity_limit(joint_name), urdf.getJoint(joint_name)->limits->velocity);
  }
}

TEST_F(RobotTest, GetWeight) {
  ASSERT_NO_THROW(TestRobot(urdf, root_link, tip_links, weights, thresholds));
  TestRobot my_robot(urdf, root_link, tip_links, weights, thresholds);
  for (auto const &joint_name: moveable_joints_names)
  {
    ASSERT_NO_THROW(my_robot.test_get_weight(joint_name));
    if (joint_name.compare("torso_lift_joint") == 0)
      EXPECT_DOUBLE_EQ(weights.find(joint_name)->second, my_robot.test_get_weight(joint_name));
    else
      EXPECT_DOUBLE_EQ(weights.find(Robot::default_joint_weight_key())->second, my_robot.test_get_weight(joint_name));
  }
}

TEST_F(RobotTest, GetWeightNoDefaults) {
  EXPECT_ANY_THROW(TestRobot(urdf, root_link, tip_links, {}, thresholds));
}

TEST_F(RobotTest, ContinuousJointsNames) {
  ASSERT_NO_THROW(Robot(urdf, root_link, empty_tip_links, {}, {}));
  Robot my_robot(urdf, root_link, empty_tip_links, {}, {});
  ASSERT_NO_THROW(my_robot.continuous_joints_names(root_link, tip_link));
  std::set<std::string> joint_names = my_robot.continuous_joints_names(root_link, tip_link);
  EXPECT_EQ(joint_names.size(), 2);
  EXPECT_TRUE(joint_names.find("l_forearm_roll_joint") != joint_names.end());
  EXPECT_TRUE(joint_names.find("l_wrist_roll_joint") != joint_names.end());
}
