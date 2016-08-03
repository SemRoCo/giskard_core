/*
 * Copyright (C) 2015 Georg Bartels <georg.bartels@cs.uni-bremen.de>
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
#include <giskard/giskard.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

class PR2FKTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      urdf::Model urdf;
      ASSERT_TRUE(urdf.initFile("pr2.urdf"));
      ASSERT_TRUE(kdl_parser::treeFromUrdfModel(urdf, tree));
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

    KDL::Tree tree;
};

TEST_F(PR2FKTest, SingleExpression)
{
  YAML::Node node = YAML::LoadFile("pr2_left_arm_single_expression.yaml");
  ASSERT_NO_THROW(node.as<giskard::FrameSpecPtr>());

  giskard::FrameSpecPtr spec = node.as<giskard::FrameSpecPtr>();
  ASSERT_NO_THROW(spec->get_expression(giskard::Scope()));

  KDL::Expression<KDL::Frame>::Ptr exp = spec->get_expression(giskard::Scope());
  std::string base = "torso_lift_link";
  std::string tip = "l_wrist_roll_link";
  TestFrameExpression(exp, base, tip);
}

TEST_F(PR2FKTest, Scope)
{
  YAML::Node node = YAML::LoadFile("pr2_left_arm_scope.yaml");

  ASSERT_NO_THROW(node.as< giskard::ScopeSpec >());
  giskard::ScopeSpec scope_spec = node.as<giskard::ScopeSpec>();

  ASSERT_NO_THROW(giskard::generate(scope_spec));
  giskard::Scope scope = giskard::generate(scope_spec);

  ASSERT_TRUE(scope.has_frame_expression("pr2_fk"));

  KDL::Expression<KDL::Frame>::Ptr exp = scope.find_frame_expression("pr2_fk");
  std::string base = "base_link";
  std::string tip = "l_wrist_roll_link";
  TestFrameExpression(exp, base, tip);
}

TEST_F(PR2FKTest, GeneratedFromUrdf)
{
  std::string base = "base_link";
  std::string tip = "l_wrist_roll_link";
  YAML::Node node = giskard::extract_expression(base, tip, "pr2.urdf");

  ASSERT_NO_THROW(node.as< giskard::ScopeSpec >());
  giskard::ScopeSpec scope_spec = node.as<giskard::ScopeSpec>();

  ASSERT_NO_THROW(giskard::generate(scope_spec));
  giskard::Scope scope = giskard::generate(scope_spec);

  ASSERT_TRUE(scope.has_frame_expression("fk"));

  KDL::Expression<KDL::Frame>::Ptr exp = scope.find_frame_expression("fk");
  TestFrameExpression(exp, base, tip);
}

TEST_F(PR2FKTest, GeneratedFromTree)
{
  std::string base = "base_link";
  std::string tip = "l_wrist_roll_link";
  YAML::Node node = giskard::extract_expression(base, tip, tree);

  ASSERT_NO_THROW(node.as< giskard::ScopeSpec >());
  giskard::ScopeSpec scope_spec = node.as<giskard::ScopeSpec>();

  ASSERT_NO_THROW(giskard::generate(scope_spec));
  giskard::Scope scope = giskard::generate(scope_spec);

  ASSERT_TRUE(scope.has_frame_expression("fk"));

  KDL::Expression<KDL::Frame>::Ptr exp = scope.find_frame_expression("fk");
  TestFrameExpression(exp, base, tip);
}

TEST_F(PR2FKTest, TipToForearm)
{
  std::string base = "l_forearm_link";
  std::string tip = "l_wrist_flex_link";
  YAML::Node node = giskard::extract_expression(tip, base, tree);

  ASSERT_NO_THROW(node.as< giskard::ScopeSpec >());
  giskard::ScopeSpec scope_spec = node.as<giskard::ScopeSpec>();

  ASSERT_NO_THROW(giskard::generate(scope_spec));
  giskard::Scope scope = giskard::generate(scope_spec);

  ASSERT_TRUE(scope.has_frame_expression("fk"));

  KDL::Expression<KDL::Frame>::Ptr exp = scope.find_frame_expression("fk");
  TestFrameExpression(exp, tip, base);
}

TEST_F(PR2FKTest, TipToBase)
{
  std::string base = "base_link";
  std::string tip = "l_wrist_flex_link";
  YAML::Node node = giskard::extract_expression(tip, base, tree);

  ASSERT_NO_THROW(node.as< giskard::ScopeSpec >());
  giskard::ScopeSpec scope_spec = node.as<giskard::ScopeSpec>();

  ASSERT_NO_THROW(giskard::generate(scope_spec));
  giskard::Scope scope = giskard::generate(scope_spec);

  ASSERT_TRUE(scope.has_frame_expression("fk"));

  KDL::Expression<KDL::Frame>::Ptr exp = scope.find_frame_expression("fk");
  TestFrameExpression(exp, tip, base);
}

TEST_F(PR2FKTest, TipToTip)
{
  std::string base = "r_wrist_flex_link";
  std::string tip = "l_wrist_flex_link";
  YAML::Node node = giskard::extract_expression(tip, base, tree);

  ASSERT_NO_THROW(node.as< giskard::ScopeSpec >());
  giskard::ScopeSpec scope_spec = node.as<giskard::ScopeSpec>();

  ASSERT_NO_THROW(giskard::generate(scope_spec));
  giskard::Scope scope = giskard::generate(scope_spec);

  ASSERT_TRUE(scope.has_frame_expression("fk"));

  KDL::Expression<KDL::Frame>::Ptr exp = scope.find_frame_expression("fk");
  TestFrameExpression(exp, tip, base);
}

TEST_F(PR2FKTest, BaseToCam)
{
  std::string base = "base_link";
  std::string tip = "l_forearm_cam_frame";
  YAML::Node node = giskard::extract_expression(tip, base, tree);

  ASSERT_NO_THROW(node.as< giskard::ScopeSpec >());
  giskard::ScopeSpec scope_spec = node.as<giskard::ScopeSpec>();

  ASSERT_NO_THROW(giskard::generate(scope_spec));
  giskard::Scope scope = giskard::generate(scope_spec);

  ASSERT_TRUE(scope.has_frame_expression("fk"));

  KDL::Expression<KDL::Frame>::Ptr exp = scope.find_frame_expression("fk");
  TestFrameExpression(exp, tip, base);
}

TEST_F(PR2FKTest, CamToBase)
{
  std::string base = "l_forearm_cam_frame";
  std::string tip = "base_link";
  YAML::Node node = giskard::extract_expression(tip, base, tree);

  ASSERT_NO_THROW(node.as< giskard::ScopeSpec >());
  giskard::ScopeSpec scope_spec = node.as<giskard::ScopeSpec>();

  ASSERT_NO_THROW(giskard::generate(scope_spec));
  giskard::Scope scope = giskard::generate(scope_spec);

  ASSERT_TRUE(scope.has_frame_expression("fk"));

  KDL::Expression<KDL::Frame>::Ptr exp = scope.find_frame_expression("fk");
  TestFrameExpression(exp, tip, base);
}

TEST_F(PR2FKTest, QPPositionControl)
{
  YAML::Node node = YAML::LoadFile("pr2_qp_position_control.yaml");

  ASSERT_TRUE(node.IsMap());

  ASSERT_NO_THROW(node.as< giskard::QPControllerSpec >());
  giskard::QPControllerSpec spec = node.as< giskard::QPControllerSpec >();
  giskard::Scope scope = giskard::generate(spec.scope_);
  KDL::Expression<double>::Ptr error = scope.find_double_expression("pr2_fk_error");

  Eigen::VectorXd state(8);
  using Eigen::operator<<;
  state << 0.02, 0.0, 0.0, 0.0, -0.16, 0.0, -0.11, 0.0;
  int nWSR = 10;
  ASSERT_NO_THROW(giskard::generate(spec));
  giskard::QPController controller = giskard::generate(spec);

  // setup
  size_t iterations = 400;
  double dt = 0.01;
  std::vector<double> state_tmp;
  state_tmp.resize(state.rows());

  error->setInputValues(state_tmp);
  EXPECT_GE(error->value(), 0.3);

  ASSERT_TRUE(controller.start(state, nWSR));
  for(size_t i=0; i<iterations; ++i)
  {
    ASSERT_TRUE(controller.update(state, nWSR));

    for(size_t j=0; j<state.rows(); ++j)
      state_tmp[j] = state(j);
    error->setInputValues(state_tmp);
    double last_error = error->value();

    state += dt * controller.get_command();

    for(size_t j=0; j<state.rows(); ++j)
      state_tmp[j] = state(j);
    error->setInputValues(state_tmp);
    double current_error = error->value();

    EXPECT_LE(current_error, last_error);
  }

  EXPECT_LE(error->value(), 0.01);
}

TEST_F(PR2FKTest, QPPositionControlWithDeactivatedControllables)
{
  YAML::Node node = YAML::LoadFile("pr2_qp_position_control_with_deactivated_controllables.yaml");

  ASSERT_TRUE(node.IsMap());

  ASSERT_NO_THROW(node.as< giskard::QPControllerSpec >());
  giskard::QPControllerSpec spec = node.as< giskard::QPControllerSpec >();

  giskard::Scope scope = giskard::generate(spec.scope_);
  KDL::Expression<double>::Ptr error = scope.find_double_expression("pr2_fk_error");

  Eigen::VectorXd state(8);
  using Eigen::operator<<;
  state << 0.02, 0.0, 0.0, 0.0, -0.16, 0.0, -0.11, 0.0;
  int nWSR = 10;
  ASSERT_NO_THROW(giskard::generate(spec));
  giskard::QPController controller = giskard::generate(spec);

  // setup
  size_t iterations = 500;
  double dt = 0.01;
  std::vector<double> state_tmp;
  state_tmp.resize(state.rows());

  error->setInputValues(state_tmp);
  EXPECT_GE(error->value(), 0.3);

  ASSERT_TRUE(controller.start(state, nWSR));
  for(size_t i=0; i<iterations; ++i)
  {
    ASSERT_TRUE(controller.update(state, nWSR));

    for(size_t j=0; j<state.rows(); ++j)
      state_tmp[j] = state(j);
    error->setInputValues(state_tmp);
    double last_error = error->value();

    state += dt * controller.get_command();

    EXPECT_DOUBLE_EQ(controller.get_command()(0), 0.0);

    for(size_t j=0; j<state.rows(); ++j)
      state_tmp[j] = state(j);
    error->setInputValues(state_tmp);
    double current_error = error->value();

    EXPECT_LE(current_error, last_error);

  }

  EXPECT_LE(error->value(), 0.01);
}

TEST_F(PR2FKTest, QPPositionControlWithExcessObservables)
{
  YAML::Node node = YAML::LoadFile("pr2_qp_position_control_with_excess_observables.yaml");

  ASSERT_TRUE(node.IsMap());

  ASSERT_NO_THROW(node.as< giskard::QPControllerSpec >());
  giskard::QPControllerSpec spec = node.as< giskard::QPControllerSpec >();

  giskard::Scope scope = giskard::generate(spec.scope_);
  KDL::Expression<double>::Ptr error = scope.find_double_expression("pr2_fk_error");

  Eigen::VectorXd state(8);
  using Eigen::operator<<;
  state << 0.02, 0.0, 0.0, 0.0, -0.16, 0.0, -0.11, 0.0;
  int nWSR = 10;
  ASSERT_NO_THROW(giskard::generate(spec));
  giskard::QPController controller = giskard::generate(spec);

  // setup
  size_t iterations = 300;
  double dt = 0.01;
  std::vector<double> state_tmp;
  state_tmp.resize(state.rows());

  error->setInputValues(state_tmp);
  EXPECT_GE(error->value(), 0.3);

  ASSERT_TRUE(controller.start(state, nWSR));
  for(size_t i=0; i<iterations; ++i)
  {
    ASSERT_TRUE(controller.update(state, nWSR));

    for(size_t j=0; j<state.rows(); ++j)
      state_tmp[j] = state(j);
    error->setInputValues(state_tmp);
    double last_error = error->value();

    state.segment(0, 6) += dt * controller.get_command();

    for(size_t j=0; j<state.rows(); ++j)
      state_tmp[j] = state(j);
    error->setInputValues(state_tmp);
    double current_error = error->value();

    EXPECT_LE(current_error, last_error);

  }

  EXPECT_LE(error->value(), 0.01);
}
