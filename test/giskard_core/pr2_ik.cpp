/*
 * Copyright (C) 2015-2017 Georg Bartels <georg.bartels@cs.uni-bremen.de>
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
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainjnttojacsolver.hpp>

class PR2IKTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      urdf::Model urdf;
      urdf.initFile("pr2.urdf");
      ASSERT_TRUE(kdl_parser::treeFromUrdfModel(urdf, tree));
   }

    virtual void TearDown(){}

    KDL::Tree tree;
};

TEST_F(PR2IKTest, SingleExpression)
{
  YAML::Node node = YAML::LoadFile("pr2_left_arm_single_expression.yaml");
  ASSERT_NO_THROW(node.as<giskard_core::FrameSpecPtr>());

  giskard_core::FrameSpecPtr spec = node.as<giskard_core::FrameSpecPtr>();
  giskard_core::ScopeSpec scopeSpec;
  scopeSpec.push_back({"foo", spec});

  giskard_core::Scope scope; 
  ASSERT_NO_THROW(scope = giskard_core::generate(scopeSpec));
  ASSERT_NO_THROW(spec->get_expression(scope));
  
  KDL::Expression<KDL::Frame>::Ptr exp = spec->get_expression(scope);
  ASSERT_TRUE(exp.get()); 

  std::string base = "torso_lift_link";
  std::string tip = "l_wrist_roll_link";
  KDL::Chain chain;
  ASSERT_TRUE(tree.getChain(base, tip, chain));
  ASSERT_EQ(chain.getNrOfJoints(), exp->number_of_derivatives());

  boost::shared_ptr<KDL::ChainJntToJacSolver> jac_solver;
  jac_solver = boost::shared_ptr<KDL::ChainJntToJacSolver>(
      new KDL::ChainJntToJacSolver(chain));

  for(int i=-11; i<12; ++i)
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
    exp->value();
    KDL::Jacobian exp_jac(exp->number_of_derivatives());
    for(size_t j=0; j<exp_jac.columns(); ++j)
      exp_jac.setColumn(j, exp->derivative(j));

    KDL::Jacobian solver_jac(chain.getNrOfJoints());
    ASSERT_GE(jac_solver->JntToJac(solver_in, solver_jac), 0.0);

    EXPECT_TRUE(KDL::Equal(solver_jac, exp_jac));
  }
}

TEST_F(PR2IKTest, SingleRowRotation)
{
  YAML::Node node = YAML::LoadFile("pr2_left_arm_scope.yaml");

  ASSERT_NO_THROW(node.as< giskard_core::ScopeSpec >());
  giskard_core::ScopeSpec scope_spec = node.as<giskard_core::ScopeSpec>();

  ASSERT_NO_THROW(giskard_core::generate(scope_spec));
  giskard_core::Scope scope = giskard_core::generate(scope_spec);

  ASSERT_TRUE(scope.has_double_expression("pr2_rot_x"));

  KDL::Expression<double>::Ptr exp = scope.find_double_expression("pr2_rot_x");
  ASSERT_TRUE(exp.get()); 

  std::string base = "base_link";
  std::string tip = "l_wrist_roll_link";
  KDL::Chain chain;
  ASSERT_TRUE(tree.getChain(base, tip, chain));
  ASSERT_EQ(chain.getNrOfJoints(), exp->number_of_derivatives());

  boost::shared_ptr<KDL::ChainJntToJacSolver> jac_solver;
  jac_solver = boost::shared_ptr<KDL::ChainJntToJacSolver>(
      new KDL::ChainJntToJacSolver(chain));

  for(int i=-11; i<12; ++i)
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
    exp->value();

    KDL::Jacobian solver_jac(chain.getNrOfJoints());
    ASSERT_GE(jac_solver->JntToJac(solver_in, solver_jac), 0.0);

    for (size_t j=0; j<chain.getNrOfJoints(); ++j)
      EXPECT_DOUBLE_EQ(solver_jac(3,j), exp->derivative(j));
  }
}
