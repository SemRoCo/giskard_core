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
#include <kdl/chainfksolverpos_recursive.hpp>

class BoxyFKTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      urdf::Model urdf;
      ASSERT_TRUE(urdf.initFile("boxy.urdf"));
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

// TODO: get rid of this test case
//TEST_F(BoxyFKTest, GeneratedFromUrdf)
//{
//  std::string base = "base_footprint";
//  std::string tip = "left_gripper_tool_frame";
//  YAML::Node node = giskard_core::extract_expression(base, tip, "boxy.urdf");
//
//  ASSERT_NO_THROW(node.as< giskard_core::ScopeSpec >());
//  giskard_core::ScopeSpec scope_spec = node.as<giskard_core::ScopeSpec>();
//
//  ASSERT_NO_THROW(giskard_core::generate(scope_spec));
//  giskard_core::Scope scope = giskard_core::generate(scope_spec);
//
//  ASSERT_TRUE(scope.has_frame_expression("fk"));
//
//  KDL::Expression<KDL::Frame>::Ptr exp = scope.find_frame_expression("fk");
//  TestFrameExpression(exp, base, tip);
//}
