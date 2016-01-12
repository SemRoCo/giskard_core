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

class FlyingCupTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
    }

    virtual void TearDown(){}

};

TEST_F(FlyingCupTest, ApproachMotion)
{
  YAML::Node node = YAML::LoadFile("flying_cup_approach_motion.yaml");

  ASSERT_NO_THROW(node.as<giskard::QPControllerSpec>());
  giskard::QPControllerSpec spec = node.as<giskard::QPControllerSpec>();

  EXPECT_EQ(14, spec.scope_.size());
  EXPECT_EQ(6, spec.controllable_constraints_.size());
  EXPECT_EQ(0, spec.hard_constraints_.size());
  EXPECT_EQ(2, spec.soft_constraints_.size());

  giskard::Scope scope = giskard::generate(spec.scope_);
  ASSERT_TRUE(scope.has_double_expression("mug-above-maker"));
  ASSERT_TRUE(scope.has_double_expression("mug-upright"));
  KDL::Expression<double>::Ptr mug_above = scope.find_double_expression("mug-above-maker");
  KDL::Expression<double>::Ptr mug_upright = scope.find_double_expression("mug-upright");

  Eigen::VectorXd state(12);
  using Eigen::operator<<;
  state << 0.2, 0.1, 1.855, 0.01, 0.01, 0, 0.3, 0.4, 0.89, 0, 0, 0;
  int nWSR = 10;

  EXPECT_EQ(14, spec.scope_.size());
  EXPECT_EQ(6, spec.controllable_constraints_.size());
  EXPECT_EQ(0, spec.hard_constraints_.size());
  EXPECT_EQ(2, spec.soft_constraints_.size());

  ASSERT_NO_THROW(giskard::generate(spec));
  giskard::QPController controller = giskard::generate(spec);
  ASSERT_EQ(8, controller.get_qp_builder().get_H().rows());
  ASSERT_EQ(8, controller.get_qp_builder().get_H().cols());
  ASSERT_EQ(2, controller.get_qp_builder().get_A().rows());
  ASSERT_EQ(8, controller.get_qp_builder().get_A().cols());

  // setup
  size_t iterations = 500;
  double dt = 0.01;
  std::vector<double> state_tmp;
  state_tmp.resize(state.rows());
  for(size_t j=0; j<state.rows(); ++j)
    state_tmp[j] = state(j);
 
  mug_above->setInputValues(state_tmp);
  EXPECT_GE(mug_above->value(), 0.5);
  mug_upright->setInputValues(state_tmp);
  EXPECT_GE(mug_upright->value(), 0.08);

  ASSERT_TRUE(controller.start(state, nWSR));
  for(size_t i=0; i<iterations; ++i)
  {
    ASSERT_TRUE(controller.update(state, nWSR));

    for(size_t j=0; j<state.rows(); ++j)
      state_tmp[j] = state(j);
    mug_upright->setInputValues(state_tmp);
    mug_above->setInputValues(state_tmp);

    double last_upright = mug_upright->value();
    double last_above = mug_above->value();

    state.segment(0, 6) += dt * controller.get_command();

    for(size_t j=0; j<state.rows(); ++j)
      state_tmp[j] = state(j);
    mug_upright->setInputValues(state_tmp);
    mug_above->setInputValues(state_tmp);
    double current_upright = mug_upright->value();
    double current_above = mug_above->value();

    EXPECT_LE(current_above, last_above);
    EXPECT_LE(current_upright, last_upright);
  }

  EXPECT_LE(0.019, mug_upright->value());
  EXPECT_LE(mug_upright->value(), 0.041);

  EXPECT_LE(0.29, mug_above->value());
  EXPECT_LE(mug_above->value(), 0.36);

}
