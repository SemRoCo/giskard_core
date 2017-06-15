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

  ASSERT_NO_THROW(node.as<giskard_core::QPControllerSpec>());
  giskard_core::QPControllerSpec spec = node.as<giskard_core::QPControllerSpec>();

  EXPECT_EQ(14, spec.scope_.size());
  EXPECT_EQ(6, spec.controllable_constraints_.size());
  EXPECT_EQ(0, spec.hard_constraints_.size());
  EXPECT_EQ(2, spec.soft_constraints_.size());

  giskard_core::Scope scope = giskard_core::generate(spec.scope_);
  ASSERT_TRUE(scope.has_double_expression("mug-above-maker"));
  ASSERT_TRUE(scope.has_double_expression("mug-upright"));
  KDL::Expression<double>::Ptr mug_above = scope.find_double_expression("mug-above-maker");
  KDL::Expression<double>::Ptr mug_upright = scope.find_double_expression("mug-upright");

  Eigen::VectorXd state(12);
  using Eigen::operator<<;
  state << 0.2, 0.1, 1.855, 0.01, 0.01, 0, 0.3, 0.4, 0.89, 0, 0, 0;
  int nWSR = 100;

  EXPECT_EQ(14, spec.scope_.size());
  EXPECT_EQ(6, spec.controllable_constraints_.size());
  EXPECT_EQ(0, spec.hard_constraints_.size());
  EXPECT_EQ(2, spec.soft_constraints_.size());

  ASSERT_NO_THROW(giskard_core::generate(spec));
  giskard_core::QPController controller = giskard_core::generate(spec);
  ASSERT_EQ(8, controller.get_qp_builder().get_H().rows());
  ASSERT_EQ(8, controller.get_qp_builder().get_H().cols());
  ASSERT_EQ(2, controller.get_qp_builder().get_A().rows());
  ASSERT_EQ(8, controller.get_qp_builder().get_A().cols());
  ASSERT_EQ(6, controller.get_controllable_names().size());
  ASSERT_EQ(2, controller.get_soft_constraint_names().size());

  // test names
  EXPECT_STREQ("mug_pos_x", controller.get_controllable_names()[0].c_str());
  EXPECT_STREQ("mug_pos_y", controller.get_controllable_names()[1].c_str());
  EXPECT_STREQ("mug_pos_z", controller.get_controllable_names()[2].c_str());
  EXPECT_STREQ("mug_rot_x", controller.get_controllable_names()[3].c_str());
  EXPECT_STREQ("mug_rot_y", controller.get_controllable_names()[4].c_str());
  EXPECT_STREQ("mug_rot_z", controller.get_controllable_names()[5].c_str());

  EXPECT_STREQ("mug_above_maker", controller.get_soft_constraint_names()[0].c_str());
  EXPECT_STREQ("mug_upright", controller.get_soft_constraint_names()[1].c_str());

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

TEST_F(FlyingCupTest, IssueBrokenFlyingCup)
{
  YAML::Node node = YAML::LoadFile("broken_flying_cup.yaml");
  ASSERT_NO_THROW(node.as<giskard_core::QPControllerSpec>());
  giskard_core::QPControllerSpec spec = node.as<giskard_core::QPControllerSpec>();
  ASSERT_NO_THROW(giskard_core::generate(spec));
  giskard_core::QPController controller = giskard_core::generate(spec);

  // setup
  size_t nWSR = 100;
  KDL::Frame maker_frame =
    KDL::Frame(
      KDL::Rotation::Quaternion(
          0.706475862557,
          0.705969763725,
          -0.0347925064966,
          0.0358891323504),
      KDL::Vector(-0.131965, 0.924513, 1.05));

  KDL::Frame mug_frame =
    KDL::Frame(KDL::Rotation::Quaternion(0,0,0,1),   
        KDL::Vector(-0.459675, 0.976666, 1.36252));

  Eigen::VectorXd state(12);
  state(0) = mug_frame.p.x();
  state(1) = mug_frame.p.y();
  state(2) = mug_frame.p.z();
  mug_frame.M.GetEulerZYX(state(3), state(4), state(5));
  state(6) = maker_frame.p.x();
  state(7) = maker_frame.p.y();
  state(8) = maker_frame.p.z();
  maker_frame.M.GetEulerZYX(state(9), state(10), state(11));

  ASSERT_TRUE(controller.start(state, nWSR));
  ASSERT_TRUE(controller.update(state, nWSR));
}
