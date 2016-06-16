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


class RotationControlTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      using namespace KDL;
    
      goal_rot_z = 0.815;
      goal_rot = Rotation::RotZ(goal_rot_z);
      goal_rot2 = Rotation::RotZ(0.3) * Rotation::RotY(-0.2) * Rotation::RotX(0.1);

      ee = rot_z(input(0)) * rot_y(input(1)) * rot_x(input(2));

      error = norm(getRotVec(inv(Constant(goal_rot)) * ee));
      error2 = norm(getRotVec(inv(Constant(goal_rot2)) * ee));

      control = Constant(-1.0) * error;
      control2 = Constant(-1.0) * error2;

      joints.resize(3);
      for(size_t i=0; i<joints.rows(); ++i)
        joints(i) = 0.0;
    }

    virtual void TearDown(){}
    Eigen::VectorXd joints;
    KDL::Expression<KDL::Rotation>::Ptr ee;
    KDL::Expression<double>::Ptr error, error2, control, control2;
    double goal_rot_z;
    KDL::Rotation goal_rot, goal_rot2;

    Eigen::VectorXd toEigen(const std::vector<double>& in)
    {
      Eigen::VectorXd out(in.size());
      for(size_t i=0; i<in.size(); ++i)
        out(i) = in[i];
      return out;
    }

    std::vector<double> toSTL(const Eigen::VectorXd& in)
    {
      std::vector<double> out;
      out.resize(in.rows());
      for(size_t i=0; i<in.rows(); ++i)
        out[i] = in(i);
      return out;
    }
};

TEST_F(RotationControlTest, KDLBasics)
{
  error->setInputValues(toSTL(joints));
  EXPECT_NEAR(error->value(), goal_rot_z, 0.00001);

  ASSERT_EQ(error->number_of_derivatives(), joints.rows());
  EXPECT_NEAR(error->derivative(0), -1, 0.0000001);
  EXPECT_NEAR(error->derivative(1), 0, 0.0000001);
  EXPECT_NEAR(error->derivative(2), 0, 0.0000001);
}

TEST_F(RotationControlTest, QPControlSingleAxis)
{
  using namespace KDL;
  using namespace giskard;
  using namespace Eigen;
  using namespace std;

  QPController::DoubleExpressionVector controllable_lower, controllable_upper, controllable_weights;
  QPController::DoubleExpressionVector soft_exp, soft_lower, soft_upper, soft_weights;
  QPController::DoubleExpressionVector hard_exp, hard_lower, hard_upper;
  QPController::StringVector soft_names, controllable_names;

  for(size_t i=0; i<joints.rows(); ++i)
  {
    controllable_lower.push_back(Constant(-0.05));
    controllable_upper.push_back(Constant(0.05));
    controllable_weights.push_back(Constant(1.0));
    controllable_names.push_back("joint " + i);
  }

  soft_exp.push_back(error);
  soft_lower.push_back(control);
  soft_upper.push_back(control);
  soft_weights.push_back(Constant(1.0));
  soft_names.push_back("rot control single-axis");

  QPController controller;
  ASSERT_TRUE(controller.init(controllable_lower, controllable_upper, controllable_weights,
      controllable_names, soft_exp, soft_lower, soft_upper, soft_weights, soft_names,
      hard_exp, hard_lower, hard_upper));

  ASSERT_TRUE(controller.start(joints, 100));

  for(size_t i=0; i<100; ++i)
  {
    error->setInputValues(toSTL(joints));
    double old_error = error->value();

    ASSERT_TRUE(controller.update(joints, 100));
    joints += controller.get_command();

    EXPECT_LE(0.0, controller.get_command()(0));
    EXPECT_LE(controller.get_command()(0), 0.05);
    EXPECT_EQ(0.0, controller.get_command()(1));
    EXPECT_EQ(0.0, controller.get_command()(2));

    error->setInputValues(toSTL(joints));
    EXPECT_LE(error->value(), old_error);
  }

  EXPECT_TRUE(Equal(ee->value(), goal_rot));
}

TEST_F(RotationControlTest, QPControlSeveralAxes)
{
  using namespace KDL;
  using namespace giskard;
  using namespace Eigen;
  using namespace std;

  QPController::DoubleExpressionVector controllable_lower, controllable_upper, controllable_weights;
  QPController::DoubleExpressionVector soft_exp, soft_lower, soft_upper, soft_weights;
  QPController::DoubleExpressionVector hard_exp, hard_lower, hard_upper;
  QPController::StringVector soft_names, controllable_names;

  for(size_t i=0; i<joints.rows(); ++i)
  {
    controllable_lower.push_back(Constant(-0.05));
    controllable_upper.push_back(Constant(0.05));
    controllable_weights.push_back(Constant(1.0));
    controllable_names.push_back("joint " + i);
  }

  soft_exp.push_back(error2);
  soft_lower.push_back(control2);
  soft_upper.push_back(control2);
  soft_weights.push_back(Constant(1.0));
  soft_names.push_back("rot control multi-axis");

  QPController controller;
  ASSERT_TRUE(controller.init(controllable_lower, controllable_upper, controllable_weights,
      controllable_names, soft_exp, soft_lower, soft_upper, soft_weights, soft_names,
      hard_exp, hard_lower, hard_upper));

  ASSERT_TRUE(controller.start(joints, 100));

  for(size_t i=0; i<100; ++i)
  {
    error->setInputValues(toSTL(joints));
    double old_error = error2->value();

    ASSERT_TRUE(controller.update(joints, 100));
    joints += controller.get_command();

    EXPECT_LE(-0.05, controller.get_command()(0));
    EXPECT_LE(controller.get_command()(0), 0.05);
    EXPECT_LE(-0.05, controller.get_command()(1));
    EXPECT_LE(controller.get_command()(1), 0.05);
    EXPECT_LE(-0.05, controller.get_command()(2));
    EXPECT_LE(controller.get_command()(2), 0.05);

    error2->setInputValues(toSTL(joints));
    EXPECT_LE(error2->value(), old_error);
  }

  EXPECT_TRUE(Equal(ee->value(), goal_rot2));
}
