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


class ScopeTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      double_a = KDL::input(0);
      double_b = KDL::Constant(1.0);
      frame_1 = KDL::frame(KDL::rot_x(KDL::Constant(M_PI/2.0)));
      frame_2 = KDL::frame(KDL::vector(KDL::Constant(1.0), KDL::Constant(2.0), KDL::Constant(3.0)));
      rot_1 = KDL::rot_x(KDL::Constant(M_PI/2.0));
      rot_2 = KDL::rot_x(KDL::Constant(-M_PI/2.0));
    }

    virtual void TearDown(){}

    KDL::Expression<double>::Ptr double_a, double_b;
    KDL::Expression<KDL::Frame>::Ptr frame_1, frame_2;
    KDL::Expression<KDL::Rotation>::Ptr rot_1, rot_2;
};

TEST_F(ScopeTest, HasDouble)
{
  giskard_core::Scope scope;

  EXPECT_FALSE(scope.has_double_expression("a"));
  EXPECT_FALSE(scope.has_double_expression("b"));
  EXPECT_FALSE(scope.has_double_expression("c"));

  scope.add_double_expression("a", double_a);
  EXPECT_TRUE(scope.has_double_expression("a"));
  EXPECT_FALSE(scope.has_double_expression("b"));
  EXPECT_FALSE(scope.has_double_expression("c"));
 
  scope.add_double_expression("b", double_b);
  EXPECT_TRUE(scope.has_double_expression("a"));
  EXPECT_TRUE(scope.has_double_expression("b"));
  EXPECT_FALSE(scope.has_double_expression("c"));

  EXPECT_FALSE(scope.has_frame_expression("a"));
  EXPECT_FALSE(scope.has_frame_expression("b"));
  EXPECT_FALSE(scope.has_frame_expression("c"));

  EXPECT_FALSE(scope.has_rotation_expression("1"));
  EXPECT_FALSE(scope.has_rotation_expression("2"));
  EXPECT_FALSE(scope.has_rotation_expression("3"));
}

TEST_F(ScopeTest, HasFrame)
{
  giskard_core::Scope scope;

  EXPECT_FALSE(scope.has_frame_expression("1"));
  EXPECT_FALSE(scope.has_frame_expression("2"));
  EXPECT_FALSE(scope.has_frame_expression("3"));

  scope.add_frame_expression("1", frame_1);
  EXPECT_TRUE(scope.has_frame_expression("1"));
  EXPECT_FALSE(scope.has_frame_expression("2"));
  EXPECT_FALSE(scope.has_frame_expression("3"));

  scope.add_frame_expression("2", frame_2);
  EXPECT_TRUE(scope.has_frame_expression("1"));
  EXPECT_TRUE(scope.has_frame_expression("2"));
  EXPECT_FALSE(scope.has_frame_expression("3"));

  EXPECT_FALSE(scope.has_double_expression("1"));
  EXPECT_FALSE(scope.has_double_expression("2"));
  EXPECT_FALSE(scope.has_double_expression("3"));

  EXPECT_FALSE(scope.has_rotation_expression("1"));
  EXPECT_FALSE(scope.has_rotation_expression("2"));
  EXPECT_FALSE(scope.has_rotation_expression("3"));
}

TEST_F(ScopeTest, HasRotation)
{
  giskard_core::Scope scope;

  EXPECT_FALSE(scope.has_rotation_expression("1"));
  EXPECT_FALSE(scope.has_rotation_expression("2"));
  EXPECT_FALSE(scope.has_rotation_expression("3"));

  scope.add_rotation_expression("1", rot_1);
  EXPECT_TRUE(scope.has_rotation_expression("1"));
  EXPECT_FALSE(scope.has_rotation_expression("2"));
  EXPECT_FALSE(scope.has_rotation_expression("3"));

  scope.add_rotation_expression("2", rot_2);
  EXPECT_TRUE(scope.has_rotation_expression("1"));
  EXPECT_TRUE(scope.has_rotation_expression("2"));
  EXPECT_FALSE(scope.has_rotation_expression("3"));

  EXPECT_FALSE(scope.has_double_expression("1"));
  EXPECT_FALSE(scope.has_double_expression("2"));
  EXPECT_FALSE(scope.has_double_expression("3"));

  EXPECT_FALSE(scope.has_frame_expression("1"));
  EXPECT_FALSE(scope.has_frame_expression("2"));
  EXPECT_FALSE(scope.has_frame_expression("3"));
}

TEST_F(ScopeTest, FindDouble)
{
  giskard_core::Scope scope;

  scope.add_double_expression("a", double_a);
  EXPECT_EQ(double_a, scope.find_double_expression("a"));

  scope.add_double_expression("b", double_b);
  EXPECT_EQ(double_a, scope.find_double_expression("a"));
  EXPECT_EQ(double_b, scope.find_double_expression("b"));
}

TEST_F(ScopeTest, FindFrame)
{
  giskard_core::Scope scope;

  scope.add_frame_expression("1", frame_1);
  EXPECT_EQ(frame_1, scope.find_frame_expression("1"));

  scope.add_frame_expression("2", frame_2);
  EXPECT_EQ(frame_1, scope.find_frame_expression("1"));
  EXPECT_EQ(frame_2, scope.find_frame_expression("2"));
}

TEST_F(ScopeTest, FindRotation)
{
  giskard_core::Scope scope;

  scope.add_rotation_expression("rot_1", rot_1);
  EXPECT_EQ(rot_1, scope.find_rotation_expression("rot_1"));

  scope.add_rotation_expression("rot_2", rot_2);
  EXPECT_EQ(rot_1, scope.find_rotation_expression("rot_1"));
  EXPECT_EQ(rot_2, scope.find_rotation_expression("rot_2"));
}
