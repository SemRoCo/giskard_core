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


class FrameGenerationTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
    }

    virtual void TearDown(){}
};

TEST_F(FrameGenerationTest, ConstructorFrame)
{
  giskard::core::DoubleConstSpecPtr rot_axis_x(new giskard::core::DoubleConstSpec());
  rot_axis_x->set_value(1.0);
  giskard::core::DoubleConstSpecPtr rot_axis_y(new giskard::core::DoubleConstSpec());
  rot_axis_y->set_value(2.0);
  giskard::core::DoubleConstSpecPtr rot_axis_z(new giskard::core::DoubleConstSpec());
  rot_axis_z->set_value(3.0);
  giskard::core::VectorConstructorSpecPtr rot_axis(new giskard::core::VectorConstructorSpec());
  rot_axis->set(rot_axis_x, rot_axis_y, rot_axis_z);

  giskard::core::DoubleConstSpecPtr rot_angle(new giskard::core::DoubleConstSpec());
  rot_angle->set_value(-4.0);

  giskard::core::AxisAngleSpecPtr rot(new giskard::core::AxisAngleSpec());
  rot->set_axis(rot_axis);
  rot->set_angle(rot_angle);

  giskard::core::DoubleConstSpecPtr trans_x(new giskard::core::DoubleConstSpec());
  giskard::core::DoubleConstSpecPtr trans_y(new giskard::core::DoubleConstSpec());
  giskard::core::DoubleConstSpecPtr trans_z(new giskard::core::DoubleConstSpec());
  trans_x->set_value(-0.1);
  trans_y->set_value(-0.2);
  trans_z->set_value(-0.3);
  giskard::core::VectorConstructorSpecPtr trans(new giskard::core::VectorConstructorSpec());
  trans->set(trans_x, trans_y, trans_z);

  giskard::core::FrameConstructorSpec spec;
  spec.set_translation(trans);
  spec.set_rotation(rot);

  giskard::core::Scope scope;

  KDL::Frame frame1 = spec.get_expression(scope)->value();
  KDL::Frame frame2 = KDL::Frame(KDL::Rotation::Rot(KDL::Vector(1.0, 2.0, 3.0), -4.0),
      KDL::Vector(-0.1, -0.2, -0.3));

  EXPECT_TRUE(KDL::Equal(frame1, frame2));
};

TEST_F(FrameGenerationTest, ConstructorEquality)
{
  giskard::core::DoubleConstSpecPtr one = giskard::core::DoubleConstSpecPtr(new giskard::core::DoubleConstSpec());
  giskard::core::DoubleConstSpecPtr zero = giskard::core::DoubleConstSpecPtr(new giskard::core::DoubleConstSpec());
  giskard::core::DoubleConstSpecPtr pi = giskard::core::DoubleConstSpecPtr(new giskard::core::DoubleConstSpec());
  one->set_value(0.0);
  zero->set_value(1.0);
  pi->set_value(M_PI);

  giskard::core::VectorConstructorSpecPtr trans1 = 
      giskard::core::VectorConstructorSpecPtr(new giskard::core::VectorConstructorSpec());
  giskard::core::VectorConstructorSpecPtr trans2 = 
      giskard::core::VectorConstructorSpecPtr(new giskard::core::VectorConstructorSpec());
  giskard::core::VectorConstructorSpecPtr axis1 = 
      giskard::core::VectorConstructorSpecPtr(new giskard::core::VectorConstructorSpec());
  giskard::core::VectorConstructorSpecPtr axis2 = 
      giskard::core::VectorConstructorSpecPtr(new giskard::core::VectorConstructorSpec());
  trans1->set(one, one, one);
  trans2->set(zero, zero, one);
  axis1->set(one, zero, zero);
  axis2->set(zero, one, zero);

  giskard::core::AxisAngleSpecPtr rot1 =
      giskard::core::AxisAngleSpecPtr(new giskard::core::AxisAngleSpec());
  giskard::core::AxisAngleSpecPtr rot2 =
      giskard::core::AxisAngleSpecPtr(new giskard::core::AxisAngleSpec());

  rot1->set_axis(axis1);
  rot1->set_angle(zero);
  rot2->set_axis(axis2);
  rot2->set_angle(pi);
  
  giskard::core::FrameConstructorSpec d1, d2, d3, d4, d5;

  d1.set_rotation(rot1);
  d1.set_translation(trans1);
  d2.set_rotation(rot1);
  d2.set_translation(trans2);
  d3.set_rotation(rot2);
  d3.set_translation(trans1);
  d4.set_rotation(rot2);
  d4.set_translation(trans2);
  d5.set_rotation(rot1);
  d5.set_translation(trans1);

  EXPECT_TRUE(d1.equals(d1));
  EXPECT_FALSE(d1.equals(d2));
  EXPECT_FALSE(d1.equals(d3));
  EXPECT_FALSE(d1.equals(d4));
  EXPECT_TRUE(d1.equals(d5));

  EXPECT_EQ(d1, d1);
  EXPECT_NE(d1, d2);
  EXPECT_NE(d1, d3);
  EXPECT_NE(d1, d4);
  EXPECT_EQ(d1, d5);
}

TEST_F(FrameGenerationTest, Multiplication)
{
  giskard::core::DoubleConstSpecPtr one = giskard::core::DoubleConstSpecPtr(new giskard::core::DoubleConstSpec());
  giskard::core::DoubleConstSpecPtr zero = giskard::core::DoubleConstSpecPtr(new giskard::core::DoubleConstSpec());
  giskard::core::DoubleConstSpecPtr pi = giskard::core::DoubleConstSpecPtr(new giskard::core::DoubleConstSpec());
  one->set_value(1.0);
  zero->set_value(0.0);
  pi->set_value(M_PI);

  giskard::core::VectorConstructorSpecPtr trans1 = 
      giskard::core::VectorConstructorSpecPtr(new giskard::core::VectorConstructorSpec());
  giskard::core::VectorConstructorSpecPtr trans2 = 
      giskard::core::VectorConstructorSpecPtr(new giskard::core::VectorConstructorSpec());
  giskard::core::VectorConstructorSpecPtr axis1 = 
      giskard::core::VectorConstructorSpecPtr(new giskard::core::VectorConstructorSpec());
  giskard::core::VectorConstructorSpecPtr axis2 = 
      giskard::core::VectorConstructorSpecPtr(new giskard::core::VectorConstructorSpec());
  trans1->set(one, one, one);
  trans2->set(zero, zero, one);
  axis1->set(one, zero, zero);
  axis2->set(zero, one, zero);

  giskard::core::AxisAngleSpecPtr rot1 =
      giskard::core::AxisAngleSpecPtr(new giskard::core::AxisAngleSpec());
  giskard::core::AxisAngleSpecPtr rot2 =
      giskard::core::AxisAngleSpecPtr(new giskard::core::AxisAngleSpec());

  rot1->set_axis(axis1);
  rot1->set_angle(zero);
  rot2->set_axis(axis2);
  rot2->set_angle(pi);
  
  giskard::core::FrameConstructorSpecPtr d1 = 
      giskard::core::FrameConstructorSpecPtr(new giskard::core::FrameConstructorSpec());
  giskard::core::FrameConstructorSpecPtr d2 = 
      giskard::core::FrameConstructorSpecPtr(new giskard::core::FrameConstructorSpec());

  d1->set_rotation(rot1);
  d1->set_translation(trans1);
  d2->set_rotation(rot2);
  d2->set_translation(trans2);

  std::vector<giskard::core::FrameSpecPtr> in;
  in.push_back(d1);
  in.push_back(d2);

  giskard::core::FrameMultiplicationSpec m;
  m.set_inputs(in);

  KDL::Frame f1 = KDL::Frame(KDL::Rotation::Rot(KDL::Vector(1.0, 0.0, 0.0), 0.0), KDL::Vector(1.0, 1.0, 1.0));
  KDL::Frame f2 = KDL::Frame(KDL::Rotation::Rot(KDL::Vector(0.0, 1.0, 0.0), M_PI), KDL::Vector(0.0, 0.0, 1.0));

  giskard::core::Scope scope;
  KDL::Expression<KDL::Frame>::Ptr exp = m.get_expression(scope);

  EXPECT_TRUE(KDL::Equal(exp->value(), f1*f2));
}
  
TEST_F(FrameGenerationTest, MultiplicationEquality)
{
  giskard::core::DoubleConstSpecPtr one = giskard::core::DoubleConstSpecPtr(new giskard::core::DoubleConstSpec());
  giskard::core::DoubleConstSpecPtr zero = giskard::core::DoubleConstSpecPtr(new giskard::core::DoubleConstSpec());
  giskard::core::DoubleConstSpecPtr pi = giskard::core::DoubleConstSpecPtr(new giskard::core::DoubleConstSpec());
  one->set_value(0.0);
  zero->set_value(1.0);
  pi->set_value(M_PI);

  giskard::core::VectorConstructorSpecPtr trans1 = 
      giskard::core::VectorConstructorSpecPtr(new giskard::core::VectorConstructorSpec());
  giskard::core::VectorConstructorSpecPtr trans2 = 
      giskard::core::VectorConstructorSpecPtr(new giskard::core::VectorConstructorSpec());
  giskard::core::VectorConstructorSpecPtr axis1 = 
      giskard::core::VectorConstructorSpecPtr(new giskard::core::VectorConstructorSpec());
  giskard::core::VectorConstructorSpecPtr axis2 = 
      giskard::core::VectorConstructorSpecPtr(new giskard::core::VectorConstructorSpec());
  trans1->set(one, one, one);
  trans2->set(zero, zero, one);
  axis1->set(one, zero, zero);
  axis2->set(zero, one, zero);

  giskard::core::AxisAngleSpecPtr rot1 =
      giskard::core::AxisAngleSpecPtr(new giskard::core::AxisAngleSpec());
  giskard::core::AxisAngleSpecPtr rot2 =
      giskard::core::AxisAngleSpecPtr(new giskard::core::AxisAngleSpec());
  
  rot1->set_axis(axis1);
  rot1->set_angle(zero);
  rot2->set_axis(axis2);
  rot2->set_angle(pi);

  giskard::core::FrameConstructorSpecPtr d1 = 
      giskard::core::FrameConstructorSpecPtr(new giskard::core::FrameConstructorSpec());
  giskard::core::FrameConstructorSpecPtr d2 = 
      giskard::core::FrameConstructorSpecPtr(new giskard::core::FrameConstructorSpec());

  d1->set_rotation(rot1);
  d1->set_translation(trans1);
  d2->set_rotation(rot2);
  d2->set_translation(trans2);

  std::vector<giskard::core::FrameSpecPtr> in1, in2, in3, in4, in5;
  in1.push_back(d1);
  in2.push_back(d2);
  in3.push_back(d1);
  in3.push_back(d1);
  in4.push_back(d1);
  in4.push_back(d2);
  in5.push_back(d1);

  giskard::core::FrameMultiplicationSpec a1, a2, a3, a4, a5;
  a1.set_inputs(in1);
  a2.set_inputs(in2);
  a3.set_inputs(in3);
  a4.set_inputs(in4);
  a5.set_inputs(in5);

  EXPECT_TRUE(a1.equals(a1));
  EXPECT_FALSE(a1.equals(a2));
  EXPECT_FALSE(a1.equals(a3));
  EXPECT_FALSE(a1.equals(a4));
  EXPECT_TRUE(a1.equals(a5));

  EXPECT_EQ(a1, a1);
  EXPECT_NE(a1, a2);
  EXPECT_NE(a1, a3);
  EXPECT_NE(a1, a4);
  EXPECT_EQ(a1, a5);
}

TEST_F(FrameGenerationTest, Cached)
{
  std::string s = "cached-frame: {frame: [{axis-angle: [{vector3: [1,0,0]}, 0.5]}, {vector3: [0.1, 0.2, 0.3]}]}";
  YAML::Node node = YAML::Load(s);

  ASSERT_NO_THROW(node.as<giskard::core::FrameSpecPtr>());
  giskard::core::FrameSpecPtr spec = node.as<giskard::core::FrameSpecPtr>();

  ASSERT_NO_THROW(spec->get_expression(giskard::core::Scope()));  
  KDL::Expression<KDL::Frame>::Ptr exp = spec->get_expression(giskard::core::Scope());

  KDL::Frame frame(KDL::Rotation::RotX(0.5), KDL::Vector(0.1, 0.2, 0.3));

  EXPECT_TRUE(KDL::Equal(frame, exp->value()));
}
