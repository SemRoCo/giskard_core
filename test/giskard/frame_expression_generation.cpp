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
  giskard::DoubleConstSpecPtr rot_axis_x(new giskard::DoubleConstSpec());
  rot_axis_x->set_value(1.0);
  giskard::DoubleConstSpecPtr rot_axis_y(new giskard::DoubleConstSpec());
  rot_axis_y->set_value(2.0);
  giskard::DoubleConstSpecPtr rot_axis_z(new giskard::DoubleConstSpec());
  rot_axis_z->set_value(3.0);
  giskard::VectorConstructorSpecPtr rot_axis(new giskard::VectorConstructorSpec());
  rot_axis->set(rot_axis_x, rot_axis_y, rot_axis_z);

  giskard::DoubleConstSpecPtr rot_angle(new giskard::DoubleConstSpec());
  rot_angle->set_value(-4.0);

  giskard::AxisAngleSpecPtr rot(new giskard::AxisAngleSpec());
  rot->set_axis(rot_axis);
  rot->set_angle(rot_angle);

  giskard::DoubleConstSpecPtr trans_x(new giskard::DoubleConstSpec());
  giskard::DoubleConstSpecPtr trans_y(new giskard::DoubleConstSpec());
  giskard::DoubleConstSpecPtr trans_z(new giskard::DoubleConstSpec());
  trans_x->set_value(-0.1);
  trans_y->set_value(-0.2);
  trans_z->set_value(-0.3);
  giskard::VectorConstructorSpecPtr trans(new giskard::VectorConstructorSpec());
  trans->set(trans_x, trans_y, trans_z);

  giskard::FrameConstructorSpec spec;
  spec.set_translation(trans);
  spec.set_rotation(rot);

  giskard::Scope scope;

  KDL::Frame frame1 = spec.get_expression(scope)->value();
  KDL::Frame frame2 = KDL::Frame(KDL::Rotation::Rot2(KDL::Vector(1.0, 2.0, 3.0), -4.0),
      KDL::Vector(-0.1, -0.2, -0.3));

  EXPECT_TRUE(KDL::Equal(frame1, frame2));
};

TEST_F(FrameGenerationTest, ConstructorEquality)
{
  giskard::DoubleConstSpecPtr one = giskard::DoubleConstSpecPtr(new giskard::DoubleConstSpec());
  giskard::DoubleConstSpecPtr zero = giskard::DoubleConstSpecPtr(new giskard::DoubleConstSpec());
  giskard::DoubleConstSpecPtr pi = giskard::DoubleConstSpecPtr(new giskard::DoubleConstSpec());
  one->set_value(0.0);
  zero->set_value(1.0);
  pi->set_value(M_PI);

  giskard::VectorConstructorSpecPtr trans1 = 
      giskard::VectorConstructorSpecPtr(new giskard::VectorConstructorSpec());
  giskard::VectorConstructorSpecPtr trans2 = 
      giskard::VectorConstructorSpecPtr(new giskard::VectorConstructorSpec());
  giskard::VectorConstructorSpecPtr axis1 = 
      giskard::VectorConstructorSpecPtr(new giskard::VectorConstructorSpec());
  giskard::VectorConstructorSpecPtr axis2 = 
      giskard::VectorConstructorSpecPtr(new giskard::VectorConstructorSpec());
  trans1->set(one, one, one);
  trans2->set(zero, zero, one);
  axis1->set(one, zero, zero);
  axis2->set(zero, one, zero);

  giskard::AxisAngleSpecPtr rot1 =
      giskard::AxisAngleSpecPtr(new giskard::AxisAngleSpec());
  giskard::AxisAngleSpecPtr rot2 =
      giskard::AxisAngleSpecPtr(new giskard::AxisAngleSpec());

  rot1->set_axis(axis1);
  rot1->set_angle(zero);
  rot2->set_axis(axis2);
  rot2->set_angle(pi);
  
  giskard::FrameConstructorSpec d1, d2, d3, d4, d5;

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
  giskard::DoubleConstSpecPtr one = giskard::DoubleConstSpecPtr(new giskard::DoubleConstSpec());
  giskard::DoubleConstSpecPtr zero = giskard::DoubleConstSpecPtr(new giskard::DoubleConstSpec());
  giskard::DoubleConstSpecPtr pi = giskard::DoubleConstSpecPtr(new giskard::DoubleConstSpec());
  one->set_value(1.0);
  zero->set_value(0.0);
  pi->set_value(M_PI);

  giskard::VectorConstructorSpecPtr trans1 = 
      giskard::VectorConstructorSpecPtr(new giskard::VectorConstructorSpec());
  giskard::VectorConstructorSpecPtr trans2 = 
      giskard::VectorConstructorSpecPtr(new giskard::VectorConstructorSpec());
  giskard::VectorConstructorSpecPtr axis1 = 
      giskard::VectorConstructorSpecPtr(new giskard::VectorConstructorSpec());
  giskard::VectorConstructorSpecPtr axis2 = 
      giskard::VectorConstructorSpecPtr(new giskard::VectorConstructorSpec());
  trans1->set(one, one, one);
  trans2->set(zero, zero, one);
  axis1->set(one, zero, zero);
  axis2->set(zero, one, zero);

  giskard::AxisAngleSpecPtr rot1 =
      giskard::AxisAngleSpecPtr(new giskard::AxisAngleSpec());
  giskard::AxisAngleSpecPtr rot2 =
      giskard::AxisAngleSpecPtr(new giskard::AxisAngleSpec());

  rot1->set_axis(axis1);
  rot1->set_angle(zero);
  rot2->set_axis(axis2);
  rot2->set_angle(pi);
  
  giskard::FrameConstructorSpecPtr d1 = 
      giskard::FrameConstructorSpecPtr(new giskard::FrameConstructorSpec());
  giskard::FrameConstructorSpecPtr d2 = 
      giskard::FrameConstructorSpecPtr(new giskard::FrameConstructorSpec());

  d1->set_rotation(rot1);
  d1->set_translation(trans1);
  d2->set_rotation(rot2);
  d2->set_translation(trans2);

  std::vector<giskard::FrameSpecPtr> in;
  in.push_back(d1);
  in.push_back(d2);

  giskard::FrameMultiplicationSpec m;
  m.set_inputs(in);

  KDL::Frame f1 = KDL::Frame(KDL::Rotation::Rot(KDL::Vector(1.0, 0.0, 0.0), 0.0), KDL::Vector(1.0, 1.0, 1.0));
  KDL::Frame f2 = KDL::Frame(KDL::Rotation::Rot(KDL::Vector(0.0, 1.0, 0.0), M_PI), KDL::Vector(0.0, 0.0, 1.0));

  giskard::Scope scope;
  KDL::Expression<KDL::Frame>::Ptr exp = m.get_expression(scope);

  EXPECT_TRUE(KDL::Equal(exp->value(), f1*f2));
}
  
TEST_F(FrameGenerationTest, MultiplicationEquality)
{
  giskard::DoubleConstSpecPtr one = giskard::DoubleConstSpecPtr(new giskard::DoubleConstSpec());
  giskard::DoubleConstSpecPtr zero = giskard::DoubleConstSpecPtr(new giskard::DoubleConstSpec());
  giskard::DoubleConstSpecPtr pi = giskard::DoubleConstSpecPtr(new giskard::DoubleConstSpec());
  one->set_value(0.0);
  zero->set_value(1.0);
  pi->set_value(M_PI);

  giskard::VectorConstructorSpecPtr trans1 = 
      giskard::VectorConstructorSpecPtr(new giskard::VectorConstructorSpec());
  giskard::VectorConstructorSpecPtr trans2 = 
      giskard::VectorConstructorSpecPtr(new giskard::VectorConstructorSpec());
  giskard::VectorConstructorSpecPtr axis1 = 
      giskard::VectorConstructorSpecPtr(new giskard::VectorConstructorSpec());
  giskard::VectorConstructorSpecPtr axis2 = 
      giskard::VectorConstructorSpecPtr(new giskard::VectorConstructorSpec());
  trans1->set(one, one, one);
  trans2->set(zero, zero, one);
  axis1->set(one, zero, zero);
  axis2->set(zero, one, zero);

  giskard::AxisAngleSpecPtr rot1 =
      giskard::AxisAngleSpecPtr(new giskard::AxisAngleSpec());
  giskard::AxisAngleSpecPtr rot2 =
      giskard::AxisAngleSpecPtr(new giskard::AxisAngleSpec());
  
  rot1->set_axis(axis1);
  rot1->set_angle(zero);
  rot2->set_axis(axis2);
  rot2->set_angle(pi);

  giskard::FrameConstructorSpecPtr d1 = 
      giskard::FrameConstructorSpecPtr(new giskard::FrameConstructorSpec());
  giskard::FrameConstructorSpecPtr d2 = 
      giskard::FrameConstructorSpecPtr(new giskard::FrameConstructorSpec());

  d1->set_rotation(rot1);
  d1->set_translation(trans1);
  d2->set_rotation(rot2);
  d2->set_translation(trans2);

  std::vector<giskard::FrameSpecPtr> in1, in2, in3, in4, in5;
  in1.push_back(d1);
  in2.push_back(d2);
  in3.push_back(d1);
  in3.push_back(d1);
  in4.push_back(d1);
  in4.push_back(d2);
  in5.push_back(d1);

  giskard::FrameMultiplicationSpec a1, a2, a3, a4, a5;
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

  ASSERT_NO_THROW(node.as<giskard::FrameSpecPtr>());
  giskard::FrameSpecPtr spec = node.as<giskard::FrameSpecPtr>();

  ASSERT_NO_THROW(spec->get_expression(giskard::Scope()));  
  KDL::Expression<KDL::Frame>::Ptr exp = spec->get_expression(giskard::Scope());

  KDL::Frame frame(KDL::Rotation::RotX(0.5), KDL::Vector(0.1, 0.2, 0.3));

  EXPECT_TRUE(KDL::Equal(frame, exp->value()));
}
