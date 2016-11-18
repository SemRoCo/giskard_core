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

using namespace giskard;
using namespace KDL;

class RotationGenerationTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
    }

    virtual void TearDown(){}
};

TEST_F(RotationGenerationTest, AxisAngle)
{
  giskard::DoubleConstSpecPtr x(new giskard::DoubleConstSpec());
  x->set_value(1.0);
  giskard::DoubleConstSpecPtr y(new giskard::DoubleConstSpec());
  y->set_value(0.0);
  giskard::DoubleConstSpecPtr z(new giskard::DoubleConstSpec());
  z->set_value(0.0);
  giskard::DoubleConstSpecPtr angle(new giskard::DoubleConstSpec());
  angle->set_value(M_PI/2.0);

  giskard::VectorConstructorSpecPtr axis(new giskard::VectorConstructorSpec());
  axis->set(x, y, z);
 
  giskard::AxisAngleSpec spec;
  spec.set_axis(axis);
  spec.set_angle(angle);

  giskard::Scope scope;
  KDL::Rotation rot = spec.get_expression(scope)->value();
  KDL::Rotation rot2 = KDL::Rotation::Rot(KDL::Vector(1.0, 0.0, 0.0), M_PI/2.0);

  EXPECT_TRUE(KDL::Equal(rot, rot2));
}

TEST_F(RotationGenerationTest, AxisAngleEquality)
{
  VectorConstructorSpecPtr axis1 = 
    vector_constructor_spec(double_const_spec(1.0), double_const_spec(0.0), double_const_spec(0.0));
  VectorConstructorSpecPtr axis2 = 
    vector_constructor_spec(double_const_spec(1.0), double_const_spec(1.0), double_const_spec(1.0));
  DoubleConstSpecPtr angle = double_const_spec(M_PI/2.0);
  DoubleConstSpecPtr x = double_const_spec(1.0);

  AxisAngleSpec s1, s2, s3, s4;
  s1.set_axis(axis1);
  s1.set_angle(angle);
  s2.set_axis(axis1);
  s2.set_angle(x);
  s3.set_axis(axis2);
  s3.set_angle(angle);
  s4.set_axis(axis1);
  s4.set_angle(angle);

  EXPECT_TRUE(s1.equals(s1));
  EXPECT_FALSE(s1.equals(s2));
  EXPECT_FALSE(s1.equals(s3));
  EXPECT_TRUE(s1.equals(s4));

  EXPECT_EQ(s1, s1);
  EXPECT_NE(s1, s2);
  EXPECT_NE(s1, s3);
  EXPECT_EQ(s1, s4);
}

TEST_F(RotationGenerationTest, QuaternionConstructor)
{
  giskard::RotationQuaternionConstructorSpec spec;
  spec.set_x(0.0);
  spec.set_y(0.70710678118);
  spec.set_z(0.0);
  spec.set_w(-0.70710678118);

  giskard::Scope scope;
  KDL::Rotation rot = spec.get_expression(scope)->value();
  KDL::Rotation rot2 = KDL::Rotation::Quaternion(0.0, 0.70710678118, 0.0, -0.70710678118);

  EXPECT_TRUE(KDL::Equal(rot, rot2));
}

TEST_F(RotationGenerationTest, QuaternionConstructorEquality)
{
  giskard::RotationQuaternionConstructorSpec s1, s2, s3, s4;
  s1.set_x(0.0);
  s1.set_y(0.70710678118);
  s1.set_z(0.0);
  s1.set_w(-0.70710678118);
 
  s2.set_x(0.0);
  s2.set_y(0.70710678118);
  s2.set_z(0.0);
  s2.set_w(-0.70710678118);
 
  s3.set_y(0.0);
  s3.set_x(0.70710678118);
  s3.set_z(0.0);
  s3.set_w(-0.70710678118);
 
  s4.set_x(0.0);
  s4.set_y(0.70710678118);
  s4.set_w(0.0);
  s4.set_z(-0.70710678118);
 
  EXPECT_TRUE(s1.equals(s1));
  EXPECT_TRUE(s1.equals(s2));
  EXPECT_TRUE(s2.equals(s1));
  EXPECT_FALSE(s1.equals(s3));
  EXPECT_FALSE(s1.equals(s4));
  EXPECT_FALSE(s3.equals(s4));
  EXPECT_FALSE(s4.equals(s3));

  EXPECT_EQ(s1, s1);
  EXPECT_EQ(s1, s2);
  EXPECT_EQ(s2, s1);
  EXPECT_NE(s1, s3);
  EXPECT_NE(s1, s4);
  EXPECT_NE(s3, s4);
  EXPECT_NE(s4, s3);
}

void test_rotation_generation(const RotationSpecPtr& spec, const KDL::Rotation& rot) 
{
  ASSERT_TRUE(spec.get() != NULL);
  ASSERT_NO_THROW(spec->get_expression(Scope()));
  Expression<Rotation>::Ptr exp = spec->get_expression(Scope());
  EXPECT_TRUE(Equal(rot, exp->value()));
}

TEST_F(RotationGenerationTest, OrientationOf)
{
  test_rotation_generation(orientation_of_spec(frame_constructor_spec(
      vector_constructor_spec(), quaternion_spec(0.0, 1.0, 0.0, 0.0))), 
      KDL::Rotation::Quaternion(0.0, 1.0, 0.0, 0.0));

  test_rotation_generation(orientation_of_spec(frame_constructor_spec(
      vector_constructor_spec(), quaternion_spec(1.0, 0.0, 0.0, 0.0))), 
      KDL::Rotation::Quaternion(1.0, 0.0, 0.0, 0.0));

  test_rotation_generation(orientation_of_spec(frame_constructor_spec(
      vector_constructor_spec(), quaternion_spec(0.0, 0.0, 0.0, 1.0))), 
      KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0));

  test_rotation_generation(orientation_of_spec(frame_constructor_spec(
      vector_constructor_spec(), quaternion_spec(0.0, 0.0, 0.1, 0.0))), 
      KDL::Rotation::Quaternion(0.0, 0.0, 0.1, 0.0));
}

TEST_F(RotationGenerationTest, InverseRotation)
{
  test_rotation_generation(inverse_rotation_spec(quaternion_spec(0.0, 1.0, 0.0, 0.0)), 
      KDL::Rotation::Quaternion(0.0, 1.0, 0.0, 0.0).Inverse());

  test_rotation_generation(inverse_rotation_spec(quaternion_spec(1.0, 0.0, 0.0, 0.0)), 
      KDL::Rotation::Quaternion(1.0, 0.0, 0.0, 0.0).Inverse());

  test_rotation_generation(inverse_rotation_spec(quaternion_spec(0.0, 0.0, 0.0, 1.0)), 
      KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0).Inverse());

  test_rotation_generation(inverse_rotation_spec(quaternion_spec(0.0, 0.0, 0.1, 0.0)), 
      KDL::Rotation::Quaternion(0.0, 0.0, 0.1, 0.0).Inverse());
}

TEST_F(RotationGenerationTest, RotationMultiplication)
{
  std::vector<RotationSpecPtr> rots1, rots2;
  rots1.push_back(quaternion_spec(0.0, 1.0, 0.0, 0.0));
  rots1.push_back(quaternion_spec(1.0, 0.0, 0.0, 0.0));

  rots2 = rots1;
  rots2.push_back(quaternion_spec(0.0, 0.0, 0.1, 0.0));

  KDL::Rotation rot1, rot2;
  rot1 = KDL::Rotation::Quaternion(0.0, 1.0, 0.0, 0.0) *
    KDL::Rotation::Quaternion(1.0, 0.0, 0.0, 0.0);

  rot2 = rot1 * KDL::Rotation::Quaternion(0.0, 0.0, 0.1, 0.0);

  test_rotation_generation(rotation_multiplication_spec(rots1), rot1);
  test_rotation_generation(rotation_multiplication_spec(rots2), rot2);
}

TEST_F(RotationGenerationTest, Slerp)
{
  std::string q1 = "{quaternion: [0.348, -0.52, 0.616, -0.479]}";
  std::string q2 = "{quaternion: [0.845, 0.262, 0.363, 0.293]}";
  std::string s1 = "{slerp: [" + q1 + ", " + q2 + ", 0]}";
  std::string s2 = "{slerp: [" + q1 + ", " + q2 + ", 0.5]}";
  std::string s3 = "{slerp: [" + q1 + ", " + q2 + ", 1]}";
  double eps = 0.001;

  // CASE 1
  YAML::Node node = YAML::Load(s1);
  ASSERT_NO_THROW(node.as<giskard::RotationSpecPtr>());
  RotationSpecPtr spec = node.as<giskard::RotationSpecPtr>();

  ASSERT_NO_THROW(spec->get_expression(giskard::Scope()));
  KDL::Expression<KDL::Rotation>::Ptr exp = spec->get_expression(giskard::Scope());
  ASSERT_TRUE(exp.get());
  KDL::Rotation r = exp->value();
  EXPECT_TRUE(KDL::Equal(r, KDL::Rotation::Quaternion(0.348, -0.52, 0.616, -0.479), eps));

  // CASE 2
  node = YAML::Load(s2);
  ASSERT_NO_THROW(node.as<giskard::RotationSpecPtr>());
  spec = node.as<giskard::RotationSpecPtr>();

  ASSERT_NO_THROW(spec->get_expression(giskard::Scope()));
  exp = spec->get_expression(giskard::Scope());
  ASSERT_TRUE(exp.get());
  r = exp->value();
  EXPECT_TRUE(KDL::Equal(r, KDL::Rotation::Quaternion(0.757226, -0.163759, 0.621395, -0.118059), eps));

  // CASE 3
  node = YAML::Load(s3);
  ASSERT_NO_THROW(node.as<giskard::RotationSpecPtr>());
  spec = node.as<giskard::RotationSpecPtr>();

  ASSERT_NO_THROW(spec->get_expression(giskard::Scope()));
  exp = spec->get_expression(giskard::Scope());
  ASSERT_TRUE(exp.get());
  r = exp->value();
  EXPECT_TRUE(KDL::Equal(r, KDL::Rotation::Quaternion(0.845, 0.262, 0.363, 0.293), eps));
}

TEST_F(RotationGenerationTest, ZeroAxisAngle)
{
  std::string s = "{axis-angle: [{vector3: [0,0,0]}, 1]}";
  YAML::Node node = YAML::Load(s);

  ASSERT_NO_THROW(node.as<giskard::RotationSpecPtr>());
  giskard::RotationSpecPtr spec = node.as<giskard::RotationSpecPtr>();

  ASSERT_NO_THROW(spec->get_expression(giskard::Scope()));  
  KDL::Expression<KDL::Rotation>::Ptr exp = spec->get_expression(giskard::Scope());

  using namespace KDL;
  std::cout << exp->value() << std::endl;
  EXPECT_TRUE(KDL::Equal(KDL::Rotation::Identity(), exp->value()));
}
