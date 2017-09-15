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


class VectorExpressionGenerationTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      x = giskard_core::DoubleConstSpecPtr(new giskard_core::DoubleConstSpec());
      y = giskard_core::DoubleConstSpecPtr(new giskard_core::DoubleConstSpec());
      z = giskard_core::DoubleConstSpecPtr(new giskard_core::DoubleConstSpec());

      x->set_value(1.1);
      y->set_value(2.2);
      z->set_value(3.3);
    }

    virtual void TearDown(){}

    giskard_core::DoubleConstSpecPtr x, y, z;
};

TEST_F(VectorExpressionGenerationTest, Constructor)
{
  giskard_core::VectorConstructorSpec descr;
  giskard_core::Scope scope;

  descr.set_x(x);
  descr.set_y(y);
  descr.set_z(z);

  KDL::Vector v = descr.get_expression(scope)->value();
  
  EXPECT_DOUBLE_EQ(v.x(), 1.1);
  EXPECT_DOUBLE_EQ(v.y(), 2.2);
  EXPECT_DOUBLE_EQ(v.z(), 3.3);
}

TEST_F(VectorExpressionGenerationTest, ConstructorEquality)
{
  giskard_core::VectorConstructorSpec d1, d2, d3;

  d1.set(x, y, z);
  d2.set(x, y, y);
  d3.set(x, y, z);

  EXPECT_TRUE(d1.equals(d1));
  EXPECT_FALSE(d1.equals(d2));
  EXPECT_TRUE(d1.equals(d3));

  EXPECT_EQ(d1, d1);
  EXPECT_NE(d1, d2);
  EXPECT_EQ(d1, d3);

  EXPECT_FALSE(d2.equals(d1));
  EXPECT_TRUE(d2.equals(d2));
  EXPECT_FALSE(d2.equals(d3));

  EXPECT_TRUE(d3.equals(d1));
  EXPECT_FALSE(d3.equals(d2));
  EXPECT_TRUE(d3.equals(d3));
}

TEST_F(VectorExpressionGenerationTest, VectorAddition)
{
  std::string s1 = "{vector-add: []}";
  std::string s2 = "{vector-add: [{vector3: [1, 0, 0]}]}";
  std::string s3 = "{vector-add: [{vector3: [1, 0, 0]}, {vector3: [0, 2, 0]}, {vector3: [0, 0, 3]}]}";

  // test 1
  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard_core::VectorSpecPtr>());
  giskard_core::VectorSpecPtr spec = node.as<giskard_core::VectorSpecPtr>();
  
  giskard_core::Scope scope;
  ASSERT_NO_THROW(spec->get_expression(scope));
  KDL::Expression<KDL::Vector>::Ptr exp = spec->get_expression(scope);
  
  ASSERT_TRUE(exp.get());
  KDL::Vector val1 = exp->value();
  KDL::Vector val2 = KDL::Vector(0,0,0);
  EXPECT_TRUE(KDL::Equal(val1, val2));

  // test 2
  node = YAML::Load(s2);

  ASSERT_NO_THROW(node.as<giskard_core::VectorSpecPtr>());
  spec = node.as<giskard_core::VectorSpecPtr>();
  
  ASSERT_NO_THROW(spec->get_expression(scope));
  exp = spec->get_expression(scope);
  
  ASSERT_TRUE(exp.get());
  val1 = exp->value();
  val2 = KDL::Vector(1,0,0);
  EXPECT_TRUE(KDL::Equal(val1, val2));

  // test 3
  node = YAML::Load(s3);

  ASSERT_NO_THROW(node.as<giskard_core::VectorSpecPtr>());
  spec = node.as<giskard_core::VectorSpecPtr>();
  
  ASSERT_NO_THROW(spec->get_expression(scope));
  exp = spec->get_expression(scope);
  
  ASSERT_TRUE(exp.get());
  val1 = exp->value();
  val2 = KDL::Vector(1,2,3);
  EXPECT_TRUE(KDL::Equal(val1, val2));
}

TEST_F(VectorExpressionGenerationTest, VectorCross)
{
  std::string s1 = "{vector-cross: [{vector3: [1, 0, 0]}, {vector3: [0, 1, 0]}]}";

  // test 1
  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard_core::VectorSpecPtr>());
  giskard_core::VectorSpecPtr spec = node.as<giskard_core::VectorSpecPtr>();
  
  giskard_core::Scope scope;
  ASSERT_NO_THROW(spec->get_expression(scope));
  KDL::Expression<KDL::Vector>::Ptr exp = spec->get_expression(scope);
  
  ASSERT_TRUE(exp.get());
  KDL::Vector val1 = exp->value();
  KDL::Vector val2 = KDL::Vector(0,0,1);
  EXPECT_TRUE(KDL::Equal(val1, val2));
}

TEST_F(VectorExpressionGenerationTest, VectorSubtraction)
{
  std::string v1 = "{vector3: [1.1, 2.2, 3.3]}";
  std::string v2 = "{vector3: [-0.5, 0.5, 1.5]}";

  std::string s1 = "{vector-sub: [" + v1 + "]}";
  std::string s2 = "{vector-sub: [" + v1 + ", " + v2 + "]}";
  std::string s3 = "{vector-sub: [" + v1 + ", " + v2 + ", " + v2 + "]}";

  // test 1
  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard_core::VectorSpecPtr>());
  giskard_core::VectorSpecPtr spec = node.as<giskard_core::VectorSpecPtr>();
  
  giskard_core::Scope scope;
  ASSERT_NO_THROW(spec->get_expression(scope));
  KDL::Expression<KDL::Vector>::Ptr exp = spec->get_expression(scope);
  
  ASSERT_TRUE(exp.get());
  KDL::Vector val1 = exp->value();
  KDL::Vector val2 = KDL::Vector(-1.1, -2.2, -3.3);
  EXPECT_TRUE(KDL::Equal(val1, val2));

  // test 2
  node = YAML::Load(s2);

  ASSERT_NO_THROW(node.as<giskard_core::VectorSpecPtr>());
  spec = node.as<giskard_core::VectorSpecPtr>();
  
  ASSERT_NO_THROW(spec->get_expression(scope));
  exp = spec->get_expression(scope);
  
  ASSERT_TRUE(exp.get());
  val1 = exp->value();
  val2 = KDL::Vector(1.6, 1.7, 1.8);
  EXPECT_TRUE(KDL::Equal(val1, val2));

  // test 3
  node = YAML::Load(s3);

  ASSERT_NO_THROW(node.as<giskard_core::VectorSpecPtr>());
  spec = node.as<giskard_core::VectorSpecPtr>();
  
  ASSERT_NO_THROW(spec->get_expression(scope));
  exp = spec->get_expression(scope);
  
  ASSERT_TRUE(exp.get());
  val1 = exp->value();
  val2 = KDL::Vector(2.1, 1.2, 0.3);
  EXPECT_TRUE(KDL::Equal(val1, val2));
}

TEST_F(VectorExpressionGenerationTest, VectorFrameMultiplication)
{
  std::string v1 = "{vector3: [0.1, 0.2, 0.3]}";
  std::string v2 = "{vector3: [1, 2, 3]}";
  std::string r = "{axis-angle: [{vector3: [1, 0, 0]}, 0]}";
  std::string f = "{frame: [" + r + ", " + v1 + "]}";
  std::string s1 = "{transform-vector: [" + f + ", " + v2 + "]}";

  YAML::Node node = YAML::Load(s1);
  ASSERT_NO_THROW(node.as<giskard_core::VectorSpecPtr>());
  giskard_core::VectorSpecPtr spec = node.as<giskard_core::VectorSpecPtr>();
  
  ASSERT_NO_THROW(spec->get_expression(giskard_core::Scope()));
  KDL::Expression<KDL::Vector>::Ptr exp = spec->get_expression(giskard_core::Scope());
  
  ASSERT_TRUE(exp.get());
  KDL::Vector val1 = exp->value();
  KDL::Vector val2 = KDL::Vector(1.1, 2.2, 3.3);
  EXPECT_TRUE(KDL::Equal(val1, val2));
}

TEST_F(VectorExpressionGenerationTest, VectorDoubleMultiplication)
{
  std::string s1 = "{scale-vector: [0.5, {vector3: [1, 2, 3]}]}";

  YAML::Node node = YAML::Load(s1);
  ASSERT_NO_THROW(node.as<giskard_core::VectorSpecPtr>());
  giskard_core::VectorSpecPtr spec = node.as<giskard_core::VectorSpecPtr>();
  
  ASSERT_NO_THROW(spec->get_expression(giskard_core::Scope()));
  KDL::Expression<KDL::Vector>::Ptr exp = spec->get_expression(giskard_core::Scope());
  
  ASSERT_TRUE(exp.get());
  KDL::Vector val1 = exp->value();
  KDL::Vector val2 = KDL::Vector(0.5, 1.0, 1.5);
  EXPECT_TRUE(KDL::Equal(val1, val2));
}

TEST_F(VectorExpressionGenerationTest, ProjectPointOnPlane)
{
  YAML::Node node = YAML::LoadFile("project_point_into_plane.yaml");
  ASSERT_NO_THROW(node.as<giskard_core::ScopeSpec>());
  giskard_core::ScopeSpec spec = node.as<giskard_core::ScopeSpec>();
  
  ASSERT_NO_THROW(giskard_core::generate(spec));
  giskard_core::Scope scope = giskard_core::generate(spec);

  ASSERT_TRUE(scope.has_vector_expression("projected-point"));
  KDL::Expression<KDL::Vector>::Ptr exp = scope.find_vector_expression("projected-point");
  EXPECT_TRUE(KDL::Equal(exp->value(), KDL::Vector(1, 1, 12)));

  ASSERT_TRUE(scope.has_vector_expression("projected-point2"));
  exp = scope.find_vector_expression("projected-point2");
  EXPECT_TRUE(KDL::Equal(exp->value(), KDL::Vector(2,-1.5,1.5)));

  ASSERT_TRUE(scope.has_vector_expression("projected-point3"));
  exp = scope.find_vector_expression("projected-point3");
  EXPECT_TRUE(KDL::Equal(exp->value(), KDL::Vector(25, 0, 8)));

  ASSERT_TRUE(scope.has_vector_expression("circle-point"));
  exp = scope.find_vector_expression("circle-point");
  EXPECT_TRUE(KDL::Equal(exp->value(), KDL::Vector(2.5, 0, 8)));
}

TEST_F(VectorExpressionGenerationTest, CachedVector)
{
  std::string s = "cached-vector: {vector3: [-0.1, -0.2, -0.3]}";
  YAML::Node node = YAML::Load(s);

  ASSERT_NO_THROW(node.as<giskard_core::VectorSpecPtr>());
  giskard_core::VectorSpecPtr spec = node.as<giskard_core::VectorSpecPtr>();

  ASSERT_NO_THROW(spec->get_expression(giskard_core::Scope()));  
  KDL::Expression<KDL::Vector>::Ptr exp = spec->get_expression(giskard_core::Scope());

  KDL::Vector vec(-0.1, -0.2, -0.3);

  EXPECT_TRUE(KDL::Equal(vec, exp->value()));
}

void test_rot_vector(const std::string& s, const KDL::Vector& v)
{
  YAML::Node node = YAML::Load(s);

  ASSERT_NO_THROW(node.as<giskard_core::VectorSpecPtr>());
  giskard_core::VectorSpecPtr spec = node.as<giskard_core::VectorSpecPtr>();

  ASSERT_NO_THROW(spec->get_expression(giskard_core::Scope()));  
  KDL::Expression<KDL::Vector>::Ptr exp = spec->get_expression(giskard_core::Scope());

  EXPECT_TRUE(KDL::Equal(v, exp->value(), 0.00001));
}

TEST_F(VectorExpressionGenerationTest, RotationVector)
{
  test_rot_vector("{rot-vector: {quaternion: [0.0, 0.0, 0.0, 1.0]}}",
      KDL::Vector());
  test_rot_vector("{rot-vector: {quaternion: [1.0, 0.0, 0.0, 0.0]}}",
      KDL::Vector(M_PI, 0, 0));
  test_rot_vector("{rot-vector: {quaternion: [0.0, 0.707107, 0.0, 0.707107]}}",
      KDL::Vector(0, M_PI/2, 0));
  test_rot_vector("{rot-vector: {quaternion: [0.0, 0.0, -0.382683, 0.92388]}}",
      KDL::Vector(0, 0, -M_PI/4));
}

TEST_F(VectorExpressionGenerationTest, VectorRotationMultiplication)
{
  std::string v = "{vector3: [0.1, -1.2, 0.03]}";
  std::string r = "{quaternion: [0.348, -0.52, 0.616, -0.479]}";
  std::string s = "{rotate-vector: [" + r + ", " + v + "]}";

  YAML::Node node = YAML::Load(s);
  ASSERT_NO_THROW(node.as<giskard_core::VectorSpecPtr>());
  giskard_core::VectorSpecPtr spec = node.as<giskard_core::VectorSpecPtr>();
  
  ASSERT_NO_THROW(spec->get_expression(giskard_core::Scope()));
  KDL::Expression<KDL::Vector>::Ptr exp = spec->get_expression(giskard_core::Scope());
  
  ASSERT_TRUE(exp.get());
  using namespace KDL;
  Vector result = Rotation::Quaternion(0.348, -0.52, 0.616, -0.479) * Vector(0.1, -1.2, 0.03);
  EXPECT_TRUE(Equal(exp->value(), result));
}
