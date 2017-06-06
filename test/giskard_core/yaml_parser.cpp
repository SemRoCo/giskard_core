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
#include <boost/assign/list_of.hpp>
#include <boost/lexical_cast.hpp>

class YamlParserTest : public ::testing::Test
{
  protected:
    virtual void SetUp(){}
    virtual void TearDown(){}
};

TEST_F(YamlParserTest, ConstDoubleExpression)
{
  std::string c = "1.1";

  // parsing to const double spec
  YAML::Node node = YAML::Load(c);
  ASSERT_NO_THROW(node.as<giskard_core::DoubleConstSpecPtr>());
  giskard_core::DoubleConstSpecPtr s1 = node.as<giskard_core::DoubleConstSpecPtr>();

  EXPECT_DOUBLE_EQ(1.1, s1->get_value());

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard_core::DoubleConstSpecPtr>());
  giskard_core::DoubleConstSpecPtr s2 = node2.as<giskard_core::DoubleConstSpecPtr>();

  EXPECT_DOUBLE_EQ(s1->get_value(), s2->get_value());

  // parsing to double spec
  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr s3 = node.as<giskard_core::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s3).get());
  giskard_core::DoubleConstSpecPtr s4 = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s3);

  EXPECT_DOUBLE_EQ(1.1, s4->get_value());

  // roundtrip with generation to double spec
  YAML::Node node3;
  node3 = s3;
  ASSERT_NO_THROW(node3.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr s5 = node3.as<giskard_core::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s5).get());
  giskard_core::DoubleConstSpecPtr s6 = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s5);

  EXPECT_DOUBLE_EQ(1.1, s6->get_value());
};

TEST_F(YamlParserTest, InputExpression)
{
  std::string i = "{input-var: 2}";

  // parsing input double
  YAML::Node node = YAML::Load(i);
  ASSERT_NO_THROW(node.as<giskard_core::DoubleInputSpecPtr>());
  giskard_core::DoubleInputSpecPtr s1 = node.as<giskard_core::DoubleInputSpecPtr>();

  EXPECT_EQ(2, s1->get_input_num());

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard_core::DoubleInputSpecPtr>());
  giskard_core::DoubleInputSpecPtr s2 = node2.as<giskard_core::DoubleInputSpecPtr>();

  EXPECT_EQ(s1->get_input_num(), s2->get_input_num());

  // parsing to double spec
  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr s3 = node.as<giskard_core::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleInputSpec>(s3).get());
  giskard_core::DoubleInputSpecPtr s4 = boost::dynamic_pointer_cast<giskard_core::DoubleInputSpec>(s3);

  EXPECT_EQ(2, s4->get_input_num());

  // roundtrip with generation to double spec
  YAML::Node node3;
  node3 = s3;
  ASSERT_NO_THROW(node3.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr s5 = node3.as<giskard_core::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleInputSpec>(s5).get());
  giskard_core::DoubleInputSpecPtr s6 = boost::dynamic_pointer_cast<giskard_core::DoubleInputSpec>(s5);

  EXPECT_EQ(2, s6->get_input_num());
};

TEST_F(YamlParserTest, RotationVectorSpec)
{
  std::string v = "{rot-vector: {quaternion: [0.70710678118, 0.0, -0.70710678118, 0.0]}}";

  // parsing rotation vector spec
  YAML::Node node = YAML::Load(v);
  ASSERT_NO_THROW(node.as<giskard_core::VectorRotationVectorSpecPtr>());
  giskard_core::VectorRotationVectorSpecPtr s1 = node.as<giskard_core::VectorRotationVectorSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::RotationQuaternionConstructorSpec>(s1->get_rotation()).get());
  giskard_core::RotationQuaternionConstructorSpecPtr r1 =
    boost::dynamic_pointer_cast<giskard_core::RotationQuaternionConstructorSpec>(s1->get_rotation());

  EXPECT_DOUBLE_EQ(0.70710678118, r1->get_x());
  EXPECT_DOUBLE_EQ(0.0, r1->get_y());
  EXPECT_DOUBLE_EQ(-0.70710678118, r1->get_z());
  EXPECT_DOUBLE_EQ(0.0, r1->get_w());

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard_core::VectorRotationVectorSpecPtr>());
  giskard_core::VectorRotationVectorSpecPtr s2 = node2.as<giskard_core::VectorRotationVectorSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::RotationQuaternionConstructorSpec>(s2->get_rotation()).get());
  giskard_core::RotationQuaternionConstructorSpecPtr r2 =
    boost::dynamic_pointer_cast<giskard_core::RotationQuaternionConstructorSpec>(s2->get_rotation());

  EXPECT_DOUBLE_EQ(0.70710678118, r2->get_x());
  EXPECT_DOUBLE_EQ(0.0, r2->get_y());
  EXPECT_DOUBLE_EQ(-0.70710678118, r2->get_z());
  EXPECT_DOUBLE_EQ(0.0, r2->get_w());

  // parsing to vector spec
  ASSERT_NO_THROW(node.as<giskard_core::VectorSpecPtr>());
  giskard_core::VectorSpecPtr s3 = node.as<giskard_core::VectorSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorRotationVectorSpec>(s3).get());
  giskard_core::VectorRotationVectorSpecPtr s4 = boost::dynamic_pointer_cast<giskard_core::VectorRotationVectorSpec>(s3);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::RotationQuaternionConstructorSpec>(s4->get_rotation()).get());
  giskard_core::RotationQuaternionConstructorSpecPtr r3 =
    boost::dynamic_pointer_cast<giskard_core::RotationQuaternionConstructorSpec>(s4->get_rotation());

  EXPECT_DOUBLE_EQ(0.70710678118, r3->get_x());
  EXPECT_DOUBLE_EQ(0.0, r3->get_y());
  EXPECT_DOUBLE_EQ(-0.70710678118, r3->get_z());
  EXPECT_DOUBLE_EQ(0.0, r3->get_w());

  // roundtrip with generation to double spec
  YAML::Node node3;
  node3 = s3;
  ASSERT_NO_THROW(node3.as<giskard_core::VectorSpecPtr>());
  giskard_core::VectorSpecPtr s5 = node.as<giskard_core::VectorSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorRotationVectorSpec>(s5).get());
  giskard_core::VectorRotationVectorSpecPtr s6 = boost::dynamic_pointer_cast<giskard_core::VectorRotationVectorSpec>(s5);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::RotationQuaternionConstructorSpec>(s6->get_rotation()).get());
  giskard_core::RotationQuaternionConstructorSpecPtr r4 =
    boost::dynamic_pointer_cast<giskard_core::RotationQuaternionConstructorSpec>(s6->get_rotation());

  EXPECT_DOUBLE_EQ(0.70710678118, r4->get_x());
  EXPECT_DOUBLE_EQ(0.0, r4->get_y());
  EXPECT_DOUBLE_EQ(-0.70710678118, r4->get_z());
  EXPECT_DOUBLE_EQ(0.0, r4->get_w());
}
 
TEST_F(YamlParserTest, VectorConstructorSpec)
{
  std::string v = "{vector3: [1.1, 2.2, 3.3]}";

  // parsing constructor vector
  YAML::Node node = YAML::Load(v);
  ASSERT_NO_THROW(node.as<giskard_core::VectorConstructorSpecPtr>());
  giskard_core::VectorConstructorSpecPtr s1 = node.as<giskard_core::VectorConstructorSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s1->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s1->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s1->get_z()).get());

  giskard_core::DoubleConstSpecPtr x = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s1->get_x());
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  giskard_core::DoubleConstSpecPtr y = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s1->get_y());
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  giskard_core::DoubleConstSpecPtr z = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s1->get_z());
  EXPECT_DOUBLE_EQ(z->get_value(), 3.3);

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard_core::VectorConstructorSpecPtr>());
  giskard_core::VectorConstructorSpecPtr s2 = node2.as<giskard_core::VectorConstructorSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s2->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s2->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s2->get_z()).get());

  x = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s2->get_x());
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  y = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s2->get_y());
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  z = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s2->get_z());
  EXPECT_DOUBLE_EQ(z->get_value(), 3.3);

  // parsing to vector spec
  ASSERT_NO_THROW(node.as<giskard_core::VectorSpecPtr>());
  giskard_core::VectorSpecPtr s3 = node.as<giskard_core::VectorSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s3).get());
  giskard_core::VectorConstructorSpecPtr s4 = boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s3);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s4->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s4->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s4->get_z()).get());

  x = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s4->get_x());
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  y = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s4->get_y());
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  z = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s4->get_z());
  EXPECT_DOUBLE_EQ(z->get_value(), 3.3);

  // roundtrip with generation to double spec
  YAML::Node node3;
  node3 = s3;
  ASSERT_NO_THROW(node3.as<giskard_core::VectorSpecPtr>());
  giskard_core::VectorSpecPtr s5 = node3.as<giskard_core::VectorSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s5).get());
  giskard_core::VectorConstructorSpecPtr s6 = boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s5);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s6->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s6->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s6->get_z()).get());

  x = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s6->get_x());
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  y = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s6->get_y());
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  z = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(s6->get_z());
  EXPECT_DOUBLE_EQ(z->get_value(), 3.3);
};

TEST_F(YamlParserTest, AxisAngleSpec)
{
  std::string r = "{axis-angle: [{vector3: [1.0, 0.0, 0.0]}, {input-var: 3}]}"; 

  // parsing into axis angle specification
  YAML::Node node = YAML::Load(r);

  ASSERT_NO_THROW(node.as<giskard_core::AxisAngleSpecPtr>());
  giskard_core::AxisAngleSpecPtr s1 = node.as<giskard_core::AxisAngleSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleInputSpec>(s1->get_angle()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s1->get_axis()).get());

  giskard_core::DoubleInputSpecPtr angle = 
      boost::dynamic_pointer_cast<giskard_core::DoubleInputSpec>(s1->get_angle());
  giskard_core::VectorConstructorSpecPtr axis = 
      boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s1->get_axis());

  EXPECT_EQ(angle->get_input_num(), 3);

  ASSERT_TRUE(axis->get_x().get());
  ASSERT_TRUE(axis->get_y().get());
  ASSERT_TRUE(axis->get_z().get());

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_x()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_x())->get_value(), 1.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_y()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_y())->get_value(), 0.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_z())->get_value(), 0.0);

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;

  ASSERT_NO_THROW(node2.as<giskard_core::AxisAngleSpecPtr>());
  giskard_core::AxisAngleSpecPtr s2 = node2.as<giskard_core::AxisAngleSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleInputSpec>(s2->get_angle()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s2->get_axis()).get());

  angle = boost::dynamic_pointer_cast<giskard_core::DoubleInputSpec>(s2->get_angle());
  axis = boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s2->get_axis());

  EXPECT_EQ(angle->get_input_num(), 3);

  ASSERT_TRUE(axis->get_x().get());
  ASSERT_TRUE(axis->get_y().get());
  ASSERT_TRUE(axis->get_z().get());

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_x()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_x())->get_value(), 1.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_y()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_y())->get_value(), 0.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_z())->get_value(), 0.0);

  // parsing to rotation spec
  ASSERT_NO_THROW(node.as<giskard_core::RotationSpecPtr>());
  giskard_core::RotationSpecPtr s3 = node.as<giskard_core::RotationSpecPtr>();
  
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::AxisAngleSpec>(s3).get());
  giskard_core::AxisAngleSpecPtr s4 = boost::dynamic_pointer_cast<giskard_core::AxisAngleSpec>(s3);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleInputSpec>(s4->get_angle()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s4->get_axis()).get());

  angle = boost::dynamic_pointer_cast<giskard_core::DoubleInputSpec>(s4->get_angle());
  axis = boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s4->get_axis());

  EXPECT_EQ(angle->get_input_num(), 3);

  ASSERT_TRUE(axis->get_x().get());
  ASSERT_TRUE(axis->get_y().get());
  ASSERT_TRUE(axis->get_z().get());

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_x()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_x())->get_value(), 1.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_y()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_y())->get_value(), 0.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_z())->get_value(), 0.0);

  // roundtrip with generation and parsing of rotation specification
  YAML::Node node3;
  node3 = s3;

  ASSERT_NO_THROW(node3.as<giskard_core::RotationSpecPtr>());
  giskard_core::RotationSpecPtr s5 = node3.as<giskard_core::RotationSpecPtr>();
  
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::AxisAngleSpec>(s5).get());
  giskard_core::AxisAngleSpecPtr s6 = boost::dynamic_pointer_cast<giskard_core::AxisAngleSpec>(s5);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleInputSpec>(s6->get_angle()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s6->get_axis()).get());

  angle = boost::dynamic_pointer_cast<giskard_core::DoubleInputSpec>(s6->get_angle());
  axis = boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s6->get_axis());

  EXPECT_EQ(angle->get_input_num(), 3);

  ASSERT_TRUE(axis->get_x().get());
  ASSERT_TRUE(axis->get_y().get());
  ASSERT_TRUE(axis->get_z().get());

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_x()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_x())->get_value(), 1.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_y()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_y())->get_value(), 0.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(axis->get_z())->get_value(), 0.0);
};

TEST_F(YamlParserTest, RotationQuaternionConstructorSpec)
{
  std::string v = "{quaternion: [0.70710678118, 0.0, -0.70710678118, 0.0]}";

  // parsing constructor vector
  YAML::Node node = YAML::Load(v);
  ASSERT_NO_THROW(node.as<giskard_core::RotationQuaternionConstructorSpecPtr>());
  giskard_core::RotationQuaternionConstructorSpecPtr s1 = node.as<giskard_core::RotationQuaternionConstructorSpecPtr>();

  EXPECT_DOUBLE_EQ(0.70710678118, s1->get_x());
  EXPECT_DOUBLE_EQ(0.0, s1->get_y());
  EXPECT_DOUBLE_EQ(-0.70710678118, s1->get_z());
  EXPECT_DOUBLE_EQ(0.0, s1->get_w());

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard_core::RotationQuaternionConstructorSpecPtr>());
  giskard_core::RotationQuaternionConstructorSpecPtr s2 = node2.as<giskard_core::RotationQuaternionConstructorSpecPtr>();

  EXPECT_DOUBLE_EQ(0.70710678118, s2->get_x());
  EXPECT_DOUBLE_EQ(0.0, s2->get_y());
  EXPECT_DOUBLE_EQ(-0.70710678118, s2->get_z());
  EXPECT_DOUBLE_EQ(0.0, s2->get_w());

  // parsing to vector spec
  ASSERT_NO_THROW(node.as<giskard_core::RotationSpecPtr>());
  giskard_core::RotationSpecPtr s3 = node.as<giskard_core::RotationSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::RotationQuaternionConstructorSpec>(s3).get());
  giskard_core::RotationQuaternionConstructorSpecPtr s4 = 
    boost::dynamic_pointer_cast<giskard_core::RotationQuaternionConstructorSpec>(s3);

  EXPECT_DOUBLE_EQ(0.70710678118, s4->get_x());
  EXPECT_DOUBLE_EQ(0.0, s4->get_y());
  EXPECT_DOUBLE_EQ(-0.70710678118, s4->get_z());
  EXPECT_DOUBLE_EQ(0.0, s4->get_w());

  // roundtrip with generation from rotation spec
  YAML::Node node3;
  node3 = s3;
  ASSERT_NO_THROW(node3.as<giskard_core::RotationQuaternionConstructorSpecPtr>());
  giskard_core::RotationQuaternionConstructorSpecPtr s5 = 
    node3.as<giskard_core::RotationQuaternionConstructorSpecPtr>();

  EXPECT_DOUBLE_EQ(0.70710678118, s5->get_x());
  EXPECT_DOUBLE_EQ(0.0, s5->get_y());
  EXPECT_DOUBLE_EQ(-0.70710678118, s5->get_z());
  EXPECT_DOUBLE_EQ(0.0, s5->get_w());
};


TEST_F(YamlParserTest, FrameConstructorSpec)
{
  std::string r = "{axis-angle: [{vector3: [0.0, -2.0, 0.0]}, " + boost::lexical_cast<std::string>(M_PI/-2.0) + "]}";
  std::string t = "{vector3: [1.1, 2.2, 3.3]}";
  std::string f = "{frame: [" + r + ", " + t + "]}";

  // parsing into axis angle specification
  YAML::Node node = YAML::Load(f);

  ASSERT_NO_THROW(node.as<giskard_core::FrameConstructorSpecPtr>());
  giskard_core::FrameConstructorSpecPtr s1 = node.as<giskard_core::FrameConstructorSpecPtr>();

  ASSERT_TRUE(s1->get_rotation().get());
  ASSERT_TRUE(s1->get_translation().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::AxisAngleSpec>(s1->get_rotation()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s1->get_translation()).get());
  giskard_core::AxisAngleSpecPtr rot =
      boost::dynamic_pointer_cast<giskard_core::AxisAngleSpec>(s1->get_rotation());
  giskard_core::VectorConstructorSpecPtr trans =
      boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s1->get_translation());

  ASSERT_TRUE(rot->get_axis().get());
  ASSERT_TRUE(rot->get_angle().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(rot->get_axis()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot->get_angle()).get());
  giskard_core::VectorConstructorSpecPtr rot_axis = 
      boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(rot->get_axis());
  giskard_core::DoubleConstSpecPtr rot_angle =
      boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot->get_angle());

  ASSERT_TRUE(rot_axis->get_x().get());
  ASSERT_TRUE(rot_axis->get_y().get());
  ASSERT_TRUE(rot_axis->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_x())->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_y())->get_value(), -2.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_z())->get_value(), 0.0);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_angle).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_angle)->get_value(), M_PI/-2.0);

  ASSERT_TRUE(trans->get_x().get());
  ASSERT_TRUE(trans->get_y().get());
  ASSERT_TRUE(trans->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_x())->get_value(), 1.1);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_y())->get_value(), 2.2);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_z())->get_value(), 3.3);

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;

  ASSERT_NO_THROW(node2.as<giskard_core::FrameConstructorSpecPtr>());
  giskard_core::FrameConstructorSpecPtr s2 = node2.as<giskard_core::FrameConstructorSpecPtr>();

  ASSERT_TRUE(s2->get_rotation().get());
  ASSERT_TRUE(s2->get_translation().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::AxisAngleSpec>(s2->get_rotation()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s2->get_translation()).get());
  rot = boost::dynamic_pointer_cast<giskard_core::AxisAngleSpec>(s2->get_rotation());
  trans = boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s2->get_translation());

  ASSERT_TRUE(rot->get_axis().get());
  ASSERT_TRUE(rot->get_angle().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(rot->get_axis()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot->get_angle()).get());
  rot_axis = boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(rot->get_axis());
  rot_angle = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot->get_angle());

  ASSERT_TRUE(rot_axis->get_x().get());
  ASSERT_TRUE(rot_axis->get_y().get());
  ASSERT_TRUE(rot_axis->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_x())->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_y())->get_value(), -2.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_z())->get_value(), 0.0);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_angle).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_angle)->get_value(), M_PI/-2.0);

  ASSERT_TRUE(trans->get_x().get());
  ASSERT_TRUE(trans->get_y().get());
  ASSERT_TRUE(trans->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_x())->get_value(), 1.1);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_y())->get_value(), 2.2);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_z())->get_value(), 3.3);

  // parsing into frame specification
  ASSERT_NO_THROW(node.as<giskard_core::FrameSpecPtr>());
  giskard_core::FrameSpecPtr s3 = node.as<giskard_core::FrameSpecPtr>();

  ASSERT_TRUE(s3.get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::FrameConstructorSpec>(s3).get());
  giskard_core::FrameConstructorSpecPtr s4 =
      boost::dynamic_pointer_cast<giskard_core::FrameConstructorSpec>(s3);

  ASSERT_TRUE(s4->get_rotation().get());
  ASSERT_TRUE(s4->get_translation().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::AxisAngleSpec>(s4->get_rotation()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s4->get_translation()).get());
  rot = boost::dynamic_pointer_cast<giskard_core::AxisAngleSpec>(s4->get_rotation());
  trans = boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s4->get_translation());

  ASSERT_TRUE(rot->get_axis().get());
  ASSERT_TRUE(rot->get_angle().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(rot->get_axis()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot->get_angle()).get());
  rot_axis = boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(rot->get_axis());
  rot_angle = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot->get_angle());

  ASSERT_TRUE(rot_axis->get_x().get());
  ASSERT_TRUE(rot_axis->get_y().get());
  ASSERT_TRUE(rot_axis->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_x())->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_y())->get_value(), -2.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_z())->get_value(), 0.0);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_angle).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_angle)->get_value(), M_PI/-2.0);

  ASSERT_TRUE(trans->get_x().get());
  ASSERT_TRUE(trans->get_y().get());
  ASSERT_TRUE(trans->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_x())->get_value(), 1.1);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_y())->get_value(), 2.2);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_z())->get_value(), 3.3);

  // roundtrip with generation and parsing into frame specification
  YAML::Node node3;
  node3 = s3;

  ASSERT_NO_THROW(node3.as<giskard_core::FrameSpecPtr>());
  giskard_core::FrameSpecPtr s5 = node3.as<giskard_core::FrameSpecPtr>();

  ASSERT_TRUE(s5.get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::FrameConstructorSpec>(s5).get());
  giskard_core::FrameConstructorSpecPtr s6 =
      boost::dynamic_pointer_cast<giskard_core::FrameConstructorSpec>(s5);

  ASSERT_TRUE(s6->get_rotation().get());
  ASSERT_TRUE(s6->get_translation().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::AxisAngleSpec>(s6->get_rotation()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s6->get_translation()).get());
  rot = boost::dynamic_pointer_cast<giskard_core::AxisAngleSpec>(s6->get_rotation());
  trans = boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(s6->get_translation());

  ASSERT_TRUE(rot->get_axis().get());
  ASSERT_TRUE(rot->get_angle().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(rot->get_axis()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot->get_angle()).get());
  rot_axis = boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(rot->get_axis());
  rot_angle = boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot->get_angle());

  ASSERT_TRUE(rot_axis->get_x().get());
  ASSERT_TRUE(rot_axis->get_y().get());
  ASSERT_TRUE(rot_axis->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_x())->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_y())->get_value(), -2.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_axis->get_z())->get_value(), 0.0);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_angle).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rot_angle)->get_value(), M_PI/-2.0);

  ASSERT_TRUE(trans->get_x().get());
  ASSERT_TRUE(trans->get_y().get());
  ASSERT_TRUE(trans->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_x())->get_value(), 1.1);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_y())->get_value(), 2.2);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(trans->get_z())->get_value(), 3.3);
};

TEST_F(YamlParserTest, FrameMultiplicationSpec)
{
  std::string r1 = "{axis-angle: [{vector3: [1.0, 0.0, 0.0]}, 1.0]}";
  std::string r2 = "{axis-angle: [{vector3: [0.0, 1.0, 0.0]}, -0.5]}";
  std::string t1 = "{vector3: [0.1, 0.2, 0.3]}";
  std::string t2 = "{vector3: [-1.1, -2.2, -3.3]}";
  std::string f1 = "{frame: [" + r1 + ", " + t1 + "]}";
  std::string f2 = "{frame: [" + r2 + ", " + t2 + "]}";
  std::string f3 = "{frame-mul: [" + f1 + ", " + f2 + "]}";
  YAML::Node node = YAML::Load(f1);
  ASSERT_NO_THROW(node.as<giskard_core::FrameConstructorSpecPtr>());
  giskard_core::FrameConstructorSpecPtr s1 = node.as<giskard_core::FrameConstructorSpecPtr>();
  node = YAML::Load(f2);
  ASSERT_NO_THROW(node.as<giskard_core::FrameConstructorSpecPtr>());
  giskard_core::FrameConstructorSpecPtr s2 = node.as<giskard_core::FrameConstructorSpecPtr>();
 
  // parsing into frame-mul specification
  node = YAML::Load(f3);
  ASSERT_NO_THROW(node.as<giskard_core::FrameMultiplicationSpecPtr>());
  giskard_core::FrameMultiplicationSpecPtr s3 = node.as<giskard_core::FrameMultiplicationSpecPtr>();
 
  ASSERT_EQ(s3->get_inputs().size(), 2);
  EXPECT_TRUE(s3->get_inputs()[0]->equals(*s1));
  EXPECT_TRUE(s3->get_inputs()[1]->equals(*s2));

  // roundtrip with generation
  YAML::Node node2;
  node2 = s3;
  ASSERT_NO_THROW(node2.as<giskard_core::FrameMultiplicationSpecPtr>());
  giskard_core::FrameMultiplicationSpecPtr s4 = node2.as<giskard_core::FrameMultiplicationSpecPtr>();
 
  ASSERT_EQ(s4->get_inputs().size(), 2);
  EXPECT_TRUE(s4->get_inputs()[0]->equals(*s1));
  EXPECT_TRUE(s4->get_inputs()[1]->equals(*s2));

  // parsing into frame specification
  ASSERT_NO_THROW(node.as<giskard_core::FrameSpecPtr>());
  giskard_core::FrameSpecPtr s5 = node.as<giskard_core::FrameSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::FrameMultiplicationSpec>(s5).get());
  giskard_core::FrameMultiplicationSpecPtr s6 = boost::dynamic_pointer_cast<giskard_core::FrameMultiplicationSpec>(s5);
 
  ASSERT_EQ(s6->get_inputs().size(), 2);
  EXPECT_TRUE(s6->get_inputs()[0]->equals(*s1));
  EXPECT_TRUE(s6->get_inputs()[1]->equals(*s2));

  //roundtrip with generation from frame specification
  YAML::Node node3;
  node3 = s5;
  
  ASSERT_NO_THROW(node3.as<giskard_core::FrameMultiplicationSpecPtr>());
  giskard_core::FrameMultiplicationSpecPtr s7 = node2.as<giskard_core::FrameMultiplicationSpecPtr>();
 
  ASSERT_EQ(s7->get_inputs().size(), 2);
  EXPECT_TRUE(s7->get_inputs()[0]->equals(*s1));
  EXPECT_TRUE(s7->get_inputs()[1]->equals(*s2));
}

TEST_F(YamlParserTest, OrientationOfSpec)
{
  std::string r = "{axis-angle: [{vector3: [0.0, 1.0, 0.0]}, 1.5]}";
  std::string t = "{vector3: [1.1, 2.2, 3.3]}";
  std::string f = "{frame: [" + r + ", " + t + "]}";
  std::string r1 = "{orientation-of: " + f + "}";

  ASSERT_NO_THROW(YAML::Load(f).as<giskard_core::FrameSpecPtr>());
  giskard_core::FrameSpecPtr fs = YAML::Load(f).as<giskard_core::FrameSpecPtr>();
  ASSERT_TRUE(fs.get() != NULL);

  // parsing into OrientationOfSpec
  YAML::Node node = YAML::Load(r1);
  ASSERT_NO_THROW(node.as<giskard_core::OrientationOfSpecPtr>());
  giskard_core::OrientationOfSpecPtr s1 = node.as<giskard_core::OrientationOfSpecPtr>();

  ASSERT_TRUE(s1->get_frame().get() != NULL);
  EXPECT_TRUE(fs->equals(*(s1->get_frame())));

  // roundtrip with YAML generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard_core::OrientationOfSpecPtr>());
  giskard_core::OrientationOfSpecPtr s2 = node2.as<giskard_core::OrientationOfSpecPtr>();

  ASSERT_TRUE(s2->get_frame().get() != NULL);
  EXPECT_TRUE(fs->equals(*(s2->get_frame())));

  // parsing into RotationSpec
  ASSERT_NO_THROW(node.as<giskard_core::RotationSpecPtr>());
  giskard_core::RotationSpecPtr s3 = node.as<giskard_core::RotationSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::OrientationOfSpec>(s3).get());
  giskard_core::OrientationOfSpecPtr s4 = boost::dynamic_pointer_cast<giskard_core::OrientationOfSpec>(s3);

  ASSERT_TRUE(s4->get_frame().get() != NULL);
  EXPECT_TRUE(fs->equals(*(s4->get_frame())));

  // roundtrip with generation from RotationSpec
  YAML::Node node3;
  node3 = s3;

  ASSERT_NO_THROW(node3.as<giskard_core::OrientationOfSpecPtr>());
  giskard_core::OrientationOfSpecPtr s5 = node3.as<giskard_core::OrientationOfSpecPtr>();

  ASSERT_TRUE(s5->get_frame().get() != NULL);
  EXPECT_TRUE(fs->equals(*(s5->get_frame())));
}

TEST_F(YamlParserTest, InverseRotationSpec)
{
  std::string r = "{axis-angle: [{vector3: [0.0, 1.0, 0.0]}, 1.5]}";
  std::string ir = "{inverse-rotation: " + r + "}";

  ASSERT_NO_THROW(YAML::Load(r).as<giskard_core::RotationSpecPtr>());
  giskard_core::RotationSpecPtr rs = YAML::Load(r).as<giskard_core::RotationSpecPtr>();
  ASSERT_TRUE(rs.get() != NULL);

  // parsing into OrientationOfSpec
  YAML::Node node = YAML::Load(ir);
  ASSERT_NO_THROW(node.as<giskard_core::InverseRotationSpecPtr>());
  giskard_core::InverseRotationSpecPtr s1 = node.as<giskard_core::InverseRotationSpecPtr>();

  ASSERT_TRUE(s1->get_rotation().get() != NULL);
  EXPECT_TRUE(rs->equals(*(s1->get_rotation())));

  // roundtrip with YAML generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard_core::InverseRotationSpecPtr>());
  giskard_core::InverseRotationSpecPtr s2 = node2.as<giskard_core::InverseRotationSpecPtr>();

  ASSERT_TRUE(s2->get_rotation().get() != NULL);
  EXPECT_TRUE(rs->equals(*(s2->get_rotation())));

  // parsing into RotationSpec
  ASSERT_NO_THROW(node.as<giskard_core::RotationSpecPtr>());
  giskard_core::RotationSpecPtr s3 = node.as<giskard_core::RotationSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::InverseRotationSpec>(s3).get());
  giskard_core::InverseRotationSpecPtr s4 = boost::dynamic_pointer_cast<giskard_core::InverseRotationSpec>(s3);

  ASSERT_TRUE(s4->get_rotation().get() != NULL);
  EXPECT_TRUE(rs->equals(*(s4->get_rotation())));

  // roundtrip with generation from RotationSpec
  YAML::Node node3;
  node3 = s3;

  ASSERT_NO_THROW(node3.as<giskard_core::InverseRotationSpecPtr>());
  giskard_core::InverseRotationSpecPtr s5 = node3.as<giskard_core::InverseRotationSpecPtr>();

  ASSERT_TRUE(s5->get_rotation().get() != NULL);
  EXPECT_TRUE(rs->equals(*(s5->get_rotation())));
}

void equality_check_rot_specs(const giskard_core::RotationSpecPtr& r1, const giskard_core::RotationSpecPtr& r2)
{
  ASSERT_TRUE(r1.get() != NULL);
  ASSERT_TRUE(r2.get() != NULL);
  EXPECT_TRUE(r1->equals(*r2));
}

void equality_check_rot_mul(const giskard_core::RotationMultiplicationSpecPtr& mul,
    const std::vector<giskard_core::RotationSpecPtr>& rots)
{
  ASSERT_TRUE(mul.get() != NULL);
  ASSERT_EQ(mul->get_inputs().size(), rots.size());
  for(size_t i=0; i<rots.size(); ++i)
    equality_check_rot_specs(mul->get_inputs()[i], rots[i]);
}

TEST_F(YamlParserTest, RotationMultiplicationSpec)
{
  std::string r1 = "{axis-angle: [{vector3: [0.0, 1.0, 0.0]}, 1.5]}";
  std::string r2 = "{quaternion: [0.0, 0.0, 1.0, 0.0]}";
  std::string r = "{rotation-mul: [" + r1 + ", " + r2 + "]}";

  std::vector< giskard_core::RotationSpecPtr > rots;
  ASSERT_NO_THROW(YAML::Load(r1).as<giskard_core::RotationSpecPtr>());
  rots.push_back(YAML::Load(r1).as<giskard_core::RotationSpecPtr>());
  ASSERT_NO_THROW(YAML::Load(r2).as<giskard_core::RotationSpecPtr>());
  rots.push_back(YAML::Load(r2).as<giskard_core::RotationSpecPtr>());

  // parsing into RotationMultiplicationSpec
  YAML::Node node = YAML::Load(r);
  ASSERT_NO_THROW(node.as<giskard_core::RotationMultiplicationSpecPtr>());
  giskard_core::RotationMultiplicationSpecPtr s1 = node.as<giskard_core::RotationMultiplicationSpecPtr>();

  equality_check_rot_mul(s1, rots);

  // roundtrip with YAML generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard_core::RotationMultiplicationSpecPtr>());
  giskard_core::RotationMultiplicationSpecPtr s2 = node2.as<giskard_core::RotationMultiplicationSpecPtr>();

  equality_check_rot_mul(s2, rots);

  // parsing into RotationSpec
  ASSERT_NO_THROW(node.as<giskard_core::RotationSpecPtr>());
  giskard_core::RotationSpecPtr s3 = node.as<giskard_core::RotationSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard_core::RotationMultiplicationSpec>(s3).get());
  giskard_core::RotationMultiplicationSpecPtr s4 = boost::dynamic_pointer_cast<giskard_core::RotationMultiplicationSpec>(s3);

  equality_check_rot_mul(s4, rots);

  // roundtrip with generation from RotationSpec
  YAML::Node node3;
  node3 = s3;

  ASSERT_NO_THROW(node3.as<giskard_core::RotationMultiplicationSpecPtr>());
  giskard_core::RotationMultiplicationSpecPtr s5 = node3.as<giskard_core::RotationMultiplicationSpecPtr>();

  equality_check_rot_mul(s5, rots);
}

TEST_F(YamlParserTest, GithubIssueNo1)
{
  std::string s = "{double-mul: [-1, {vector3: [1, 2, 3]}]}";

  YAML::Node node = YAML::Load(s);
  EXPECT_ANY_THROW(node.as<giskard_core::DoubleSpecPtr>());
}

TEST_F(YamlParserTest, ControllableConstraintSpec)
{
  std::string s = "controllable-constraint: [-0.1, 0.2, 5.0, 2, my name]";

  YAML::Node node = YAML::Load(s);

  ASSERT_NO_THROW(node.as<giskard_core::ControllableConstraintSpec>());
  giskard_core::ControllableConstraintSpec spec = node.as<giskard_core::ControllableConstraintSpec>();

  EXPECT_DOUBLE_EQ(spec.lower_->get_expression(giskard_core::Scope())->value(), -0.1);
  EXPECT_DOUBLE_EQ(spec.upper_->get_expression(giskard_core::Scope())->value(), 0.2);
  EXPECT_DOUBLE_EQ(spec.weight_->get_expression(giskard_core::Scope())->value(), 5.0);
  EXPECT_EQ(spec.input_number_, 2);
  EXPECT_STREQ(spec.name_.c_str(), "my name");
}

TEST_F(YamlParserTest, SoftConstraintSpec)
{
  std::string s = "{soft-constraint: [-10.1, 120.2, 5.0, 1.1, some name]}";

  YAML::Node node = YAML::Load(s);

  ASSERT_NO_THROW(node.as<giskard_core::SoftConstraintSpec>());
  giskard_core::SoftConstraintSpec spec = node.as<giskard_core::SoftConstraintSpec>();

  EXPECT_DOUBLE_EQ(spec.lower_->get_expression(giskard_core::Scope())->value(), -10.1);
  EXPECT_DOUBLE_EQ(spec.upper_->get_expression(giskard_core::Scope())->value(), 120.2);
  EXPECT_DOUBLE_EQ(spec.weight_->get_expression(giskard_core::Scope())->value(), 5.0);
  EXPECT_DOUBLE_EQ(spec.expression_->get_expression(giskard_core::Scope())->value(), 1.1);
  EXPECT_STREQ(spec.name_.c_str(), "some name");
}

TEST_F(YamlParserTest, HardConstraintSpec)
{
  std::string s = "{hard-constraint: [-10.1, 120.2, 1.1]}";

  YAML::Node node = YAML::Load(s);

  ASSERT_NO_THROW(node.as<giskard_core::HardConstraintSpec>());
  giskard_core::HardConstraintSpec spec = node.as<giskard_core::HardConstraintSpec>();

  EXPECT_DOUBLE_EQ(spec.lower_->get_expression(giskard_core::Scope())->value(), -10.1);
  EXPECT_DOUBLE_EQ(spec.upper_->get_expression(giskard_core::Scope())->value(), 120.2);
  EXPECT_DOUBLE_EQ(spec.expression_->get_expression(giskard_core::Scope())->value(), 1.1);
}

TEST_F(YamlParserTest, QPControllerSpec)
{
  std::string sc = "scope: []";
  std::string co = "controllable-constraints: [{controllable-constraint: [-0.1, 0.2, 5.0, 2, controllable1]}]";
  std::string so = "soft-constraints: [{soft-constraint: [-10.1, 120.2, 5.0, 1.1, goal1]}]";
  std::string ha = "hard-constraints: [{hard-constraint: [-33.1, 110.3, 17.1]}]";

  std::string s = sc + "\n" + co + "\n" + so + "\n" + ha;

  YAML::Node node = YAML::Load(s);

  ASSERT_NO_THROW(node.as<giskard_core::QPControllerSpec>());
  giskard_core::QPControllerSpec spec = node.as<giskard_core::QPControllerSpec>();

  ASSERT_EQ(spec.scope_.size(), 0);
  ASSERT_EQ(spec.controllable_constraints_.size(), 1);
  ASSERT_EQ(spec.soft_constraints_.size(), 1);
  ASSERT_EQ(spec.hard_constraints_.size(), 1);

  EXPECT_DOUBLE_EQ(spec.controllable_constraints_[0].lower_->get_expression(giskard_core::Scope())->value(), -0.1);
  EXPECT_DOUBLE_EQ(spec.controllable_constraints_[0].upper_->get_expression(giskard_core::Scope())->value(), 0.2);
  EXPECT_DOUBLE_EQ(spec.controllable_constraints_[0].weight_->get_expression(giskard_core::Scope())->value(), 5.0);
  EXPECT_EQ(spec.controllable_constraints_[0].input_number_, 2);
  EXPECT_STREQ(spec.controllable_constraints_[0].name_.c_str(), "controllable1");

  EXPECT_DOUBLE_EQ(spec.soft_constraints_[0].lower_->get_expression(giskard_core::Scope())->value(), -10.1);
  EXPECT_DOUBLE_EQ(spec.soft_constraints_[0].upper_->get_expression(giskard_core::Scope())->value(), 120.2);
  EXPECT_DOUBLE_EQ(spec.soft_constraints_[0].weight_->get_expression(giskard_core::Scope())->value(), 5.0);
  EXPECT_DOUBLE_EQ(spec.soft_constraints_[0].expression_->get_expression(giskard_core::Scope())->value(), 1.1);
  EXPECT_STREQ(spec.soft_constraints_[0].name_.c_str(), "goal1");

  EXPECT_DOUBLE_EQ(spec.hard_constraints_[0].lower_->get_expression(giskard_core::Scope())->value(), -33.1);
  EXPECT_DOUBLE_EQ(spec.hard_constraints_[0].upper_->get_expression(giskard_core::Scope())->value(), 110.3);
  EXPECT_DOUBLE_EQ(spec.hard_constraints_[0].expression_->get_expression(giskard_core::Scope())->value(), 17.1);
}
