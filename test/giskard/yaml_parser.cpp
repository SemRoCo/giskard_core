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
  ASSERT_NO_THROW(node.as<giskard::DoubleConstSpecPtr>());
  giskard::DoubleConstSpecPtr s1 = node.as<giskard::DoubleConstSpecPtr>();

  EXPECT_DOUBLE_EQ(1.1, s1->get_value());

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard::DoubleConstSpecPtr>());
  giskard::DoubleConstSpecPtr s2 = node2.as<giskard::DoubleConstSpecPtr>();

  EXPECT_DOUBLE_EQ(s1->get_value(), s2->get_value());

  // parsing to double spec
  ASSERT_NO_THROW(node.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr s3 = node.as<giskard::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s3).get());
  giskard::DoubleConstSpecPtr s4 = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s3);

  EXPECT_DOUBLE_EQ(1.1, s4->get_value());

  // roundtrip with generation to double spec
  YAML::Node node3;
  node3 = s3;
  ASSERT_NO_THROW(node3.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr s5 = node3.as<giskard::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s5).get());
  giskard::DoubleConstSpecPtr s6 = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s5);

  EXPECT_DOUBLE_EQ(1.1, s6->get_value());
};

TEST_F(YamlParserTest, InputExpression)
{
  std::string i = "{input-var: 2}";

  // parsing input double
  YAML::Node node = YAML::Load(i);
  ASSERT_NO_THROW(node.as<giskard::DoubleInputSpecPtr>());
  giskard::DoubleInputSpecPtr s1 = node.as<giskard::DoubleInputSpecPtr>();

  EXPECT_EQ(2, s1->get_input_num());

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard::DoubleInputSpecPtr>());
  giskard::DoubleInputSpecPtr s2 = node2.as<giskard::DoubleInputSpecPtr>();

  EXPECT_EQ(s1->get_input_num(), s2->get_input_num());

  // parsing to double spec
  ASSERT_NO_THROW(node.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr s3 = node.as<giskard::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleInputSpec>(s3).get());
  giskard::DoubleInputSpecPtr s4 = boost::dynamic_pointer_cast<giskard::DoubleInputSpec>(s3);

  EXPECT_EQ(2, s4->get_input_num());

  // roundtrip with generation to double spec
  YAML::Node node3;
  node3 = s3;
  ASSERT_NO_THROW(node3.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr s5 = node3.as<giskard::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleInputSpec>(s5).get());
  giskard::DoubleInputSpecPtr s6 = boost::dynamic_pointer_cast<giskard::DoubleInputSpec>(s5);

  EXPECT_EQ(2, s6->get_input_num());
};

TEST_F(YamlParserTest, VectorConstructorSpec)
{
  std::string v = "{vector3: [1.1, 2.2, 3.3]}";

  // parsing constructor vector
  YAML::Node node = YAML::Load(v);
  ASSERT_NO_THROW(node.as<giskard::VectorConstructorSpecPtr>());
  giskard::VectorConstructorSpecPtr s1 = node.as<giskard::VectorConstructorSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s1->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s1->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s1->get_z()).get());

  giskard::DoubleConstSpecPtr x = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s1->get_x());
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  giskard::DoubleConstSpecPtr y = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s1->get_y());
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  giskard::DoubleConstSpecPtr z = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s1->get_z());
  EXPECT_DOUBLE_EQ(z->get_value(), 3.3);

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard::VectorConstructorSpecPtr>());
  giskard::VectorConstructorSpecPtr s2 = node2.as<giskard::VectorConstructorSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s2->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s2->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s2->get_z()).get());

  x = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s2->get_x());
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  y = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s2->get_y());
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  z = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s2->get_z());
  EXPECT_DOUBLE_EQ(z->get_value(), 3.3);

  // parsing to vector spec
  ASSERT_NO_THROW(node.as<giskard::VectorSpecPtr>());
  giskard::VectorSpecPtr s3 = node.as<giskard::VectorSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s3).get());
  giskard::VectorConstructorSpecPtr s4 = boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s3);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s4->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s4->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s4->get_z()).get());

  x = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s4->get_x());
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  y = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s4->get_y());
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  z = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s4->get_z());
  EXPECT_DOUBLE_EQ(z->get_value(), 3.3);

  // roundtrip with generation to double spec
  YAML::Node node3;
  node3 = s3;
  ASSERT_NO_THROW(node3.as<giskard::VectorSpecPtr>());
  giskard::VectorSpecPtr s5 = node3.as<giskard::VectorSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s5).get());
  giskard::VectorConstructorSpecPtr s6 = boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s5);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s6->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s6->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s6->get_z()).get());

  x = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s6->get_x());
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  y = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s6->get_y());
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  z = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(s6->get_z());
  EXPECT_DOUBLE_EQ(z->get_value(), 3.3);
};

TEST_F(YamlParserTest, AxisAngleSpec)
{
  std::string r = "{axis-angle: [{vector3: [1.0, 0.0, 0.0]}, {input-var: 3}]}"; 

  // parsing into axis angle specification
  YAML::Node node = YAML::Load(r);

  ASSERT_NO_THROW(node.as<giskard::AxisAngleSpecPtr>());
  giskard::AxisAngleSpecPtr s1 = node.as<giskard::AxisAngleSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleInputSpec>(s1->get_angle()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s1->get_axis()).get());

  giskard::DoubleInputSpecPtr angle = 
      boost::dynamic_pointer_cast<giskard::DoubleInputSpec>(s1->get_angle());
  giskard::VectorConstructorSpecPtr axis = 
      boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s1->get_axis());

  EXPECT_EQ(angle->get_input_num(), 3);

  ASSERT_TRUE(axis->get_x().get());
  ASSERT_TRUE(axis->get_y().get());
  ASSERT_TRUE(axis->get_z().get());

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_x()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_x())->get_value(), 1.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_y()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_y())->get_value(), 0.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_z())->get_value(), 0.0);

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;

  ASSERT_NO_THROW(node2.as<giskard::AxisAngleSpecPtr>());
  giskard::AxisAngleSpecPtr s2 = node2.as<giskard::AxisAngleSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleInputSpec>(s2->get_angle()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s2->get_axis()).get());

  angle = boost::dynamic_pointer_cast<giskard::DoubleInputSpec>(s2->get_angle());
  axis = boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s2->get_axis());

  EXPECT_EQ(angle->get_input_num(), 3);

  ASSERT_TRUE(axis->get_x().get());
  ASSERT_TRUE(axis->get_y().get());
  ASSERT_TRUE(axis->get_z().get());

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_x()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_x())->get_value(), 1.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_y()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_y())->get_value(), 0.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_z())->get_value(), 0.0);

  // parsing to rotation spec
  ASSERT_NO_THROW(node.as<giskard::RotationSpecPtr>());
  giskard::RotationSpecPtr s3 = node.as<giskard::RotationSpecPtr>();
  
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s3).get());
  giskard::AxisAngleSpecPtr s4 = boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s3);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleInputSpec>(s4->get_angle()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s4->get_axis()).get());

  angle = boost::dynamic_pointer_cast<giskard::DoubleInputSpec>(s4->get_angle());
  axis = boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s4->get_axis());

  EXPECT_EQ(angle->get_input_num(), 3);

  ASSERT_TRUE(axis->get_x().get());
  ASSERT_TRUE(axis->get_y().get());
  ASSERT_TRUE(axis->get_z().get());

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_x()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_x())->get_value(), 1.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_y()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_y())->get_value(), 0.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_z())->get_value(), 0.0);

  // roundtrip with generation and parsing of rotation specification
  YAML::Node node3;
  node3 = s3;

  ASSERT_NO_THROW(node3.as<giskard::RotationSpecPtr>());
  giskard::RotationSpecPtr s5 = node3.as<giskard::RotationSpecPtr>();
  
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s5).get());
  giskard::AxisAngleSpecPtr s6 = boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s5);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleInputSpec>(s6->get_angle()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s6->get_axis()).get());

  angle = boost::dynamic_pointer_cast<giskard::DoubleInputSpec>(s6->get_angle());
  axis = boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s6->get_axis());

  EXPECT_EQ(angle->get_input_num(), 3);

  ASSERT_TRUE(axis->get_x().get());
  ASSERT_TRUE(axis->get_y().get());
  ASSERT_TRUE(axis->get_z().get());

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_x()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_x())->get_value(), 1.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_y()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_y())->get_value(), 0.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(axis->get_z())->get_value(), 0.0);
};

TEST_F(YamlParserTest, RotationQuaternionConstructorSpec)
{
  std::string v = "{quaternion: [0.70710678118, 0.0, -0.70710678118, 0.0]}";

  // parsing constructor vector
  YAML::Node node = YAML::Load(v);
  ASSERT_NO_THROW(node.as<giskard::RotationQuaternionConstructorSpecPtr>());
  giskard::RotationQuaternionConstructorSpecPtr s1 = node.as<giskard::RotationQuaternionConstructorSpecPtr>();

  //std::cout << "x=" << x << ", y=" << y << ", z=" << z << ", w=" << w << std::endl;

  EXPECT_DOUBLE_EQ(0.70710678118, s1->get_x());
  EXPECT_DOUBLE_EQ(0.0, s1->get_y());
  EXPECT_DOUBLE_EQ(-0.70710678118, s1->get_z());
  EXPECT_DOUBLE_EQ(0.0, s1->get_w());

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard::RotationQuaternionConstructorSpecPtr>());
  giskard::RotationQuaternionConstructorSpecPtr s2 = node2.as<giskard::RotationQuaternionConstructorSpecPtr>();

  EXPECT_DOUBLE_EQ(0.70710678118, s2->get_x());
  EXPECT_DOUBLE_EQ(0.0, s2->get_y());
  EXPECT_DOUBLE_EQ(-0.70710678118, s2->get_z());
  EXPECT_DOUBLE_EQ(0.0, s2->get_w());

  // parsing to vector spec
  ASSERT_NO_THROW(node.as<giskard::RotationSpecPtr>());
  giskard::RotationSpecPtr s3 = node.as<giskard::RotationSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::RotationQuaternionConstructorSpec>(s3).get());
  giskard::RotationQuaternionConstructorSpecPtr s4 = 
    boost::dynamic_pointer_cast<giskard::RotationQuaternionConstructorSpec>(s3);

  EXPECT_DOUBLE_EQ(0.70710678118, s4->get_x());
  EXPECT_DOUBLE_EQ(0.0, s4->get_y());
  EXPECT_DOUBLE_EQ(-0.70710678118, s4->get_z());
  EXPECT_DOUBLE_EQ(0.0, s4->get_w());

  // roundtrip with generation from rotation spec
  YAML::Node node3;
  node3 = s3;
  ASSERT_NO_THROW(node3.as<giskard::RotationQuaternionConstructorSpecPtr>());
  giskard::RotationQuaternionConstructorSpecPtr s5 = 
    node3.as<giskard::RotationQuaternionConstructorSpecPtr>();

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

  ASSERT_NO_THROW(node.as<giskard::FrameConstructorSpecPtr>());
  giskard::FrameConstructorSpecPtr s1 = node.as<giskard::FrameConstructorSpecPtr>();

  ASSERT_TRUE(s1->get_rotation().get());
  ASSERT_TRUE(s1->get_translation().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s1->get_rotation()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s1->get_translation()).get());
  giskard::AxisAngleSpecPtr rot =
      boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s1->get_rotation());
  giskard::VectorConstructorSpecPtr trans =
      boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s1->get_translation());

  ASSERT_TRUE(rot->get_axis().get());
  ASSERT_TRUE(rot->get_angle().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(rot->get_axis()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot->get_angle()).get());
  giskard::VectorConstructorSpecPtr rot_axis = 
      boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(rot->get_axis());
  giskard::DoubleConstSpecPtr rot_angle =
      boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot->get_angle());

  ASSERT_TRUE(rot_axis->get_x().get());
  ASSERT_TRUE(rot_axis->get_y().get());
  ASSERT_TRUE(rot_axis->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_x())->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_y())->get_value(), -2.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_z())->get_value(), 0.0);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_angle).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_angle)->get_value(), M_PI/-2.0);

  ASSERT_TRUE(trans->get_x().get());
  ASSERT_TRUE(trans->get_y().get());
  ASSERT_TRUE(trans->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_x())->get_value(), 1.1);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_y())->get_value(), 2.2);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_z())->get_value(), 3.3);

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;

  ASSERT_NO_THROW(node2.as<giskard::FrameConstructorSpecPtr>());
  giskard::FrameConstructorSpecPtr s2 = node2.as<giskard::FrameConstructorSpecPtr>();

  ASSERT_TRUE(s2->get_rotation().get());
  ASSERT_TRUE(s2->get_translation().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s2->get_rotation()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s2->get_translation()).get());
  rot = boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s2->get_rotation());
  trans = boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s2->get_translation());

  ASSERT_TRUE(rot->get_axis().get());
  ASSERT_TRUE(rot->get_angle().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(rot->get_axis()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot->get_angle()).get());
  rot_axis = boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(rot->get_axis());
  rot_angle = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot->get_angle());

  ASSERT_TRUE(rot_axis->get_x().get());
  ASSERT_TRUE(rot_axis->get_y().get());
  ASSERT_TRUE(rot_axis->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_x())->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_y())->get_value(), -2.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_z())->get_value(), 0.0);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_angle).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_angle)->get_value(), M_PI/-2.0);

  ASSERT_TRUE(trans->get_x().get());
  ASSERT_TRUE(trans->get_y().get());
  ASSERT_TRUE(trans->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_x())->get_value(), 1.1);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_y())->get_value(), 2.2);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_z())->get_value(), 3.3);

  // parsing into frame specification
  ASSERT_NO_THROW(node.as<giskard::FrameSpecPtr>());
  giskard::FrameSpecPtr s3 = node.as<giskard::FrameSpecPtr>();

  ASSERT_TRUE(s3.get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::FrameConstructorSpec>(s3).get());
  giskard::FrameConstructorSpecPtr s4 =
      boost::dynamic_pointer_cast<giskard::FrameConstructorSpec>(s3);

  ASSERT_TRUE(s4->get_rotation().get());
  ASSERT_TRUE(s4->get_translation().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s4->get_rotation()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s4->get_translation()).get());
  rot = boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s4->get_rotation());
  trans = boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s4->get_translation());

  ASSERT_TRUE(rot->get_axis().get());
  ASSERT_TRUE(rot->get_angle().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(rot->get_axis()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot->get_angle()).get());
  rot_axis = boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(rot->get_axis());
  rot_angle = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot->get_angle());

  ASSERT_TRUE(rot_axis->get_x().get());
  ASSERT_TRUE(rot_axis->get_y().get());
  ASSERT_TRUE(rot_axis->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_x())->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_y())->get_value(), -2.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_z())->get_value(), 0.0);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_angle).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_angle)->get_value(), M_PI/-2.0);

  ASSERT_TRUE(trans->get_x().get());
  ASSERT_TRUE(trans->get_y().get());
  ASSERT_TRUE(trans->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_x())->get_value(), 1.1);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_y())->get_value(), 2.2);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_z())->get_value(), 3.3);

  // roundtrip with generation and parsing into frame specification
  YAML::Node node3;
  node3 = s3;

  ASSERT_NO_THROW(node3.as<giskard::FrameSpecPtr>());
  giskard::FrameSpecPtr s5 = node3.as<giskard::FrameSpecPtr>();

  ASSERT_TRUE(s5.get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::FrameConstructorSpec>(s5).get());
  giskard::FrameConstructorSpecPtr s6 =
      boost::dynamic_pointer_cast<giskard::FrameConstructorSpec>(s5);

  ASSERT_TRUE(s6->get_rotation().get());
  ASSERT_TRUE(s6->get_translation().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s6->get_rotation()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s6->get_translation()).get());
  rot = boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s6->get_rotation());
  trans = boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(s6->get_translation());

  ASSERT_TRUE(rot->get_axis().get());
  ASSERT_TRUE(rot->get_angle().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(rot->get_axis()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot->get_angle()).get());
  rot_axis = boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(rot->get_axis());
  rot_angle = boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot->get_angle());

  ASSERT_TRUE(rot_axis->get_x().get());
  ASSERT_TRUE(rot_axis->get_y().get());
  ASSERT_TRUE(rot_axis->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_x())->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_y())->get_value(), -2.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_axis->get_z())->get_value(), 0.0);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_angle).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rot_angle)->get_value(), M_PI/-2.0);

  ASSERT_TRUE(trans->get_x().get());
  ASSERT_TRUE(trans->get_y().get());
  ASSERT_TRUE(trans->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_x())->get_value(), 1.1);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_y())->get_value(), 2.2);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(trans->get_z())->get_value(), 3.3);
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
  ASSERT_NO_THROW(node.as<giskard::FrameConstructorSpecPtr>());
  giskard::FrameConstructorSpecPtr s1 = node.as<giskard::FrameConstructorSpecPtr>();
  node = YAML::Load(f2);
  ASSERT_NO_THROW(node.as<giskard::FrameConstructorSpecPtr>());
  giskard::FrameConstructorSpecPtr s2 = node.as<giskard::FrameConstructorSpecPtr>();
 
  // parsing into frame-mul specification
  node = YAML::Load(f3);
  ASSERT_NO_THROW(node.as<giskard::FrameMultiplicationSpecPtr>());
  giskard::FrameMultiplicationSpecPtr s3 = node.as<giskard::FrameMultiplicationSpecPtr>();
 
  ASSERT_EQ(s3->get_inputs().size(), 2);
  EXPECT_TRUE(s3->get_inputs()[0]->equals(*s1));
  EXPECT_TRUE(s3->get_inputs()[1]->equals(*s2));

  // roundtrip with generation
  YAML::Node node2;
  node2 = s3;
  ASSERT_NO_THROW(node2.as<giskard::FrameMultiplicationSpecPtr>());
  giskard::FrameMultiplicationSpecPtr s4 = node2.as<giskard::FrameMultiplicationSpecPtr>();
 
  ASSERT_EQ(s4->get_inputs().size(), 2);
  EXPECT_TRUE(s4->get_inputs()[0]->equals(*s1));
  EXPECT_TRUE(s4->get_inputs()[1]->equals(*s2));

  // parsing into frame specification
  ASSERT_NO_THROW(node.as<giskard::FrameSpecPtr>());
  giskard::FrameSpecPtr s5 = node.as<giskard::FrameSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::FrameMultiplicationSpec>(s5).get());
  giskard::FrameMultiplicationSpecPtr s6 = boost::dynamic_pointer_cast<giskard::FrameMultiplicationSpec>(s5);
 
  ASSERT_EQ(s6->get_inputs().size(), 2);
  EXPECT_TRUE(s6->get_inputs()[0]->equals(*s1));
  EXPECT_TRUE(s6->get_inputs()[1]->equals(*s2));

  //roundtrip with generation from frame specification
  YAML::Node node3;
  node3 = s5;
  
  ASSERT_NO_THROW(node3.as<giskard::FrameMultiplicationSpecPtr>());
  giskard::FrameMultiplicationSpecPtr s7 = node2.as<giskard::FrameMultiplicationSpecPtr>();
 
  ASSERT_EQ(s7->get_inputs().size(), 2);
  EXPECT_TRUE(s7->get_inputs()[0]->equals(*s1));
  EXPECT_TRUE(s7->get_inputs()[1]->equals(*s2));
}

TEST_F(YamlParserTest, ControllableConstraintSpec)
{
  std::string s = "controllable-constraint: [-0.1, 0.2, 5.0, 2]";

  YAML::Node node = YAML::Load(s);

  ASSERT_NO_THROW(node.as<giskard::ControllableConstraintSpec>());
  giskard::ControllableConstraintSpec spec = node.as<giskard::ControllableConstraintSpec>();

  EXPECT_DOUBLE_EQ(spec.lower_->get_expression(giskard::Scope())->value(), -0.1);
  EXPECT_DOUBLE_EQ(spec.upper_->get_expression(giskard::Scope())->value(), 0.2);
  EXPECT_DOUBLE_EQ(spec.weight_->get_expression(giskard::Scope())->value(), 5.0);
  EXPECT_EQ(spec.input_number_, 2);
}

TEST_F(YamlParserTest, SoftConstraintSpec)
{
  std::string s = "{soft-constraint: [-10.1, 120.2, 5.0, 1.1]}";

  YAML::Node node = YAML::Load(s);

  ASSERT_NO_THROW(node.as<giskard::SoftConstraintSpec>());
  giskard::SoftConstraintSpec spec = node.as<giskard::SoftConstraintSpec>();

  EXPECT_DOUBLE_EQ(spec.lower_->get_expression(giskard::Scope())->value(), -10.1);
  EXPECT_DOUBLE_EQ(spec.upper_->get_expression(giskard::Scope())->value(), 120.2);
  EXPECT_DOUBLE_EQ(spec.weight_->get_expression(giskard::Scope())->value(), 5.0);
  EXPECT_DOUBLE_EQ(spec.expression_->get_expression(giskard::Scope())->value(), 1.1);
}

TEST_F(YamlParserTest, HardConstraintSpec)
{
  std::string s = "{hard-constraint: [-10.1, 120.2, 1.1]}";

  YAML::Node node = YAML::Load(s);

  ASSERT_NO_THROW(node.as<giskard::HardConstraintSpec>());
  giskard::HardConstraintSpec spec = node.as<giskard::HardConstraintSpec>();

  EXPECT_DOUBLE_EQ(spec.lower_->get_expression(giskard::Scope())->value(), -10.1);
  EXPECT_DOUBLE_EQ(spec.upper_->get_expression(giskard::Scope())->value(), 120.2);
  EXPECT_DOUBLE_EQ(spec.expression_->get_expression(giskard::Scope())->value(), 1.1);
}

TEST_F(YamlParserTest, QPControllerSpec)
{
  std::string sc = "scope: []";
  std::string co = "controllable-constraints: [{controllable-constraint: [-0.1, 0.2, 5.0, 2]}]";
  std::string so = "soft-constraints: [{soft-constraint: [-10.1, 120.2, 5.0, 1.1]}]";
  std::string ha = "hard-constraints: [{hard-constraint: [-33.1, 110.3, 17.1]}]";

  std::string s = sc + "\n" + co + "\n" + so + "\n" + ha;

  YAML::Node node = YAML::Load(s);

  ASSERT_NO_THROW(node.as<giskard::QPControllerSpec>());
  giskard::QPControllerSpec spec = node.as<giskard::QPControllerSpec>();

  ASSERT_EQ(spec.scope_.size(), 0);
  ASSERT_EQ(spec.controllable_constraints_.size(), 1);
  ASSERT_EQ(spec.soft_constraints_.size(), 1);
  ASSERT_EQ(spec.hard_constraints_.size(), 1);

  EXPECT_DOUBLE_EQ(spec.controllable_constraints_[0].lower_->get_expression(giskard::Scope())->value(), -0.1);
  EXPECT_DOUBLE_EQ(spec.controllable_constraints_[0].upper_->get_expression(giskard::Scope())->value(), 0.2);
  EXPECT_DOUBLE_EQ(spec.controllable_constraints_[0].weight_->get_expression(giskard::Scope())->value(), 5.0);
  EXPECT_EQ(spec.controllable_constraints_[0].input_number_, 2);

  EXPECT_DOUBLE_EQ(spec.soft_constraints_[0].lower_->get_expression(giskard::Scope())->value(), -10.1);
  EXPECT_DOUBLE_EQ(spec.soft_constraints_[0].upper_->get_expression(giskard::Scope())->value(), 120.2);
  EXPECT_DOUBLE_EQ(spec.soft_constraints_[0].weight_->get_expression(giskard::Scope())->value(), 5.0);
  EXPECT_DOUBLE_EQ(spec.soft_constraints_[0].expression_->get_expression(giskard::Scope())->value(), 1.1);

  EXPECT_DOUBLE_EQ(spec.hard_constraints_[0].lower_->get_expression(giskard::Scope())->value(), -33.1);
  EXPECT_DOUBLE_EQ(spec.hard_constraints_[0].upper_->get_expression(giskard::Scope())->value(), 110.3);
  EXPECT_DOUBLE_EQ(spec.hard_constraints_[0].expression_->get_expression(giskard::Scope())->value(), 17.1);
}
