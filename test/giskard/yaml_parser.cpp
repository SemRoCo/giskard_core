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
  ASSERT_NO_THROW(node.as<giskard::ConstDoubleSpecPtr>());
  giskard::ConstDoubleSpecPtr s1 = node.as<giskard::ConstDoubleSpecPtr>();

  EXPECT_DOUBLE_EQ(1.1, s1->get_value());
  EXPECT_FALSE(s1->get_cached());

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard::ConstDoubleSpecPtr>());
  giskard::ConstDoubleSpecPtr s2 = node2.as<giskard::ConstDoubleSpecPtr>();

  EXPECT_DOUBLE_EQ(s1->get_value(), s2->get_value());
  EXPECT_EQ(s1->get_cached(), s2->get_cached());

  // parsing to double spec
  ASSERT_NO_THROW(node.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr s3 = node.as<giskard::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s3).get());
  giskard::ConstDoubleSpecPtr s4 = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s3);

  EXPECT_DOUBLE_EQ(1.1, s4->get_value());
  EXPECT_FALSE(s4->get_cached());

  // roundtrip with generation to double spec
  YAML::Node node3;
  node3 = s3;
  ASSERT_NO_THROW(node3.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr s5 = node3.as<giskard::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s5).get());
  giskard::ConstDoubleSpecPtr s6 = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s5);

  EXPECT_DOUBLE_EQ(1.1, s6->get_value());
  EXPECT_FALSE(s6->get_cached());
};

TEST_F(YamlParserTest, InputExpression)
{
  std::string i = "{type: INPUT, input-number: 2}";

  // parsing input double
  YAML::Node node = YAML::Load(i);
  ASSERT_NO_THROW(node.as<giskard::InputDoubleSpecPtr>());
  giskard::InputDoubleSpecPtr s1 = node.as<giskard::InputDoubleSpecPtr>();

  EXPECT_EQ(2, s1->get_input_num());
  EXPECT_FALSE(s1->get_cached());

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard::InputDoubleSpecPtr>());
  giskard::InputDoubleSpecPtr s2 = node2.as<giskard::InputDoubleSpecPtr>();

  EXPECT_EQ(s1->get_input_num(), s2->get_input_num());
  EXPECT_EQ(s1->get_cached(), s2->get_cached());

  // parsing to double spec
  ASSERT_NO_THROW(node.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr s3 = node.as<giskard::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s3).get());
  giskard::InputDoubleSpecPtr s4 = boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s3);

  EXPECT_EQ(2, s4->get_input_num());
  EXPECT_FALSE(s4->get_cached());

  // roundtrip with generation to double spec
  YAML::Node node3;
  node3 = s3;
  ASSERT_NO_THROW(node3.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr s5 = node3.as<giskard::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s5).get());
  giskard::InputDoubleSpecPtr s6 = boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s5);

  EXPECT_EQ(2, s6->get_input_num());
  EXPECT_FALSE(s6->get_cached());
};

TEST_F(YamlParserTest, ConstructorVectorSpec)
{
  std::string v = "{type: VECTOR3, inputs: [1.1, 2.2, 3.3]}";

  // parsing constructor vector
  YAML::Node node = YAML::Load(v);
  ASSERT_NO_THROW(node.as<giskard::ConstructorVectorSpecPtr>());
  giskard::ConstructorVectorSpecPtr s1 = node.as<giskard::ConstructorVectorSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s1->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s1->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s1->get_z()).get());

  giskard::ConstDoubleSpecPtr x = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s1->get_x());
  EXPECT_FALSE(x->get_cached());
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  giskard::ConstDoubleSpecPtr y = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s1->get_y());
  EXPECT_FALSE(y->get_cached());
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  giskard::ConstDoubleSpecPtr z = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s1->get_z());
  EXPECT_FALSE(z->get_cached());
  EXPECT_DOUBLE_EQ(z->get_value(), 3.3);

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard::ConstructorVectorSpecPtr>());
  giskard::ConstructorVectorSpecPtr s2 = node2.as<giskard::ConstructorVectorSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s2->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s2->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s2->get_z()).get());

  x = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s2->get_x());
  EXPECT_FALSE(x->get_cached());
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  y = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s2->get_y());
  EXPECT_FALSE(y->get_cached());
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  z = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s2->get_z());
  EXPECT_FALSE(z->get_cached());
  EXPECT_DOUBLE_EQ(z->get_value(), 3.3);

  // parsing to vector spec
  ASSERT_NO_THROW(node.as<giskard::VectorSpecPtr>());
  giskard::VectorSpecPtr s3 = node.as<giskard::VectorSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s3).get());
  giskard::ConstructorVectorSpecPtr s4 = boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s3);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s4->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s4->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s4->get_z()).get());

  x = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s4->get_x());
  EXPECT_FALSE(x->get_cached());
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  y = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s4->get_y());
  EXPECT_FALSE(y->get_cached());
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  z = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s4->get_z());
  EXPECT_FALSE(z->get_cached());
  EXPECT_DOUBLE_EQ(z->get_value(), 3.3);

  // roundtrip with generation to double spec
  YAML::Node node3;
  node3 = s3;
  ASSERT_NO_THROW(node3.as<giskard::VectorSpecPtr>());
  giskard::VectorSpecPtr s5 = node3.as<giskard::VectorSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s5).get());
  giskard::ConstructorVectorSpecPtr s6 = boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s5);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s6->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s6->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s6->get_z()).get());

  x = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s6->get_x());
  EXPECT_FALSE(x->get_cached());
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  y = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s6->get_y());
  EXPECT_FALSE(y->get_cached());
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  z = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s6->get_z());
  EXPECT_FALSE(z->get_cached());
  EXPECT_DOUBLE_EQ(z->get_value(), 3.3);
};

TEST_F(YamlParserTest, AxisAngleSpec)
{
  std::string r = "{type: ROTATION, axis: {type: VECTOR3, inputs: [1.0, 0.0, 0.0]}, angle: {type: INPUT, input-number: 3}}"; 

  // parsing into axis angle specification
  YAML::Node node = YAML::Load(r);

  ASSERT_NO_THROW(node.as<giskard::AxisAngleSpecPtr>());
  giskard::AxisAngleSpecPtr s1 = node.as<giskard::AxisAngleSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s1->get_angle()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s1->get_axis()).get());

  giskard::InputDoubleSpecPtr angle = 
      boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s1->get_angle());
  giskard::ConstructorVectorSpecPtr axis = 
      boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s1->get_axis());

  EXPECT_EQ(angle->get_input_num(), 3);
  EXPECT_FALSE(angle->get_cached());

  ASSERT_TRUE(axis->get_x().get());
  ASSERT_TRUE(axis->get_y().get());
  ASSERT_TRUE(axis->get_z().get());

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_x()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_x())->get_value(), 1.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_y()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_y())->get_value(), 0.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_z())->get_value(), 0.0);

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;

  ASSERT_NO_THROW(node2.as<giskard::AxisAngleSpecPtr>());
  giskard::AxisAngleSpecPtr s2 = node2.as<giskard::AxisAngleSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s2->get_angle()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s2->get_axis()).get());

  angle = boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s2->get_angle());
  axis = boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s2->get_axis());

  EXPECT_EQ(angle->get_input_num(), 3);
  EXPECT_FALSE(angle->get_cached());

  ASSERT_TRUE(axis->get_x().get());
  ASSERT_TRUE(axis->get_y().get());
  ASSERT_TRUE(axis->get_z().get());

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_x()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_x())->get_value(), 1.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_y()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_y())->get_value(), 0.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_z())->get_value(), 0.0);

  // parsing to rotation spec
  ASSERT_NO_THROW(node.as<giskard::RotationSpecPtr>());
  giskard::RotationSpecPtr s3 = node.as<giskard::RotationSpecPtr>();
  
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s3).get());
  giskard::AxisAngleSpecPtr s4 = boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s3);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s4->get_angle()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s4->get_axis()).get());

  angle = boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s4->get_angle());
  axis = boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s4->get_axis());

  EXPECT_EQ(angle->get_input_num(), 3);
  EXPECT_FALSE(angle->get_cached());

  ASSERT_TRUE(axis->get_x().get());
  ASSERT_TRUE(axis->get_y().get());
  ASSERT_TRUE(axis->get_z().get());

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_x()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_x())->get_value(), 1.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_y()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_y())->get_value(), 0.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_z())->get_value(), 0.0);

  // roundtrip with generation and parsing of rotation specification
  YAML::Node node3;
  node3 = s3;

  ASSERT_NO_THROW(node3.as<giskard::RotationSpecPtr>());
  giskard::RotationSpecPtr s5 = node3.as<giskard::RotationSpecPtr>();
  
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s5).get());
  giskard::AxisAngleSpecPtr s6 = boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s5);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s6->get_angle()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s6->get_axis()).get());

  angle = boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s6->get_angle());
  axis = boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s6->get_axis());

  EXPECT_EQ(angle->get_input_num(), 3);
  EXPECT_FALSE(angle->get_cached());

  ASSERT_TRUE(axis->get_x().get());
  ASSERT_TRUE(axis->get_y().get());
  ASSERT_TRUE(axis->get_z().get());

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_x()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_x())->get_value(), 1.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_y()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_y())->get_value(), 0.0);
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(axis->get_z())->get_value(), 0.0);
};

TEST_F(YamlParserTest, ConstructorFrameSpec)
{
  std::string r = "{type: ROTATION, axis: {type: VECTOR3, inputs: [0.0, -2.0, 0.0]}, angle: " + boost::lexical_cast<std::string>(M_PI/-2.0) + "}";
  std::string t = "{type: VECTOR3, inputs: [1.1, 2.2, 3.3]}";
  std::string f = "{type: FRAME, rotation: " + r + ", translation: " + t + "}";

  // parsing into axis angle specification
  YAML::Node node = YAML::Load(f);

  ASSERT_NO_THROW(node.as<giskard::ConstructorFrameSpecPtr>());
  giskard::ConstructorFrameSpecPtr s1 = node.as<giskard::ConstructorFrameSpecPtr>();

  ASSERT_TRUE(s1->get_rotation().get());
  ASSERT_TRUE(s1->get_translation().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s1->get_rotation()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s1->get_translation()).get());
  giskard::AxisAngleSpecPtr rot =
      boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s1->get_rotation());
  giskard::ConstructorVectorSpecPtr trans =
      boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s1->get_translation());

  ASSERT_TRUE(rot->get_axis().get());
  ASSERT_TRUE(rot->get_angle().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(rot->get_axis()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot->get_angle()).get());
  giskard::ConstructorVectorSpecPtr rot_axis = 
      boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(rot->get_axis());
  giskard::ConstDoubleSpecPtr rot_angle =
      boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot->get_angle());

  ASSERT_TRUE(rot_axis->get_x().get());
  ASSERT_TRUE(rot_axis->get_y().get());
  ASSERT_TRUE(rot_axis->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_x())->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_y())->get_value(), -2.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_z())->get_value(), 0.0);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_angle).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_angle)->get_value(), M_PI/-2.0);

  ASSERT_TRUE(trans->get_x().get());
  ASSERT_TRUE(trans->get_y().get());
  ASSERT_TRUE(trans->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_x())->get_value(), 1.1);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_y())->get_value(), 2.2);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_z())->get_value(), 3.3);

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;

  ASSERT_NO_THROW(node2.as<giskard::ConstructorFrameSpecPtr>());
  giskard::ConstructorFrameSpecPtr s2 = node2.as<giskard::ConstructorFrameSpecPtr>();

  ASSERT_TRUE(s2->get_rotation().get());
  ASSERT_TRUE(s2->get_translation().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s2->get_rotation()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s2->get_translation()).get());
  rot = boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s2->get_rotation());
  trans = boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s2->get_translation());

  ASSERT_TRUE(rot->get_axis().get());
  ASSERT_TRUE(rot->get_angle().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(rot->get_axis()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot->get_angle()).get());
  rot_axis = boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(rot->get_axis());
  rot_angle = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot->get_angle());

  ASSERT_TRUE(rot_axis->get_x().get());
  ASSERT_TRUE(rot_axis->get_y().get());
  ASSERT_TRUE(rot_axis->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_x())->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_y())->get_value(), -2.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_z())->get_value(), 0.0);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_angle).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_angle)->get_value(), M_PI/-2.0);

  ASSERT_TRUE(trans->get_x().get());
  ASSERT_TRUE(trans->get_y().get());
  ASSERT_TRUE(trans->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_x())->get_value(), 1.1);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_y())->get_value(), 2.2);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_z())->get_value(), 3.3);

  // parsing into frame specification
  ASSERT_NO_THROW(node.as<giskard::FrameSpecPtr>());
  giskard::FrameSpecPtr s3 = node.as<giskard::FrameSpecPtr>();

  ASSERT_TRUE(s3.get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorFrameSpec>(s3).get());
  giskard::ConstructorFrameSpecPtr s4 =
      boost::dynamic_pointer_cast<giskard::ConstructorFrameSpec>(s3);

  ASSERT_TRUE(s4->get_rotation().get());
  ASSERT_TRUE(s4->get_translation().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s4->get_rotation()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s4->get_translation()).get());
  rot = boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s4->get_rotation());
  trans = boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s4->get_translation());

  ASSERT_TRUE(rot->get_axis().get());
  ASSERT_TRUE(rot->get_angle().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(rot->get_axis()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot->get_angle()).get());
  rot_axis = boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(rot->get_axis());
  rot_angle = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot->get_angle());

  ASSERT_TRUE(rot_axis->get_x().get());
  ASSERT_TRUE(rot_axis->get_y().get());
  ASSERT_TRUE(rot_axis->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_x())->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_y())->get_value(), -2.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_z())->get_value(), 0.0);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_angle).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_angle)->get_value(), M_PI/-2.0);

  ASSERT_TRUE(trans->get_x().get());
  ASSERT_TRUE(trans->get_y().get());
  ASSERT_TRUE(trans->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_x())->get_value(), 1.1);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_y())->get_value(), 2.2);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_z())->get_value(), 3.3);

  // roundtrip with generation and parsing into frame specification
  YAML::Node node3;
  node3 = s3;

  ASSERT_NO_THROW(node3.as<giskard::FrameSpecPtr>());
  giskard::FrameSpecPtr s5 = node3.as<giskard::FrameSpecPtr>();

  ASSERT_TRUE(s5.get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorFrameSpec>(s5).get());
  giskard::ConstructorFrameSpecPtr s6 =
      boost::dynamic_pointer_cast<giskard::ConstructorFrameSpec>(s6);

  ASSERT_TRUE(s6->get_rotation().get());
  ASSERT_TRUE(s6->get_translation().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s6->get_rotation()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s6->get_translation()).get());
  rot = boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(s6->get_rotation());
  trans = boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(s6->get_translation());

  ASSERT_TRUE(rot->get_axis().get());
  ASSERT_TRUE(rot->get_angle().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(rot->get_axis()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot->get_angle()).get());
  rot_axis = boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(rot->get_axis());
  rot_angle = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot->get_angle());

  ASSERT_TRUE(rot_axis->get_x().get());
  ASSERT_TRUE(rot_axis->get_y().get());
  ASSERT_TRUE(rot_axis->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_x())->get_value(), 0.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_y())->get_value(), -2.0);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_axis->get_z())->get_value(), 0.0);

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_angle).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rot_angle)->get_value(), M_PI/-2.0);

  ASSERT_TRUE(trans->get_x().get());
  ASSERT_TRUE(trans->get_y().get());
  ASSERT_TRUE(trans->get_z().get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_x()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_y()).get());
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_z()).get());
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_x())->get_value(), 1.1);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_y())->get_value(), 2.2);
  EXPECT_DOUBLE_EQ(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(trans->get_z())->get_value(), 3.3);
};

TEST_F(YamlParserTest, MultiplicationFrameSpec)
{
  std::string r1 = "{type: ROTATION, axis: {type: VECTOR3, inputs: [1.0, 0.0, 0.0]}, angle: 1.0}";
  std::string r2 = "{type: ROTATION, axis: {type: VECTOR3, inputs: [0.0, 1.0, 0.0]}, angle: -0.5}";
  std::string t1 = "{type: VECTOR3, inputs: [0.1, 0.2, 0.3]}";
  std::string t2 = "{type: VECTOR3, inputs: [-1.1, -2.2, -3.3]}";
  std::string f1 = "{type: FRAME, rotation: " + r1 + ", translation: " + t1 + "}";
  std::string f2 = "{type: FRAME, rotation: " + r2 + ", translation: " + t2 + "}";
  std::string f3 = "{type: FRAME-MULTIPLICATION, inputs: [" + f1 + ", " + f2 + "]}";
  YAML::Node node = YAML::Load(f1);
  ASSERT_NO_THROW(node.as<giskard::ConstructorFrameSpecPtr>());
  giskard::ConstructorFrameSpecPtr s1 = node.as<giskard::ConstructorFrameSpecPtr>();
  node = YAML::Load(f2);
  ASSERT_NO_THROW(node.as<giskard::ConstructorFrameSpecPtr>());
  giskard::ConstructorFrameSpecPtr s2 = node.as<giskard::ConstructorFrameSpecPtr>();
 
  // parsing into frame-multiplication specification
  node = YAML::Load(f3);
  ASSERT_NO_THROW(node.as<giskard::MultiplicationFrameSpecPtr>());
  giskard::MultiplicationFrameSpecPtr s3 = node.as<giskard::MultiplicationFrameSpecPtr>();
 
  ASSERT_EQ(s3->get_inputs().size(), 2);
  EXPECT_TRUE(s3->get_inputs()[0]->equals(*s1));
  EXPECT_TRUE(s3->get_inputs()[1]->equals(*s2));

  // roundtrip with generation
  YAML::Node node2;
  node2 = s3;
  ASSERT_NO_THROW(node2.as<giskard::MultiplicationFrameSpecPtr>());
  giskard::MultiplicationFrameSpecPtr s4 = node2.as<giskard::MultiplicationFrameSpecPtr>();
 
  ASSERT_EQ(s4->get_inputs().size(), 2);
  EXPECT_TRUE(s4->get_inputs()[0]->equals(*s1));
  EXPECT_TRUE(s4->get_inputs()[1]->equals(*s2));

  // parsing into frame specification
  ASSERT_NO_THROW(node.as<giskard::FrameSpecPtr>());
  giskard::FrameSpecPtr s5 = node.as<giskard::FrameSpecPtr>();

  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::MultiplicationFrameSpec>(s5).get());
  giskard::MultiplicationFrameSpecPtr s6 = boost::dynamic_pointer_cast<giskard::MultiplicationFrameSpec>(s5);
 
  ASSERT_EQ(s6->get_inputs().size(), 2);
  EXPECT_TRUE(s6->get_inputs()[0]->equals(*s1));
  EXPECT_TRUE(s6->get_inputs()[1]->equals(*s2));

  //roundtrip with generation from frame specification
  YAML::Node node3;
  node3 = s5;
  
  ASSERT_NO_THROW(node3.as<giskard::MultiplicationFrameSpecPtr>());
  giskard::MultiplicationFrameSpecPtr s7 = node2.as<giskard::MultiplicationFrameSpecPtr>();
 
  ASSERT_EQ(s7->get_inputs().size(), 2);
  EXPECT_TRUE(s7->get_inputs()[0]->equals(*s1));
  EXPECT_TRUE(s7->get_inputs()[1]->equals(*s2));
}
