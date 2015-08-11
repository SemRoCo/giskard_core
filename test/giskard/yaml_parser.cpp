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
  EXPECT_STREQ(s1->get_name().c_str(), "");

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard::ConstDoubleSpecPtr>());
  giskard::ConstDoubleSpecPtr s2 = node2.as<giskard::ConstDoubleSpecPtr>();

  EXPECT_DOUBLE_EQ(s1->get_value(), s2->get_value());
  EXPECT_EQ(s1->get_cached(), s2->get_cached());
  EXPECT_STREQ(s1->get_name().c_str(), s2->get_name().c_str());

  // parsing to double spec
  ASSERT_NO_THROW(node.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr s3 = node.as<giskard::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s3).get());
  giskard::ConstDoubleSpecPtr s4 = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s3);

  EXPECT_DOUBLE_EQ(1.1, s4->get_value());
  EXPECT_FALSE(s4->get_cached());
  EXPECT_STREQ(s4->get_name().c_str(), "");

  // roundtrip with generation to double spec
  YAML::Node node3;
  node3 = s3;
  ASSERT_NO_THROW(node3.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr s5 = node3.as<giskard::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s5).get());
  giskard::ConstDoubleSpecPtr s6 = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s5);

  EXPECT_DOUBLE_EQ(1.1, s6->get_value());
  EXPECT_FALSE(s6->get_cached());
  EXPECT_STREQ(s6->get_name().c_str(), "");
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
  EXPECT_STREQ(s1->get_name().c_str(), "");

  // roundtrip with generation
  YAML::Node node2;
  node2 = s1;
  ASSERT_NO_THROW(node2.as<giskard::InputDoubleSpecPtr>());
  giskard::InputDoubleSpecPtr s2 = node2.as<giskard::InputDoubleSpecPtr>();

  EXPECT_EQ(s1->get_input_num(), s2->get_input_num());
  EXPECT_EQ(s1->get_cached(), s2->get_cached());
  EXPECT_STREQ(s1->get_name().c_str(), s2->get_name().c_str());

  // parsing to double spec
  ASSERT_NO_THROW(node.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr s3 = node.as<giskard::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s3).get());
  giskard::InputDoubleSpecPtr s4 = boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s3);

  EXPECT_EQ(2, s4->get_input_num());
  EXPECT_FALSE(s4->get_cached());
  EXPECT_STREQ(s4->get_name().c_str(), "");

  // roundtrip with generation to double spec
  YAML::Node node3;
  node3 = s3;
  ASSERT_NO_THROW(node3.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr s5 = node3.as<giskard::DoubleSpecPtr>();
  ASSERT_TRUE(boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s5).get());
  giskard::InputDoubleSpecPtr s6 = boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(s5);

  EXPECT_EQ(2, s6->get_input_num());
  EXPECT_FALSE(s6->get_cached());
  EXPECT_STREQ(s6->get_name().c_str(), "");
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
  EXPECT_STREQ(x->get_name().c_str(), "");
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  giskard::ConstDoubleSpecPtr y = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s1->get_y());
  EXPECT_FALSE(y->get_cached());
  EXPECT_STREQ(y->get_name().c_str(), "");
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  giskard::ConstDoubleSpecPtr z = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s1->get_z());
  EXPECT_FALSE(z->get_cached());
  EXPECT_STREQ(z->get_name().c_str(), "");
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
  EXPECT_STREQ(x->get_name().c_str(), "");
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  y = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s2->get_y());
  EXPECT_FALSE(y->get_cached());
  EXPECT_STREQ(y->get_name().c_str(), "");
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  z = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s2->get_z());
  EXPECT_FALSE(z->get_cached());
  EXPECT_STREQ(z->get_name().c_str(), "");
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
  EXPECT_STREQ(x->get_name().c_str(), "");
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  y = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s4->get_y());
  EXPECT_FALSE(y->get_cached());
  EXPECT_STREQ(y->get_name().c_str(), "");
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  z = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s4->get_z());
  EXPECT_FALSE(z->get_cached());
  EXPECT_STREQ(z->get_name().c_str(), "");
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
  EXPECT_STREQ(x->get_name().c_str(), "");
  EXPECT_DOUBLE_EQ(x->get_value(), 1.1);

  y = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s6->get_y());
  EXPECT_FALSE(y->get_cached());
  EXPECT_STREQ(y->get_name().c_str(), "");
  EXPECT_DOUBLE_EQ(y->get_value(), 2.2);

  z = boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(s6->get_z());
  EXPECT_FALSE(z->get_cached());
  EXPECT_STREQ(z->get_name().c_str(), "");
  EXPECT_DOUBLE_EQ(z->get_value(), 3.3);
};
