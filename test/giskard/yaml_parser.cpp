#include <gtest/gtest.h>
#include <giskard/giskard.hpp>

class YamlParserTest : public ::testing::Test
{
  protected:
    virtual void SetUp(){}
    virtual void TearDown(){}
};


TEST_F(YamlParserTest, ParseInputVariable)
{
  YAML::Node valid_node = YAML::Load("{name: trans-x, type: INPUT-VAR}");
  EXPECT_TRUE(giskard::is_input_variable(valid_node));
  unsigned int input_index = 2;

  KDL::Expression<double>::Ptr expression = 
      giskard::parse_input_variable(valid_node, input_index);
  KDL::InputTypePtr input = boost::static_pointer_cast< KDL::InputType >(expression);
 
  EXPECT_STREQ(input->name.c_str(), "trans-x");
  EXPECT_EQ(input->variable_number, input_index);

  YAML::Node invalid_node1 = YAML::Load("{names: trans-x, type: INPUT-VAR}");
  YAML::Node invalid_node2 = YAML::Load("{name: trans-x, types: INPUT-VAR}");
  YAML::Node invalid_node3 = YAML::Load("{type: INPUT-VAR}");
  YAML::Node invalid_node4 = YAML::Load("{name: trans-x}");
  YAML::Node invalid_node5 = YAML::Load("[1.0, 2.0]");
 
  EXPECT_FALSE(giskard::is_input_variable(invalid_node1));
  EXPECT_FALSE(giskard::is_input_variable(invalid_node2));
  EXPECT_FALSE(giskard::is_input_variable(invalid_node3));
  EXPECT_FALSE(giskard::is_input_variable(invalid_node4));
  EXPECT_FALSE(giskard::is_input_variable(invalid_node5));

  EXPECT_THROW(giskard::parse_input_variable(invalid_node1, input_index),
     giskard::YamlParserException); 
  EXPECT_THROW(giskard::parse_input_variable(invalid_node2, input_index),
     giskard::YamlParserException); 
  EXPECT_THROW(giskard::parse_input_variable(invalid_node3, input_index),
     giskard::YamlParserException); 
  EXPECT_THROW(giskard::parse_input_variable(invalid_node4, input_index),
     giskard::YamlParserException); 
  EXPECT_THROW(giskard::parse_input_variable(invalid_node5, input_index),
     giskard::YamlParserException); 
};
