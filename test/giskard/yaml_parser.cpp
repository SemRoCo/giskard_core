#include <gtest/gtest.h>
#include <giskard/giskard.hpp>
#include <boost/assign/list_of.hpp>

class YamlParserTest : public ::testing::Test
{
  protected:
    virtual void SetUp(){}
    virtual void TearDown(){}
};


TEST_F(YamlParserTest, ParseInputVariable)
{
  // Making sure we can parse a proper description
  YAML::Node valid_node = YAML::Load("{name: trans-x, type: INPUT-VARIABLE}");
  EXPECT_TRUE(giskard::is_input_variable(valid_node));
  unsigned int input_index = 2;

  KDL::Expression<double>::Ptr expression = 
      giskard::parse_input_variable(valid_node, input_index);
  KDL::InputTypePtr input = boost::static_pointer_cast< KDL::InputType >(expression);
 
  EXPECT_STREQ(input->name.c_str(), "trans-x");
  EXPECT_EQ(input->variable_number, input_index);

  // Catching various improper descriptions
  YAML::Node invalid_node1 = YAML::Load("{name: trans-x, type: INPUT-VAR}");
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

TEST_F(YamlParserTest, ParseInputFrame)
{
  // Making sure we can parse a proper description
  YAML::Node valid_node = YAML::Load("{name: my frame, type: INPUT-FRAME}");
  EXPECT_TRUE(giskard::is_input_frame(valid_node));

  std::vector<std::string> input_names = boost::assign::list_of("my frame translation-x")
      ("my frame translation-y")("my frame translation-z")("my frame rotation-x")
      ("my frame rotation-y")("my frame rotation-z");

  unsigned int start_index = 2;
  std::vector< KDL::Expression<double>::Ptr > expressions = 
      giskard::parse_input_frame(valid_node, start_index);

  ASSERT_EQ(expressions.size(), input_names.size() );
  for(unsigned int i=0; i<input_names.size(); ++i)
  {
    KDL::InputTypePtr input = boost::static_pointer_cast< KDL::InputType >(expressions[i]);
   
    EXPECT_STREQ(input->name.c_str(), input_names[i].c_str());
    EXPECT_EQ(input->variable_number, start_index + i);
  }

  // Catching various improper descriptions
  YAML::Node invalid_node1 = YAML::Load("{name: my frame, type: INPUT}");
  YAML::Node invalid_node2 = YAML::Load("{name: my frame}");
  YAML::Node invalid_node3 = YAML::Load("{type: INPUT-FRAME}");
  YAML::Node invalid_node4 = YAML::Load("[1.0, 2.0, 3.0]");

  EXPECT_FALSE(giskard::is_input_variable(invalid_node1));
  EXPECT_FALSE(giskard::is_input_variable(invalid_node2));
  EXPECT_FALSE(giskard::is_input_variable(invalid_node3));
  EXPECT_FALSE(giskard::is_input_variable(invalid_node4));

  EXPECT_THROW(giskard::parse_input_variable(invalid_node1, start_index),
     giskard::YamlParserException); 
  EXPECT_THROW(giskard::parse_input_variable(invalid_node2, start_index),
     giskard::YamlParserException); 
  EXPECT_THROW(giskard::parse_input_variable(invalid_node3, start_index),
     giskard::YamlParserException); 
  EXPECT_THROW(giskard::parse_input_variable(invalid_node4, start_index),
     giskard::YamlParserException); 
};
