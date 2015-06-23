#include <gtest/gtest.h>
#include <giskard/giskard.hpp>
#include <boost/assign/list_of.hpp>

class YamlParserTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      valid_input_variable = "{name: trans-x, type: INPUT-VARIABLE}";
      valid_input_variable2 = "{name: trans-y, type: INPUT-VARIABLE}";
      valid_input_frame = "{name: my frame, type: INPUT-FRAME}";
      valid_input_list = "{inputs: [" + valid_input_variable + ", " + valid_input_frame
          + ", " + valid_input_variable2 + "]}";

      postfixes = boost::assign::list_of(" translation-x")(" translation-y")
          (" translation-z")(" rotation-x")(" rotation-y")(" rotation-z");

      for(unsigned int i=0; i<postfixes.size(); ++i)
        input_names.push_back("my frame" + postfixes[i]);

      valid_output_spec = "{name: trans-x, weight: 1.2, lower-velocity-limit: -0.1, upper-velocity-limit: 0.15}";
      valid_output_spec2 = "{name: trans-y, weight: 0.2, lower-velocity-limit: 0.1, upper-velocity-limit: 0.15}";
      valid_output_list = "{outputs: [" + valid_output_spec + ", " + valid_output_spec2 + "]}";
    }

    virtual void TearDown(){}

    // PARSING OF INPUTS
    std::string valid_input_variable, valid_input_variable2, 
        valid_input_frame, valid_input_list;
    std::vector<std::string> postfixes, input_names;

    // PARSING OF OUTPUTS
    std::string valid_output_spec, valid_output_spec2, valid_output_list;
};


TEST_F(YamlParserTest, ParseInputVariable)
{
  // Making sure we can parse a proper description
  YAML::Node valid_node = YAML::Load(valid_input_variable);
  EXPECT_TRUE(giskard::is_input_variable(valid_node));
  unsigned int input_index = 2;
  std::string input_name = "trans-x";

  KDL::Expression<double>::Ptr expression = 
      giskard::parse_input_variable(valid_node, input_index);
  KDL::InputTypePtr input = boost::static_pointer_cast< KDL::InputType >(expression);
 
  EXPECT_STREQ(input->name.c_str(), input_name.c_str());
  EXPECT_EQ(input->variable_number, input_index);

  // Catching various improper descriptions
  YAML::Node invalid_node1 = YAML::Load("{name: trans-x, type: INPUT-VAR}");
  YAML::Node invalid_node2 = YAML::Load("{name: trans-x, types: INPUT-VARIABLE}");
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
  YAML::Node valid_node = YAML::Load(valid_input_frame);
  EXPECT_TRUE(giskard::is_input_frame(valid_node));

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

TEST_F(YamlParserTest, ParseInputList)
{
  // Making sure we can parse a proper description
  YAML::Node valid_node = YAML::Load(valid_input_list);
  EXPECT_TRUE(giskard::is_input_list(valid_node));

  ASSERT_NO_THROW(giskard::parse_input_list(valid_node));
  std::vector< KDL::Expression<double>::Ptr > expressions =
      giskard::parse_input_list(valid_node);
  EXPECT_EQ(expressions.size(), 8);

  for(unsigned int i=0; i<expressions.size(); ++i)
  {
    KDL::InputTypePtr input = boost::static_pointer_cast< KDL::InputType >(expressions[i]);
    EXPECT_EQ(input->variable_number, i);   
    
    if(i==0)
      EXPECT_STREQ(input->name.c_str(), "trans-x");
    else if (i==7)
      EXPECT_STREQ(input->name.c_str(), "trans-y");
    else
      EXPECT_STREQ(input->name.c_str(), input_names[i-1].c_str());
  }
};

TEST_F(YamlParserTest, ParseInputListFromFile)
{
  YAML::Node node = YAML::LoadFile("input-list.yaml");
  EXPECT_TRUE(giskard::is_input_list(node));

  ASSERT_NO_THROW(giskard::parse_input_list(node));
  std::vector< KDL::Expression<double>::Ptr > expressions =
      giskard::parse_input_list(node);
  EXPECT_EQ(expressions.size(), 12);

  for(unsigned int i=0; i<expressions.size(); ++i)
  {
    KDL::InputTypePtr input = boost::static_pointer_cast< KDL::InputType >(expressions[i]);
    EXPECT_EQ(input->variable_number, i);   
    
    std::string input_name;
    if(i<6)
      input_name = "cup frame" + postfixes[i];
    else
      input_name = "stove frame" + postfixes[i-6];
    
    EXPECT_STREQ(input->name.c_str(), input_name.c_str());
  }
};


TEST_F(YamlParserTest, ParseOutSpecification)
{
  // Making sure we can parse a proper description
  YAML::Node valid_node = YAML::Load(valid_output_spec);
  EXPECT_TRUE(giskard::is_output_spec(valid_node));

  ASSERT_NO_THROW(giskard::parse_output_spec(valid_node));
  giskard::OutputSpec spec = giskard::parse_output_spec(valid_node);

  EXPECT_STREQ(spec.name_.c_str(), "trans-x");
  EXPECT_DOUBLE_EQ(spec.weight_, 1.2);
  EXPECT_DOUBLE_EQ(spec.lower_vel_limit_, -0.1);
  EXPECT_DOUBLE_EQ(spec.upper_vel_limit_, 0.15);
};

TEST_F(YamlParserTest, ParseOutputList)
{
  // Making sure we can parse a proper description
  YAML::Node node = YAML::Load(valid_output_list);
  EXPECT_TRUE(giskard::is_output_list(node));

  ASSERT_NO_THROW(giskard::parse_output_list(node));
  std::vector<giskard::OutputSpec> outputs = giskard::parse_output_list(node);

  EXPECT_EQ(outputs.size(), 2);
  EXPECT_STREQ(outputs[0].name_.c_str(), "trans-x");
  EXPECT_DOUBLE_EQ(outputs[0].weight_, 1.2);
  EXPECT_DOUBLE_EQ(outputs[0].lower_vel_limit_, -0.1);
  EXPECT_DOUBLE_EQ(outputs[0].upper_vel_limit_, 0.15);

  EXPECT_STREQ(outputs[1].name_.c_str(), "trans-y");
  EXPECT_DOUBLE_EQ(outputs[1].weight_, 0.2);
  EXPECT_DOUBLE_EQ(outputs[1].lower_vel_limit_, 0.1);
  EXPECT_DOUBLE_EQ(outputs[1].upper_vel_limit_, 0.15);
};
