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

TEST_F(YamlParserTest, ParseObservables)
{
  // PARSING INPUT-VARIABLE
  std::string obs_name1 = "trans-x";
  std::string obs_type1 = "INPUT-VARIABLE";
  std::string obs_input1 = "{name: " + obs_name1 + ", type: " + obs_type1 + "}";
  YAML::Node node = YAML::Load(obs_input1);

  ASSERT_NO_THROW(node.as<giskard::ObservableSpec>());
  giskard::ObservableSpec obs = node.as<giskard::ObservableSpec>();
  EXPECT_STREQ(obs.name_.c_str(), obs_name1.c_str());
  EXPECT_STREQ(obs.type_.c_str(), obs_type1.c_str());

  // PARSING INPUT-FRAME
  std::string obs_name2 = "cup frame";
  std::string obs_type2 = "INPUT-FRAME";
  std::string obs_input2 = "{name: " + obs_name2 + ", type: " + obs_type2 + "}";
  node = YAML::Load(obs_input2);

  ASSERT_NO_THROW(node.as<giskard::ObservableSpec>());
  obs = node.as<giskard::ObservableSpec>();
  EXPECT_STREQ(obs.name_.c_str(), obs_name2.c_str());
  EXPECT_STREQ(obs.type_.c_str(), obs_type2.c_str());

  // PARSING LIST OF INPUTS
  std::string obs_list_input = "[" + obs_input1 + ", " + obs_input2 + "]";
  node = YAML::Load(obs_list_input);
  std::vector<giskard::ObservableSpec> obs_list = 
      node.as< std::vector<giskard::ObservableSpec> >();
  ASSERT_EQ(obs_list.size(), 2);
  EXPECT_STREQ(obs_list[0].name_.c_str(), obs_name1.c_str()); 
  EXPECT_STREQ(obs_list[1].name_.c_str(), obs_name2.c_str()); 
  EXPECT_STREQ(obs_list[0].type_.c_str(), obs_type1.c_str()); 
  EXPECT_STREQ(obs_list[1].type_.c_str(), obs_type2.c_str()); 

  // ROUNDTRIP WITH YAML GENERATION
  YAML::Node node2;
  node2 = obs_list;
  std::vector<giskard::ObservableSpec> obs_list2 = 
      node.as< std::vector<giskard::ObservableSpec> >();
  ASSERT_EQ(obs_list2.size(), 2);
  EXPECT_STREQ(obs_list2[0].name_.c_str(), obs_name1.c_str()); 
  EXPECT_STREQ(obs_list2[1].name_.c_str(), obs_name2.c_str()); 
  EXPECT_STREQ(obs_list2[0].type_.c_str(), obs_type1.c_str()); 
  EXPECT_STREQ(obs_list2[1].type_.c_str(), obs_type2.c_str()); 
};

TEST_F(YamlParserTest, ParseControllables)
{
  std::string c1_name = "cup frame translation-z";
  std::string c2_name = "cup frame rotation-x";
  double c1_lower = -0.1;
  double c2_lower = -0.05;
  double c1_upper = 0.1;
  double c2_upper= 0.05;
  double c1_weight = 1.0;
  double c2_weight = 2.0;

  std::string c1_input = "{name: " + c1_name + ", lower_velocity_limit: " +
      boost::lexical_cast<std::string>(c1_lower) + ", upper_velocity_limit: " + 
      boost::lexical_cast<std::string>(c1_upper) + ", weight: " + 
      boost::lexical_cast<std::string>(c1_weight) + "}";
  std::string c2_input = "{name: " + c2_name + ", lower_velocity_limit: " +
      boost::lexical_cast<std::string>(c2_lower) + ", upper_velocity_limit: " + 
      boost::lexical_cast<std::string>(c2_upper) + ", weight: " + 
      boost::lexical_cast<std::string>(c2_weight) + "}";
  std::string c_list_input = "[" + c1_input + ", " + c2_input + "]";

  // PARSING A CONTROLLABLE SPEC
  YAML::Node node = YAML::Load(c1_input);
  ASSERT_NO_THROW(node.as<giskard::ControllableSpec>());
  giskard::ControllableSpec c1 = node.as<giskard::ControllableSpec>();
  EXPECT_STREQ(c1.name_.c_str(), c1_name.c_str());
  EXPECT_DOUBLE_EQ(c1.lower_vel_limit_, c1_lower);
  EXPECT_DOUBLE_EQ(c1.upper_vel_limit_, c1_upper);
  EXPECT_DOUBLE_EQ(c1.weight_, c1_weight);

  // PARSING A LIST OF CONTROLLABLE SPECS
  node = YAML::Load(c_list_input);
  std::vector<giskard::ControllableSpec> c_list = 
      node.as< std::vector<giskard::ControllableSpec> >();
  ASSERT_EQ(c_list.size(), 2);
  EXPECT_STREQ(c_list[0].name_.c_str(), c1_name.c_str()); 
  EXPECT_STREQ(c_list[1].name_.c_str(), c2_name.c_str()); 
  EXPECT_DOUBLE_EQ(c_list[0].weight_, c1_weight);
  EXPECT_DOUBLE_EQ(c_list[1].weight_, c2_weight);
  EXPECT_DOUBLE_EQ(c_list[0].lower_vel_limit_, c1_lower);
  EXPECT_DOUBLE_EQ(c_list[1].lower_vel_limit_, c2_lower);
  EXPECT_DOUBLE_EQ(c_list[0].upper_vel_limit_, c1_upper);
  EXPECT_DOUBLE_EQ(c_list[1].upper_vel_limit_, c2_upper);

  // ROUNDTRIP WITH YAML GENERATION
  YAML::Node node2;
  node2 = c_list;
  std::vector<giskard::ControllableSpec> c_list2 = 
      node.as< std::vector<giskard::ControllableSpec> >();
  ASSERT_EQ(c_list2.size(), 2);
  EXPECT_STREQ(c_list2[0].name_.c_str(), c1_name.c_str()); 
  EXPECT_STREQ(c_list2[1].name_.c_str(), c2_name.c_str()); 
  EXPECT_DOUBLE_EQ(c_list2[0].weight_, c1_weight);
  EXPECT_DOUBLE_EQ(c_list2[1].weight_, c2_weight);
  EXPECT_DOUBLE_EQ(c_list2[0].lower_vel_limit_, c1_lower);
  EXPECT_DOUBLE_EQ(c_list2[1].lower_vel_limit_, c2_lower);
  EXPECT_DOUBLE_EQ(c_list2[0].upper_vel_limit_, c1_upper);
  EXPECT_DOUBLE_EQ(c_list2[1].upper_vel_limit_, c2_upper);
};
