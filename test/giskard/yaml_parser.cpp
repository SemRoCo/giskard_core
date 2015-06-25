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
  std::string c1_ref = "cup frame translation-z";
  std::string c1_name = c1_ref + " velocity";
  std::string c2_ref = "cup frame rotation-x";
  std::string c2_name = c2_ref + " velocity";
 
  std::string c1_input = "{name: " + c1_name + ", type: VELOCITY-OF, " +
      "reference: " + c1_ref + "}";
  std::string c2_input = "{name: " + c2_name + ", type: VELOCITY-OF, " +
      "reference: " + c2_ref + "}";
  std::string c_list_input = "[" + c1_input + ", " + c2_input + "]";

  // PARSING A CONTROLLABLE SPEC
  YAML::Node node = YAML::Load(c1_input);
  ASSERT_NO_THROW(node.as<giskard::ControllableSpec>());
  giskard::ControllableSpec c1 = node.as<giskard::ControllableSpec>();
  EXPECT_STREQ(c1.name_.c_str(), c1_name.c_str());
  EXPECT_STREQ(c1.type_.c_str(), "VELOCITY-OF");
  EXPECT_STREQ(c1.reference_.c_str(), c1_ref.c_str());

  // PARSING A LIST OF CONTROLLABLE SPECS
  node = YAML::Load(c_list_input);
  ASSERT_NO_THROW(node.as< std::vector<giskard::ControllableSpec> >());
  std::vector<giskard::ControllableSpec> c_list = 
      node.as< std::vector<giskard::ControllableSpec> >();
  ASSERT_EQ(c_list.size(), 2);
  EXPECT_STREQ(c_list[0].name_.c_str(), c1_name.c_str()); 
  EXPECT_STREQ(c_list[1].name_.c_str(), c2_name.c_str()); 
  EXPECT_STREQ(c_list[0].type_.c_str(), "VELOCITY-OF");
  EXPECT_STREQ(c_list[1].type_.c_str(), "VELOCITY-OF");
  EXPECT_STREQ(c_list[0].reference_.c_str(), c1_ref.c_str()); 
  EXPECT_STREQ(c_list[1].reference_.c_str(), c2_ref.c_str()); 

  // ROUNDTRIP WITH YAML GENERATION
  YAML::Node node2;
  node2 = c_list;
  ASSERT_NO_THROW(node2.as< std::vector<giskard::ControllableSpec> >());
  std::vector<giskard::ControllableSpec> c_list2 = 
      node.as< std::vector<giskard::ControllableSpec> >();
  ASSERT_EQ(c_list2.size(), 2);
  EXPECT_STREQ(c_list2[0].name_.c_str(), c1_name.c_str()); 
  EXPECT_STREQ(c_list2[1].name_.c_str(), c2_name.c_str()); 
  EXPECT_STREQ(c_list2[0].type_.c_str(), "VELOCITY-OF");
  EXPECT_STREQ(c_list2[1].type_.c_str(), "VELOCITY-OF");
  EXPECT_STREQ(c_list2[0].reference_.c_str(), c1_ref.c_str()); 
  EXPECT_STREQ(c_list2[1].reference_.c_str(), c2_ref.c_str()); 
};

TEST_F(YamlParserTest, ParseExpressions)
{
  std::string ref_name = "cup frame rotation-z";
  std::string rot_name = "cup rotation";
  std::string cup_top_name = "cup top centroid";
  std::string cup_bottom_name = "cup bottom centroid";
  std::string cup_rel_name = "cup upright";
  double constant1 = 2.3;
  double constant2 = -1.1;
  std::string constant1_input = boost::lexical_cast<std::string>(constant1);
  std::string constant2_input = boost::lexical_cast<std::string>(constant2);
  std::string anonymous_vector_input = "{type: VECTOR3, inputs: [" +
      constant1_input + ", " + ref_name + ", " + constant2_input + "]}"; 
  std::string named_rotation_input = "{type: ROTATION3, inputs: [" +
      constant1_input + ", " + constant2_input + ", " + ref_name +
      "], name: " + rot_name + "}";
  std::string nested_input = "{name: " + cup_rel_name + ", type: SUBTRACTION, " +
      "inputs: [{type: Z-COORDINATE-OF, inputs: [" + cup_top_name + "]}, " +
      "{type: Z-COORDINATE-OF, inputs: [" + cup_bottom_name + "]}]}";

  // PARSING A REFERENCE
  YAML::Node node = YAML::Load(ref_name);
  ASSERT_NO_THROW(node.as<giskard::ExpressionSpec>());
  giskard::ExpressionSpec exp = node.as<giskard::ExpressionSpec>();
  EXPECT_STREQ(exp.type_.c_str(), "REFERENCE");
  EXPECT_STREQ(exp.name_.c_str(), ref_name.c_str());

  // PARSING A CONSTANT
  node = YAML::Load(constant1_input);
  ASSERT_NO_THROW(node.as<giskard::ExpressionSpec>());
  exp = node.as<giskard::ExpressionSpec>();
  EXPECT_STREQ(exp.type_.c_str(), "CONSTANT");
  EXPECT_DOUBLE_EQ(exp.value_, constant1);

  // PARSING AN ANONYMOUS VECTOR3
  node = YAML::Load(anonymous_vector_input);
  ASSERT_NO_THROW(node.as<giskard::ExpressionSpec>());
  exp = node.as<giskard::ExpressionSpec>();
  EXPECT_STREQ(exp.type_.c_str(), "VECTOR3");
  EXPECT_STREQ(exp.name_.c_str(), "");
  EXPECT_EQ(exp.inputs_.size(), 3);
  EXPECT_STREQ(exp.inputs_[0].type_.c_str(), "CONSTANT");
  EXPECT_DOUBLE_EQ(exp.inputs_[0].value_, constant1);
  EXPECT_STREQ(exp.inputs_[1].type_.c_str(), "REFERENCE");
  EXPECT_STREQ(exp.inputs_[1].name_.c_str(), ref_name.c_str());
  EXPECT_STREQ(exp.inputs_[2].type_.c_str(), "CONSTANT");
  EXPECT_DOUBLE_EQ(exp.inputs_[2].value_, constant2);

  // PARSING A NAMED ROTATION3
  node = YAML::Load(named_rotation_input);
  ASSERT_NO_THROW(node.as<giskard::ExpressionSpec>());
  exp = node.as<giskard::ExpressionSpec>();
  EXPECT_STREQ(exp.type_.c_str(), "ROTATION3");
  EXPECT_STREQ(exp.name_.c_str(), rot_name.c_str());
  EXPECT_EQ(exp.inputs_.size(), 3);
  EXPECT_STREQ(exp.inputs_[0].type_.c_str(), "CONSTANT");
  EXPECT_DOUBLE_EQ(exp.inputs_[0].value_, constant1);
  EXPECT_STREQ(exp.inputs_[1].type_.c_str(), "CONSTANT");
  EXPECT_DOUBLE_EQ(exp.inputs_[1].value_, constant2);
  EXPECT_STREQ(exp.inputs_[2].type_.c_str(), "REFERENCE");
  EXPECT_STREQ(exp.inputs_[2].name_.c_str(), ref_name.c_str());

  // PARSING NESTED EXPRESSIONS
  node = YAML::Load(nested_input);
  ASSERT_NO_THROW(node.as<giskard::ExpressionSpec>());
  exp = node.as<giskard::ExpressionSpec>();
  EXPECT_STREQ(exp.type_.c_str(), "SUBTRACTION");
  EXPECT_STREQ(exp.name_.c_str(), cup_rel_name.c_str());
  ASSERT_EQ(exp.inputs_.size(), 2);
  EXPECT_STREQ(exp.inputs_[0].name_.c_str(), "");
  EXPECT_STREQ(exp.inputs_[0].type_.c_str(), "Z-COORDINATE-OF");
  ASSERT_EQ(exp.inputs_[0].inputs_.size(), 1);
  EXPECT_STREQ(exp.inputs_[0].inputs_[0].name_.c_str(), cup_top_name.c_str());
  EXPECT_STREQ(exp.inputs_[0].inputs_[0].type_.c_str(), "REFERENCE"); 
  EXPECT_STREQ(exp.inputs_[1].name_.c_str(), "");
  EXPECT_STREQ(exp.inputs_[1].type_.c_str(), "Z-COORDINATE-OF");
  ASSERT_EQ(exp.inputs_[1].inputs_.size(), 1);
  EXPECT_STREQ(exp.inputs_[1].inputs_[0].name_.c_str(), cup_bottom_name.c_str());
  EXPECT_STREQ(exp.inputs_[1].inputs_[0].type_.c_str(), "REFERENCE"); 

  // ROUNDTRIP WITH YAML GENERATION
  YAML::Node node2;
  node2 = exp;
  ASSERT_NO_THROW(node2.as<giskard::ExpressionSpec>());
  giskard::ExpressionSpec exp2 = node2.as<giskard::ExpressionSpec>();
  EXPECT_STREQ(exp2.type_.c_str(), "SUBTRACTION");
  EXPECT_STREQ(exp2.name_.c_str(), cup_rel_name.c_str());
  ASSERT_EQ(exp2.inputs_.size(), 2);
  EXPECT_STREQ(exp2.inputs_[0].name_.c_str(), "");
  EXPECT_STREQ(exp2.inputs_[0].type_.c_str(), "Z-COORDINATE-OF");
  ASSERT_EQ(exp2.inputs_[0].inputs_.size(), 1);
  EXPECT_STREQ(exp2.inputs_[0].inputs_[0].name_.c_str(), cup_top_name.c_str());
  EXPECT_STREQ(exp2.inputs_[0].inputs_[0].type_.c_str(), "REFERENCE"); 
  EXPECT_STREQ(exp2.inputs_[1].name_.c_str(), "");
  EXPECT_STREQ(exp2.inputs_[1].type_.c_str(), "Z-COORDINATE-OF");
  ASSERT_EQ(exp2.inputs_[1].inputs_.size(), 1);
  EXPECT_STREQ(exp2.inputs_[1].inputs_[0].name_.c_str(), cup_bottom_name.c_str());
  EXPECT_STREQ(exp2.inputs_[1].inputs_[0].type_.c_str(), "REFERENCE"); 
};
