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
};

//TEST_F(YamlParserTest, ParseObservables)
//{
//  // PARSING INPUT-VARIABLE
//  std::string obs_name1 = "trans-x";
//  std::string obs_type1 = "INPUT-VARIABLE";
//  std::string obs_input1 = "{name: " + obs_name1 + ", type: " + obs_type1 + "}";
//  YAML::Node node = YAML::Load(obs_input1);
//
//  ASSERT_NO_THROW(node.as<giskard::ObservableSpec>());
//  giskard::ObservableSpec obs = node.as<giskard::ObservableSpec>();
//  EXPECT_STREQ(obs.name_.c_str(), obs_name1.c_str());
//  EXPECT_STREQ(obs.type_.c_str(), obs_type1.c_str());
//
//  // PARSING INPUT-FRAME
//  std::string obs_name2 = "cup frame";
//  std::string obs_type2 = "INPUT-FRAME";
//  std::string obs_input2 = "{name: " + obs_name2 + ", type: " + obs_type2 + "}";
//  node = YAML::Load(obs_input2);
//
//  ASSERT_NO_THROW(node.as<giskard::ObservableSpec>());
//  obs = node.as<giskard::ObservableSpec>();
//  EXPECT_STREQ(obs.name_.c_str(), obs_name2.c_str());
//  EXPECT_STREQ(obs.type_.c_str(), obs_type2.c_str());
//
//  // PARSING LIST OF INPUTS
//  std::string obs_list_input = "[" + obs_input1 + ", " + obs_input2 + "]";
//  node = YAML::Load(obs_list_input);
//  std::vector<giskard::ObservableSpec> obs_list = 
//      node.as< std::vector<giskard::ObservableSpec> >();
//  ASSERT_EQ(obs_list.size(), 2);
//  EXPECT_STREQ(obs_list[0].name_.c_str(), obs_name1.c_str()); 
//  EXPECT_STREQ(obs_list[1].name_.c_str(), obs_name2.c_str()); 
//  EXPECT_STREQ(obs_list[0].type_.c_str(), obs_type1.c_str()); 
//  EXPECT_STREQ(obs_list[1].type_.c_str(), obs_type2.c_str()); 
//
//  // ROUNDTRIP WITH YAML GENERATION
//  YAML::Node node2;
//  node2 = obs_list;
//  std::vector<giskard::ObservableSpec> obs_list2 = 
//      node.as< std::vector<giskard::ObservableSpec> >();
//  ASSERT_EQ(obs_list2.size(), 2);
//  EXPECT_STREQ(obs_list2[0].name_.c_str(), obs_name1.c_str()); 
//  EXPECT_STREQ(obs_list2[1].name_.c_str(), obs_name2.c_str()); 
//  EXPECT_STREQ(obs_list2[0].type_.c_str(), obs_type1.c_str()); 
//  EXPECT_STREQ(obs_list2[1].type_.c_str(), obs_type2.c_str()); 
//};
//
//TEST_F(YamlParserTest, ParseControllables)
//{
//  std::string c1_ref = "cup frame translation-z";
//  std::string c1_name = c1_ref + " velocity";
//  std::string c2_ref = "cup frame rotation-x";
//  std::string c2_name = c2_ref + " velocity";
// 
//  std::string c1_input = "{name: " + c1_name + ", type: VELOCITY-OF, " +
//      "reference: " + c1_ref + "}";
//  std::string c2_input = "{name: " + c2_name + ", type: VELOCITY-OF, " +
//      "reference: " + c2_ref + "}";
//  std::string c_list_input = "[" + c1_input + ", " + c2_input + "]";
//
//  // PARSING A CONTROLLABLE SPEC
//  YAML::Node node = YAML::Load(c1_input);
//  ASSERT_NO_THROW(node.as<giskard::ControllableSpec>());
//  giskard::ControllableSpec c1 = node.as<giskard::ControllableSpec>();
//  EXPECT_STREQ(c1.name_.c_str(), c1_name.c_str());
//  EXPECT_STREQ(c1.type_.c_str(), "VELOCITY-OF");
//  EXPECT_STREQ(c1.reference_.c_str(), c1_ref.c_str());
//
//  // PARSING A LIST OF CONTROLLABLE SPECS
//  node = YAML::Load(c_list_input);
//  ASSERT_NO_THROW(node.as< std::vector<giskard::ControllableSpec> >());
//  std::vector<giskard::ControllableSpec> c_list = 
//      node.as< std::vector<giskard::ControllableSpec> >();
//  ASSERT_EQ(c_list.size(), 2);
//  EXPECT_STREQ(c_list[0].name_.c_str(), c1_name.c_str()); 
//  EXPECT_STREQ(c_list[1].name_.c_str(), c2_name.c_str()); 
//  EXPECT_STREQ(c_list[0].type_.c_str(), "VELOCITY-OF");
//  EXPECT_STREQ(c_list[1].type_.c_str(), "VELOCITY-OF");
//  EXPECT_STREQ(c_list[0].reference_.c_str(), c1_ref.c_str()); 
//  EXPECT_STREQ(c_list[1].reference_.c_str(), c2_ref.c_str()); 
//
//  // ROUNDTRIP WITH YAML GENERATION
//  YAML::Node node2;
//  node2 = c_list;
//  ASSERT_NO_THROW(node2.as< std::vector<giskard::ControllableSpec> >());
//  std::vector<giskard::ControllableSpec> c_list2 = 
//      node.as< std::vector<giskard::ControllableSpec> >();
//  ASSERT_EQ(c_list2.size(), 2);
//  EXPECT_STREQ(c_list2[0].name_.c_str(), c1_name.c_str()); 
//  EXPECT_STREQ(c_list2[1].name_.c_str(), c2_name.c_str()); 
//  EXPECT_STREQ(c_list2[0].type_.c_str(), "VELOCITY-OF");
//  EXPECT_STREQ(c_list2[1].type_.c_str(), "VELOCITY-OF");
//  EXPECT_STREQ(c_list2[0].reference_.c_str(), c1_ref.c_str()); 
//  EXPECT_STREQ(c_list2[1].reference_.c_str(), c2_ref.c_str()); 
//};
//
//TEST_F(YamlParserTest, ParseExpressions)
//{
//  std::string ref_name = "cup frame rotation-z";
//  std::string rot_name = "cup rotation";
//  std::string cup_top_name = "cup top centroid";
//  std::string cup_bottom_name = "cup bottom centroid";
//  std::string cup_rel_name = "cup upright";
//  double constant1 = 2.3;
//  double constant2 = -1.1;
//  std::string constant1_input = boost::lexical_cast<std::string>(constant1);
//  std::string constant2_input = boost::lexical_cast<std::string>(constant2);
//  std::string anonymous_vector_input = "{type: VECTOR3, inputs: [" +
//      constant1_input + ", " + ref_name + ", " + constant2_input + "]}"; 
//  std::string named_rotation_input = "{type: ROTATION3, inputs: [" +
//      constant1_input + ", " + constant2_input + ", " + ref_name +
//      "], name: " + rot_name + "}";
//  std::string nested_input = "{name: " + cup_rel_name + ", type: SUBTRACTION, " +
//      "inputs: [{type: Z-COORDINATE-OF, inputs: [" + cup_top_name + "]}, " +
//      "{type: Z-COORDINATE-OF, inputs: [" + cup_bottom_name + "]}]}";
//
//  // PARSING A REFERENCE
//  YAML::Node node = YAML::Load(ref_name);
//  ASSERT_NO_THROW(node.as<giskard::ExpressionSpec>());
//  giskard::ExpressionSpec exp = node.as<giskard::ExpressionSpec>();
//  EXPECT_STREQ(exp.type_.c_str(), "REFERENCE");
//  EXPECT_STREQ(exp.name_.c_str(), ref_name.c_str());
//
//  // PARSING A CONSTANT
//  node = YAML::Load(constant1_input);
//  ASSERT_NO_THROW(node.as<giskard::ExpressionSpec>());
//  exp = node.as<giskard::ExpressionSpec>();
//  EXPECT_STREQ(exp.type_.c_str(), "CONSTANT");
//  EXPECT_DOUBLE_EQ(exp.value_, constant1);
//
//  // PARSING AN ANONYMOUS VECTOR3
//  node = YAML::Load(anonymous_vector_input);
//  ASSERT_NO_THROW(node.as<giskard::ExpressionSpec>());
//  exp = node.as<giskard::ExpressionSpec>();
//  EXPECT_STREQ(exp.type_.c_str(), "VECTOR3");
//  EXPECT_STREQ(exp.name_.c_str(), "");
//  EXPECT_EQ(exp.inputs_.size(), 3);
//  EXPECT_STREQ(exp.inputs_[0].type_.c_str(), "CONSTANT");
//  EXPECT_DOUBLE_EQ(exp.inputs_[0].value_, constant1);
//  EXPECT_STREQ(exp.inputs_[1].type_.c_str(), "REFERENCE");
//  EXPECT_STREQ(exp.inputs_[1].name_.c_str(), ref_name.c_str());
//  EXPECT_STREQ(exp.inputs_[2].type_.c_str(), "CONSTANT");
//  EXPECT_DOUBLE_EQ(exp.inputs_[2].value_, constant2);
//
//  // PARSING A NAMED ROTATION3
//  node = YAML::Load(named_rotation_input);
//  ASSERT_NO_THROW(node.as<giskard::ExpressionSpec>());
//  exp = node.as<giskard::ExpressionSpec>();
//  EXPECT_STREQ(exp.type_.c_str(), "ROTATION3");
//  EXPECT_STREQ(exp.name_.c_str(), rot_name.c_str());
//  EXPECT_EQ(exp.inputs_.size(), 3);
//  EXPECT_STREQ(exp.inputs_[0].type_.c_str(), "CONSTANT");
//  EXPECT_DOUBLE_EQ(exp.inputs_[0].value_, constant1);
//  EXPECT_STREQ(exp.inputs_[1].type_.c_str(), "CONSTANT");
//  EXPECT_DOUBLE_EQ(exp.inputs_[1].value_, constant2);
//  EXPECT_STREQ(exp.inputs_[2].type_.c_str(), "REFERENCE");
//  EXPECT_STREQ(exp.inputs_[2].name_.c_str(), ref_name.c_str());
//
//  // PARSING NESTED EXPRESSIONS
//  node = YAML::Load(nested_input);
//  ASSERT_NO_THROW(node.as<giskard::ExpressionSpec>());
//  exp = node.as<giskard::ExpressionSpec>();
//  EXPECT_STREQ(exp.type_.c_str(), "SUBTRACTION");
//  EXPECT_STREQ(exp.name_.c_str(), cup_rel_name.c_str());
//  ASSERT_EQ(exp.inputs_.size(), 2);
//  EXPECT_STREQ(exp.inputs_[0].name_.c_str(), "");
//  EXPECT_STREQ(exp.inputs_[0].type_.c_str(), "Z-COORDINATE-OF");
//  ASSERT_EQ(exp.inputs_[0].inputs_.size(), 1);
//  EXPECT_STREQ(exp.inputs_[0].inputs_[0].name_.c_str(), cup_top_name.c_str());
//  EXPECT_STREQ(exp.inputs_[0].inputs_[0].type_.c_str(), "REFERENCE"); 
//  EXPECT_STREQ(exp.inputs_[1].name_.c_str(), "");
//  EXPECT_STREQ(exp.inputs_[1].type_.c_str(), "Z-COORDINATE-OF");
//  ASSERT_EQ(exp.inputs_[1].inputs_.size(), 1);
//  EXPECT_STREQ(exp.inputs_[1].inputs_[0].name_.c_str(), cup_bottom_name.c_str());
//  EXPECT_STREQ(exp.inputs_[1].inputs_[0].type_.c_str(), "REFERENCE"); 
//
//  // ROUNDTRIP WITH YAML GENERATION
//  YAML::Node node2;
//  node2 = exp;
//  ASSERT_NO_THROW(node2.as<giskard::ExpressionSpec>());
//  giskard::ExpressionSpec exp2 = node2.as<giskard::ExpressionSpec>();
//  EXPECT_STREQ(exp2.type_.c_str(), "SUBTRACTION");
//  EXPECT_STREQ(exp2.name_.c_str(), cup_rel_name.c_str());
//  ASSERT_EQ(exp2.inputs_.size(), 2);
//  EXPECT_STREQ(exp2.inputs_[0].name_.c_str(), "");
//  EXPECT_STREQ(exp2.inputs_[0].type_.c_str(), "Z-COORDINATE-OF");
//  ASSERT_EQ(exp2.inputs_[0].inputs_.size(), 1);
//  EXPECT_STREQ(exp2.inputs_[0].inputs_[0].name_.c_str(), cup_top_name.c_str());
//  EXPECT_STREQ(exp2.inputs_[0].inputs_[0].type_.c_str(), "REFERENCE"); 
//  EXPECT_STREQ(exp2.inputs_[1].name_.c_str(), "");
//  EXPECT_STREQ(exp2.inputs_[1].type_.c_str(), "Z-COORDINATE-OF");
//  ASSERT_EQ(exp2.inputs_[1].inputs_.size(), 1);
//  EXPECT_STREQ(exp2.inputs_[1].inputs_[0].name_.c_str(), cup_bottom_name.c_str());
//  EXPECT_STREQ(exp2.inputs_[1].inputs_[0].type_.c_str(), "REFERENCE"); 
//};
//
//TEST_F(YamlParserTest, ParseConstraints)
//{
//  std::string sc_exp = "cup upright";
//  std::string sc_name = sc_exp + " constraint";
//  std::string sc_type = "SOFT-CONSTRAINT";
//  double sc_lower = 0.04;
//  double sc_upper = 0.06;
//  double sc_weight = 1.0;
//  double sc_gain = 10.0;
//  std::string sc_input = "{name: " + sc_name + ", type: " + sc_type +
//      ", expression: " + sc_exp + ", lower: " + boost::lexical_cast<std::string>(sc_lower) +
//      ", upper: " + boost::lexical_cast<std::string>(sc_upper) +
//      ", gain: " + boost::lexical_cast<std::string>(sc_gain) +
//      ", weight: " + boost::lexical_cast<std::string>(sc_weight) + "}";
//
//  std::string hc_exp = "cup frame translation-x velocity";
//  std::string hc_name = hc_exp + " limits";
//  std::string hc_type = "HARD-CONSTRAINT";
//  double hc_lower = -0.2;
//  double hc_upper = 0.2;
//  double hc_weight = 1.0;
//  std::string hc_input = "{name: " + hc_name + ", type: " + hc_type +
//      ", expression: " + hc_exp + ", lower: " + boost::lexical_cast<std::string>(hc_lower) +
//      ", upper: " + boost::lexical_cast<std::string>(hc_upper) +
//      ", weight: " + boost::lexical_cast<std::string>(hc_weight) + "}";
//
//  std::string input_list = "[" + hc_input + ", " + sc_input + "]";
//
//  // PARSING A SOFT CONSTRAINT
//  YAML::Node node = YAML::Load(sc_input);
//  ASSERT_NO_THROW(node.as<giskard::ConstraintSpec>());
//  giskard::ConstraintSpec c = node.as<giskard::ConstraintSpec>();
//  EXPECT_STREQ(c.name_.c_str(), sc_name.c_str());
//  EXPECT_STREQ(c.type_.c_str(), sc_type.c_str());
//  EXPECT_STREQ(c.expression_.c_str(), sc_exp.c_str());
//  EXPECT_DOUBLE_EQ(c.lower_, sc_lower);
//  EXPECT_DOUBLE_EQ(c.upper_, sc_upper);
//  EXPECT_DOUBLE_EQ(c.weight_, sc_weight);
//  EXPECT_DOUBLE_EQ(c.gain_, sc_gain);
//
//  // PARSING A HARD CONSTRAINT
//  node = YAML::Load(hc_input);
//  ASSERT_NO_THROW(node.as<giskard::ConstraintSpec>());
//  c = node.as<giskard::ConstraintSpec>();
//  EXPECT_STREQ(c.name_.c_str(), hc_name.c_str());
//  EXPECT_STREQ(c.type_.c_str(), hc_type.c_str());
//  EXPECT_STREQ(c.expression_.c_str(), hc_exp.c_str());
//  EXPECT_DOUBLE_EQ(c.lower_, hc_lower);
//  EXPECT_DOUBLE_EQ(c.upper_, hc_upper);
//  EXPECT_DOUBLE_EQ(c.weight_, hc_weight);
//  EXPECT_DOUBLE_EQ(c.gain_, 0.0);
//
//  // PARSING A LIST OF CONSTRAINTS
//  node = YAML::Load(input_list);
//  ASSERT_NO_THROW(node.as< std::vector<giskard::ConstraintSpec> >());
//  std::vector< giskard::ConstraintSpec > cs = node.as< std::vector<giskard::ConstraintSpec> >();
//  ASSERT_EQ(cs.size(), 2);
//  // hard constraint went into list, first
//  EXPECT_STREQ(cs[0].name_.c_str(), hc_name.c_str());
//  EXPECT_STREQ(cs[0].type_.c_str(), hc_type.c_str());
//  EXPECT_STREQ(cs[0].expression_.c_str(), hc_exp.c_str());
//  EXPECT_DOUBLE_EQ(cs[0].lower_, hc_lower);
//  EXPECT_DOUBLE_EQ(cs[0].upper_, hc_upper);
//  EXPECT_DOUBLE_EQ(cs[0].weight_, hc_weight);
//  EXPECT_DOUBLE_EQ(cs[0].gain_, 0.0);
//  // followed by soft constraint
//  EXPECT_STREQ(cs[1].name_.c_str(), sc_name.c_str());
//  EXPECT_STREQ(cs[1].type_.c_str(), sc_type.c_str());
//  EXPECT_STREQ(cs[1].expression_.c_str(), sc_exp.c_str());
//  EXPECT_DOUBLE_EQ(cs[1].lower_, sc_lower);
//  EXPECT_DOUBLE_EQ(cs[1].upper_, sc_upper);
//  EXPECT_DOUBLE_EQ(cs[1].weight_, sc_weight);
//  EXPECT_DOUBLE_EQ(cs[1].gain_, sc_gain);
//
//  // ROUNDTRIP WITH PRODUCTION OF YAML
//  YAML::Node node2;
//  node2 = cs;
//  ASSERT_NO_THROW(node2.as< std::vector<giskard::ConstraintSpec> >());
//  std::vector< giskard::ConstraintSpec> cs2 = node2.as< std::vector<giskard::ConstraintSpec> >();
//  ASSERT_EQ(cs2.size(), 2);
//  // hard constraint went into list, first
//  EXPECT_STREQ(cs2[0].name_.c_str(), hc_name.c_str());
//  EXPECT_STREQ(cs2[0].type_.c_str(), hc_type.c_str());
//  EXPECT_STREQ(cs2[0].expression_.c_str(), hc_exp.c_str());
//  EXPECT_DOUBLE_EQ(cs2[0].lower_, hc_lower);
//  EXPECT_DOUBLE_EQ(cs2[0].upper_, hc_upper);
//  EXPECT_DOUBLE_EQ(cs2[0].weight_, hc_weight);
//  EXPECT_DOUBLE_EQ(cs2[0].gain_, 0.0);
//  // followed by soft constraint
//  EXPECT_STREQ(cs2[1].name_.c_str(), sc_name.c_str());
//  EXPECT_STREQ(cs2[1].type_.c_str(), sc_type.c_str());
//  EXPECT_STREQ(cs2[1].expression_.c_str(), sc_exp.c_str());
//  EXPECT_DOUBLE_EQ(cs2[1].lower_, sc_lower);
//  EXPECT_DOUBLE_EQ(cs2[1].upper_, sc_upper);
//  EXPECT_DOUBLE_EQ(cs2[1].weight_, sc_weight);
//  EXPECT_DOUBLE_EQ(cs2[1].gain_, sc_gain);
//};
//
//TEST_F(YamlParserTest, ParseControllerSpec)
//{
//  // NOTE: THE SPECIFICATIONS BELOW ARE ONLY SYNTACTICALLY CORRECT
//  // DEFINITION OF OBSERVABLES...
//  std::string obs_name1 = "trans-x";
//  std::string obs_type1 = "INPUT-VARIABLE";
//  std::string obs_input1 = "{name: " + obs_name1 + ", type: " + obs_type1 + "}";
//  std::string obs_name2 = "cup frame";
//  std::string obs_type2 = "INPUT-FRAME";
//  std::string obs_input2 = "{name: " + obs_name2 + ", type: " + obs_type2 + "}";
//  std::string obs_list_input = "[" + obs_input1 + ", " + obs_input2 + "]";
//
//  // DEFINITION OF EXPRESSIONS
//  std::string ref_name = "cup frame rotation-z";
//  std::string rot_name = "cup rotation";
//  std::string cup_top_name = "cup top centroid";
//  std::string cup_bottom_name = "cup bottom centroid";
//  std::string cup_rel_name = "cup upright";
//  double constant1 = 2.3;
//  double constant2 = -1.1;
//  std::string constant1_input = boost::lexical_cast<std::string>(constant1);
//  std::string constant2_input = boost::lexical_cast<std::string>(constant2);
//  std::string named_rotation_input = "{type: ROTATION3, inputs: [" +
//      constant1_input + ", " + constant2_input + ", " + ref_name +
//      "], name: " + rot_name + "}";
//  std::string nested_input = "{name: " + cup_rel_name + ", type: SUBTRACTION, " +
//      "inputs: [{type: Z-COORDINATE-OF, inputs: [" + cup_top_name + "]}, " +
//      "{type: Z-COORDINATE-OF, inputs: [" + cup_bottom_name + "]}]}";
//  std::string exp_list_input = "[" + named_rotation_input + ", " + nested_input + "]";
//
//  // DEFINITION OF CONTROLLABLES
//  std::string c1_ref = "cup frame translation-z";
//  std::string c1_name = c1_ref + " velocity";
//  std::string c2_ref = "cup frame rotation-x";
//  std::string c2_name = c2_ref + " velocity";
// 
//  std::string c1_input = "{name: " + c1_name + ", type: VELOCITY-OF, " +
//      "reference: " + c1_ref + "}";
//  std::string c2_input = "{name: " + c2_name + ", type: VELOCITY-OF, " +
//      "reference: " + c2_ref + "}";
//  std::string c_list_input = "[" + c1_input + ", " + c2_input + "]";
//
//  // DEFINITION OF CONSTRAINTS
//  std::string sc_exp = "cup upright";
//  std::string sc_name = sc_exp + " constraint";
//  std::string sc_type = "SOFT-CONSTRAINT";
//  double sc_lower = 0.04;
//  double sc_upper = 0.06;
//  double sc_weight = 1.0;
//  double sc_gain = 10.0;
//  std::string sc_input = "{name: " + sc_name + ", type: " + sc_type +
//      ", expression: " + sc_exp + ", lower: " + boost::lexical_cast<std::string>(sc_lower) +
//      ", upper: " + boost::lexical_cast<std::string>(sc_upper) +
//      ", gain: " + boost::lexical_cast<std::string>(sc_gain) +
//      ", weight: " + boost::lexical_cast<std::string>(sc_weight) + "}";
//
//  std::string hc_exp = "cup frame translation-x velocity";
//  std::string hc_name = hc_exp + " limits";
//  std::string hc_type = "HARD-CONSTRAINT";
//  double hc_lower = -0.2;
//  double hc_upper = 0.2;
//  double hc_weight = 1.0;
//  std::string hc_input = "{name: " + hc_name + ", type: " + hc_type +
//      ", expression: " + hc_exp + ", lower: " + boost::lexical_cast<std::string>(hc_lower) +
//      ", upper: " + boost::lexical_cast<std::string>(hc_upper) +
//      ", weight: " + boost::lexical_cast<std::string>(hc_weight) + "}";
//
//  std::string cs_input_list = "[" + hc_input + ", " + sc_input + "]";
//
//  // DEFINITION OF CONTROLLER SPEC
//  std::string controller_input = "{observables: " + obs_list_input +
//      ", expressions: " + exp_list_input + ", controllables: " + c_list_input +
//      ", constraints: " + cs_input_list + "}";
//
//  // PARSING A SINGLE CONTROLLER SPEC
//  YAML::Node node = YAML::Load(controller_input);
//  ASSERT_NO_THROW(node.as<giskard::ControllerSpec>());
//  giskard::ControllerSpec c = node.as<giskard::ControllerSpec>();
//  // checking observales...
//  ASSERT_EQ(c.observables_.size(), 2);
//  EXPECT_STREQ(c.observables_[0].name_.c_str(), obs_name1.c_str()); 
//  EXPECT_STREQ(c.observables_[1].name_.c_str(), obs_name2.c_str()); 
//  EXPECT_STREQ(c.observables_[0].type_.c_str(), obs_type1.c_str()); 
//  EXPECT_STREQ(c.observables_[1].type_.c_str(), obs_type2.c_str()); 
//  // checking expressions...
//  ASSERT_EQ(c.expressions_.size(), 2);
//  EXPECT_STREQ(c.expressions_[0].type_.c_str(), "ROTATION3");
//  EXPECT_STREQ(c.expressions_[0].name_.c_str(), rot_name.c_str());
//  EXPECT_EQ(c.expressions_[0].inputs_.size(), 3);
//  EXPECT_STREQ(c.expressions_[0].inputs_[0].type_.c_str(), "CONSTANT");
//  EXPECT_DOUBLE_EQ(c.expressions_[0].inputs_[0].value_, constant1);
//  EXPECT_STREQ(c.expressions_[0].inputs_[1].type_.c_str(), "CONSTANT");
//  EXPECT_DOUBLE_EQ(c.expressions_[0].inputs_[1].value_, constant2);
//  EXPECT_STREQ(c.expressions_[0].inputs_[2].type_.c_str(), "REFERENCE");
//  EXPECT_STREQ(c.expressions_[0].inputs_[2].name_.c_str(), ref_name.c_str());
//  EXPECT_STREQ(c.expressions_[1].type_.c_str(), "SUBTRACTION");
//  EXPECT_STREQ(c.expressions_[1].name_.c_str(), cup_rel_name.c_str());
//  ASSERT_EQ(c.expressions_[1].inputs_.size(), 2);
//  EXPECT_STREQ(c.expressions_[1].inputs_[0].name_.c_str(), "");
//  EXPECT_STREQ(c.expressions_[1].inputs_[0].type_.c_str(), "Z-COORDINATE-OF");
//  ASSERT_EQ(c.expressions_[1].inputs_[0].inputs_.size(), 1);
//  EXPECT_STREQ(c.expressions_[1].inputs_[0].inputs_[0].name_.c_str(), cup_top_name.c_str());
//  EXPECT_STREQ(c.expressions_[1].inputs_[0].inputs_[0].type_.c_str(), "REFERENCE"); 
//  EXPECT_STREQ(c.expressions_[1].inputs_[1].name_.c_str(), "");
//  EXPECT_STREQ(c.expressions_[1].inputs_[1].type_.c_str(), "Z-COORDINATE-OF");
//  ASSERT_EQ(c.expressions_[1].inputs_[1].inputs_.size(), 1);
//  EXPECT_STREQ(c.expressions_[1].inputs_[1].inputs_[0].name_.c_str(), cup_bottom_name.c_str());
//  EXPECT_STREQ(c.expressions_[1].inputs_[1].inputs_[0].type_.c_str(), "REFERENCE"); 
//  // checking controllables...
//  ASSERT_EQ(c.controllables_.size(), 2);
//  EXPECT_STREQ(c.controllables_[0].name_.c_str(), c1_name.c_str()); 
//  EXPECT_STREQ(c.controllables_[1].name_.c_str(), c2_name.c_str()); 
//  EXPECT_STREQ(c.controllables_[0].type_.c_str(), "VELOCITY-OF");
//  EXPECT_STREQ(c.controllables_[1].type_.c_str(), "VELOCITY-OF");
//  EXPECT_STREQ(c.controllables_[0].reference_.c_str(), c1_ref.c_str()); 
//  EXPECT_STREQ(c.controllables_[1].reference_.c_str(), c2_ref.c_str()); 
//  // checking constraints...
//  ASSERT_EQ(c.constraints_.size(), 2);
//  EXPECT_STREQ(c.constraints_[0].name_.c_str(), hc_name.c_str());
//  EXPECT_STREQ(c.constraints_[0].type_.c_str(), hc_type.c_str());
//  EXPECT_STREQ(c.constraints_[0].expression_.c_str(), hc_exp.c_str());
//  EXPECT_DOUBLE_EQ(c.constraints_[0].lower_, hc_lower);
//  EXPECT_DOUBLE_EQ(c.constraints_[0].upper_, hc_upper);
//  EXPECT_DOUBLE_EQ(c.constraints_[0].weight_, hc_weight);
//  EXPECT_DOUBLE_EQ(c.constraints_[0].gain_, 0.0);
//  EXPECT_STREQ(c.constraints_[1].name_.c_str(), sc_name.c_str());
//  EXPECT_STREQ(c.constraints_[1].type_.c_str(), sc_type.c_str());
//  EXPECT_STREQ(c.constraints_[1].expression_.c_str(), sc_exp.c_str());
//  EXPECT_DOUBLE_EQ(c.constraints_[1].lower_, sc_lower);
//  EXPECT_DOUBLE_EQ(c.constraints_[1].upper_, sc_upper);
//  EXPECT_DOUBLE_EQ(c.constraints_[1].weight_, sc_weight);
//  EXPECT_DOUBLE_EQ(c.constraints_[1].gain_, sc_gain);
// 
//  // ROUNDTRIP WITH YAML GENERATION AND PARSING
//  YAML::Node node2;
//  node2 = c;
//  ASSERT_NO_THROW(node.as<giskard::ControllerSpec>());
//  giskard::ControllerSpec c2 = node.as<giskard::ControllerSpec>();
//  // checking observales...
//  ASSERT_EQ(c2.observables_.size(), 2);
//  EXPECT_STREQ(c2.observables_[0].name_.c_str(), obs_name1.c_str()); 
//  EXPECT_STREQ(c2.observables_[1].name_.c_str(), obs_name2.c_str()); 
//  EXPECT_STREQ(c2.observables_[0].type_.c_str(), obs_type1.c_str()); 
//  EXPECT_STREQ(c2.observables_[1].type_.c_str(), obs_type2.c_str()); 
//  // checking expressions...
//  ASSERT_EQ(c2.expressions_.size(), 2);
//  EXPECT_STREQ(c2.expressions_[0].type_.c_str(), "ROTATION3");
//  EXPECT_STREQ(c2.expressions_[0].name_.c_str(), rot_name.c_str());
//  EXPECT_EQ(c2.expressions_[0].inputs_.size(), 3);
//  EXPECT_STREQ(c2.expressions_[0].inputs_[0].type_.c_str(), "CONSTANT");
//  EXPECT_DOUBLE_EQ(c2.expressions_[0].inputs_[0].value_, constant1);
//  EXPECT_STREQ(c2.expressions_[0].inputs_[1].type_.c_str(), "CONSTANT");
//  EXPECT_DOUBLE_EQ(c2.expressions_[0].inputs_[1].value_, constant2);
//  EXPECT_STREQ(c2.expressions_[0].inputs_[2].type_.c_str(), "REFERENCE");
//  EXPECT_STREQ(c2.expressions_[0].inputs_[2].name_.c_str(), ref_name.c_str());
//  EXPECT_STREQ(c2.expressions_[1].type_.c_str(), "SUBTRACTION");
//  EXPECT_STREQ(c2.expressions_[1].name_.c_str(), cup_rel_name.c_str());
//  ASSERT_EQ(c2.expressions_[1].inputs_.size(), 2);
//  EXPECT_STREQ(c2.expressions_[1].inputs_[0].name_.c_str(), "");
//  EXPECT_STREQ(c2.expressions_[1].inputs_[0].type_.c_str(), "Z-COORDINATE-OF");
//  ASSERT_EQ(c2.expressions_[1].inputs_[0].inputs_.size(), 1);
//  EXPECT_STREQ(c2.expressions_[1].inputs_[0].inputs_[0].name_.c_str(), cup_top_name.c_str());
//  EXPECT_STREQ(c2.expressions_[1].inputs_[0].inputs_[0].type_.c_str(), "REFERENCE"); 
//  EXPECT_STREQ(c2.expressions_[1].inputs_[1].name_.c_str(), "");
//  EXPECT_STREQ(c2.expressions_[1].inputs_[1].type_.c_str(), "Z-COORDINATE-OF");
//  ASSERT_EQ(c2.expressions_[1].inputs_[1].inputs_.size(), 1);
//  EXPECT_STREQ(c2.expressions_[1].inputs_[1].inputs_[0].name_.c_str(), cup_bottom_name.c_str());
//  EXPECT_STREQ(c2.expressions_[1].inputs_[1].inputs_[0].type_.c_str(), "REFERENCE"); 
//  // checking controllables...
//  ASSERT_EQ(c2.controllables_.size(), 2);
//  EXPECT_STREQ(c2.controllables_[0].name_.c_str(), c1_name.c_str()); 
//  EXPECT_STREQ(c2.controllables_[1].name_.c_str(), c2_name.c_str()); 
//  EXPECT_STREQ(c2.controllables_[0].type_.c_str(), "VELOCITY-OF");
//  EXPECT_STREQ(c2.controllables_[1].type_.c_str(), "VELOCITY-OF");
//  EXPECT_STREQ(c2.controllables_[0].reference_.c_str(), c1_ref.c_str()); 
//  EXPECT_STREQ(c2.controllables_[1].reference_.c_str(), c2_ref.c_str()); 
//  // checking constraints...
//  ASSERT_EQ(c2.constraints_.size(), 2);
//  EXPECT_STREQ(c2.constraints_[0].name_.c_str(), hc_name.c_str());
//  EXPECT_STREQ(c2.constraints_[0].type_.c_str(), hc_type.c_str());
//  EXPECT_STREQ(c2.constraints_[0].expression_.c_str(), hc_exp.c_str());
//  EXPECT_DOUBLE_EQ(c2.constraints_[0].lower_, hc_lower);
//  EXPECT_DOUBLE_EQ(c2.constraints_[0].upper_, hc_upper);
//  EXPECT_DOUBLE_EQ(c2.constraints_[0].weight_, hc_weight);
//  EXPECT_DOUBLE_EQ(c2.constraints_[0].gain_, 0.0);
//  EXPECT_STREQ(c2.constraints_[1].name_.c_str(), sc_name.c_str());
//  EXPECT_STREQ(c2.constraints_[1].type_.c_str(), sc_type.c_str());
//  EXPECT_STREQ(c2.constraints_[1].expression_.c_str(), sc_exp.c_str());
//  EXPECT_DOUBLE_EQ(c2.constraints_[1].lower_, sc_lower);
//  EXPECT_DOUBLE_EQ(c2.constraints_[1].upper_, sc_upper);
//  EXPECT_DOUBLE_EQ(c2.constraints_[1].weight_, sc_weight);
//  EXPECT_DOUBLE_EQ(c2.constraints_[1].gain_, sc_gain);
//};
