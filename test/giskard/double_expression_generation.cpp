#include <gtest/gtest.h>
#include <giskard/giskard.hpp>


class DoubleExpressionGenerationTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
    }

    virtual void TearDown(){}
};

TEST_F(DoubleExpressionGenerationTest, Constants)
{
  giskard::DoubleConstSpec descr;
  giskard::Scope scope;

  descr.set_value(1.1);
  EXPECT_DOUBLE_EQ(descr.get_value(), 1.1);

  ASSERT_NO_THROW(descr.get_expression(scope));
  KDL::Expression<double>::Ptr expr = descr.get_expression(scope);
  EXPECT_DOUBLE_EQ(expr->value(), 1.1);
  ASSERT_EQ(expr->number_of_derivatives(), 0);
  EXPECT_DOUBLE_EQ(expr->derivative(0), 0.0); 
}

TEST_F(DoubleExpressionGenerationTest, ConstDoubleEquality)
{
  giskard::DoubleConstSpec d1, d2, d3;

  d1.set_value(1.1);
  d2.set_value(2.0);
  d3.set_value(1.1);

  EXPECT_TRUE(d1.equals(d1));
  EXPECT_FALSE(d1.equals(d2));
  EXPECT_TRUE(d1.equals(d3));

  EXPECT_EQ(d1, d1);
  EXPECT_NE(d1, d2);
  EXPECT_EQ(d1, d3);

  EXPECT_FALSE(d2.equals(d1));
  EXPECT_TRUE(d2.equals(d2));
  EXPECT_FALSE(d2.equals(d3));

  EXPECT_TRUE(d3.equals(d1));
  EXPECT_FALSE(d3.equals(d2));
  EXPECT_TRUE(d3.equals(d3));
}

TEST_F(DoubleExpressionGenerationTest, Inputs)
{
  giskard::DoubleInputSpec descr;
  giskard::Scope scope;

  descr.set_input_num(2);
  EXPECT_EQ(descr.get_input_num(), 2);

  ASSERT_NO_THROW(descr.get_expression(scope));
  KDL::Expression<double>::Ptr expr = descr.get_expression(scope);
  expr->setInputValue(0, 0.1);
  expr->setInputValue(1, 0.2);
  expr->setInputValue(2, 0.3);

  EXPECT_DOUBLE_EQ(expr->value(), 0.3);
  ASSERT_EQ(expr->number_of_derivatives(), 3);
  EXPECT_DOUBLE_EQ(expr->derivative(0), 0.0); 
  EXPECT_DOUBLE_EQ(expr->derivative(1), 0.0); 
  EXPECT_DOUBLE_EQ(expr->derivative(2), 1.0); 
}

TEST_F(DoubleExpressionGenerationTest, InputEquality)
{
  giskard::DoubleInputSpec d1, d2, d3;
  giskard::Scope scope;

  d1.set_input_num(1);
  d2.set_input_num(0);
  d3.set_input_num(1);

  EXPECT_TRUE(d1.equals(d1));
  EXPECT_FALSE(d1.equals(d2));
  EXPECT_TRUE(d1.equals(d3));

  EXPECT_EQ(d1, d1);
  EXPECT_NE(d1, d2);
  EXPECT_EQ(d1, d3);

  EXPECT_FALSE(d2.equals(d1));
  EXPECT_TRUE(d2.equals(d2));
  EXPECT_FALSE(d2.equals(d3));

  EXPECT_TRUE(d3.equals(d1));
  EXPECT_FALSE(d3.equals(d2));
  EXPECT_TRUE(d3.equals(d3));
}

TEST_F(DoubleExpressionGenerationTest, ReferenceEquality)
{
  giskard::DoubleReferenceSpec d1, d2, d3;
  giskard::Scope scope;

  d1.set_reference_name("my_var"); 
  d2.set_reference_name("your_var");
  d3.set_reference_name("my_var");

  EXPECT_TRUE(d1.equals(d1));
  EXPECT_FALSE(d1.equals(d2));
  EXPECT_TRUE(d1.equals(d3));

  EXPECT_EQ(d1, d1);
  EXPECT_NE(d1, d2);
  EXPECT_EQ(d1, d3);

  EXPECT_FALSE(d2.equals(d1));
  EXPECT_TRUE(d2.equals(d2));
  EXPECT_FALSE(d2.equals(d3));

  EXPECT_TRUE(d3.equals(d1));
  EXPECT_FALSE(d3.equals(d2));
  EXPECT_TRUE(d3.equals(d3));
}

TEST_F(DoubleExpressionGenerationTest, Addition)
{
  giskard::DoubleConstSpecPtr const_descr1(new
      giskard::DoubleConstSpec());
  giskard::DoubleConstSpecPtr const_descr2(new
      giskard::DoubleConstSpec());
  giskard::DoubleInputSpecPtr input_descr(new
      giskard::DoubleInputSpec());

  giskard::DoubleAdditionSpec add_descr;
  giskard::Scope scope;

  const_descr1->set_value(-0.7);
  EXPECT_DOUBLE_EQ(const_descr1->get_value(), -0.7);

  const_descr2->set_value(0.3);
  EXPECT_DOUBLE_EQ(const_descr2->get_value(), 0.3);

  input_descr->set_input_num(1);
  EXPECT_EQ(input_descr->get_input_num(), 1);

  std::vector<giskard::DoubleSpecPtr> input_descrs;
  input_descrs.push_back(const_descr1);
  input_descrs.push_back(input_descr);
  input_descrs.push_back(const_descr2);

  add_descr.set_inputs(input_descrs);
  ASSERT_EQ(add_descr.get_inputs().size(), 3);
  ASSERT_EQ(add_descr.get_inputs()[0], const_descr1);
  ASSERT_EQ(add_descr.get_inputs()[1], input_descr);
  ASSERT_EQ(add_descr.get_inputs()[2], const_descr2);

  ASSERT_NO_THROW(add_descr.get_expression(scope));
  KDL::Expression<double>::Ptr expr = add_descr.get_expression(scope);
  ASSERT_EQ(expr->number_of_derivatives(), 2);

  expr->setInputValue(0, 0.0);
  expr->setInputValue(1, 1.5);
  EXPECT_DOUBLE_EQ(expr->value(), 1.1);
  EXPECT_DOUBLE_EQ(expr->derivative(0), 0.0); 
  EXPECT_DOUBLE_EQ(expr->derivative(1), 1.0); 
}

TEST_F(DoubleExpressionGenerationTest, AdditionEquality)
{
  giskard::DoubleConstSpecPtr dd1(new giskard::DoubleConstSpec());
  giskard::DoubleConstSpecPtr dd2(new giskard::DoubleConstSpec());
  giskard::DoubleInputSpecPtr dd3(new giskard::DoubleInputSpec());

  dd1->set_value(1.0);
  dd2->set_value(2.0);
  dd3->set_input_num(3);

  std::vector<giskard::DoubleSpecPtr> in1, in2, in3, in4, in5;
  in1.push_back(dd1);
  in2.push_back(dd2);
  in3.push_back(dd3);
  in4.push_back(dd1);
  in4.push_back(dd3);
  in5.push_back(dd1);

  giskard::DoubleAdditionSpec a1, a2, a3, a4, a5;
  a1.set_inputs(in1);
  a2.set_inputs(in2);
  a3.set_inputs(in3);
  a4.set_inputs(in4);
  a5.set_inputs(in5);

  EXPECT_TRUE(a1.equals(a1));
  EXPECT_FALSE(a1.equals(a2));
  EXPECT_FALSE(a1.equals(a3));
  EXPECT_FALSE(a1.equals(a4));
  EXPECT_TRUE(a1.equals(a5));

  EXPECT_EQ(a1, a1);
  EXPECT_NE(a1, a2);
  EXPECT_NE(a1, a3);
  EXPECT_NE(a1, a4);
  EXPECT_EQ(a1, a5);
}

TEST_F(DoubleExpressionGenerationTest, VectorNorm)
{
  std::string s = "{vector-norm: {vector3: [1.0, 2.0, 2.0]}}";

  YAML::Node node = YAML::Load(s);

  ASSERT_NO_THROW(node.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr spec = node.as<giskard::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 3.0, 1e-10);
}

TEST_F(DoubleExpressionGenerationTest, DoubleMultiplication)
{
  std::string s = "{double-mul: [-0.5, 2.0, 3.5, 0.1]}";

  YAML::Node node = YAML::Load(s);

  ASSERT_NO_THROW(node.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr spec = node.as<giskard::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), -0.35, 1e-10);
}


TEST_F(DoubleExpressionGenerationTest, DoubleSubtraction)
{
  std::string s1 = "{double-sub: [-0.5]}";
  std::string s2 = "{double-sub: [0.5, 2.0]}";
  std::string s3 = "{double-sub: [1.1, 0.2, -0.3]}";

  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr spec = node.as<giskard::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 0.5, 1e-10);

  node = YAML::Load(s2);

  ASSERT_NO_THROW(node.as<giskard::DoubleSpecPtr>());
  spec = node.as<giskard::DoubleSpecPtr>();

  exp = spec->get_expression(giskard::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), -1.5, 1e-10);

  node = YAML::Load(s3);

  ASSERT_NO_THROW(node.as<giskard::DoubleSpecPtr>());
  spec = node.as<giskard::DoubleSpecPtr>();

  exp = spec->get_expression(giskard::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 1.2, 1e-10);
}

TEST_F(DoubleExpressionGenerationTest, XCoordOf)
{
  std::string s1 = "{x-coord: {vector3: [1.1, 2.2, 3.3]}}";

  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr spec = node.as<giskard::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 1.1, 1e-10);
}

TEST_F(DoubleExpressionGenerationTest, YCoordOf)
{
  std::string s1 = "{y-coord: {vector3: [1.1, 2.2, 3.3]}}";

  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr spec = node.as<giskard::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 2.2, 1e-10);
}
TEST_F(DoubleExpressionGenerationTest, ZCoordOf)
{
  std::string s1 = "{z-coord: {vector3: [1.1, 2.2, 3.3]}}";

  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard::DoubleSpecPtr>());
  giskard::DoubleSpecPtr spec = node.as<giskard::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 3.3, 1e-10);
}
