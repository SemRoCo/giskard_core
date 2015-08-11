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
  giskard::ConstDoubleSpecification descr;
  giskard::Scope scope;

  descr.set_value(1.1);
  EXPECT_DOUBLE_EQ(descr.get_value(), 1.1);

  ASSERT_NO_THROW(descr.get_expression(scope));
  KDL::Expression<double>::Ptr expr = descr.get_expression(scope);
  EXPECT_DOUBLE_EQ(expr->value(), 1.1);
  ASSERT_EQ(expr->number_of_derivatives(), 0);
  EXPECT_DOUBLE_EQ(expr->derivative(0), 0.0); 
}

TEST_F(DoubleExpressionGenerationTest, Inputs)
{
  giskard::InputDoubleSpecification descr;
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

TEST_F(DoubleExpressionGenerationTest, Addition)
{
  giskard::ConstDoubleSpecificationPtr const_descr1(new
      giskard::ConstDoubleSpecification());
  giskard::ConstDoubleSpecificationPtr const_descr2(new
      giskard::ConstDoubleSpecification());
  giskard::InputDoubleSpecificationPtr input_descr(new
      giskard::InputDoubleSpecification());

  giskard::AdditionDoubleSpecification add_descr;
  giskard::Scope scope;

  const_descr1->set_value(-0.7);
  EXPECT_DOUBLE_EQ(const_descr1->get_value(), -0.7);

  const_descr2->set_value(0.3);
  EXPECT_DOUBLE_EQ(const_descr2->get_value(), 0.3);

  input_descr->set_input_num(1);
  EXPECT_EQ(input_descr->get_input_num(), 1);

  std::vector<giskard::DoubleSpecificationPtr> input_descrs;
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
