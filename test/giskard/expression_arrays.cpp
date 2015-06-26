#include <gtest/gtest.h>
#include <giskard/giskard.hpp>

using namespace KDL;

class ExpressionArrayTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      exp1 = cached<double>(Constant(1.0) * input(0) + Constant(2.0) * input(1));
      exp2 = cached<double>(Constant(3.0) * input(3) + Constant(4.0) * input(4));
      exp3 = cached<double>(Constant(5.0) * input(1) + Constant(6.0) * input(2) + Constant(7.0) *input(3));
      exps.push_back(exp1);
      exps.push_back(exp2);
      exps.push_back(exp3);

      deriv1 = exp2->derivativeExpression(0);
      deriv2 = exp2->derivativeExpression(1);
      deriv3 = exp2->derivativeExpression(2);
      deriv4 = exp2->derivativeExpression(3);
      deriv5 = exp2->derivativeExpression(4);
      derivs.push_back(deriv1);
      derivs.push_back(deriv2);
      derivs.push_back(deriv3);
      derivs.push_back(deriv4);
      derivs.push_back(deriv5);

      num_exps = 3;
      num_derivs = 5;

      vector_state.insert(vector_state.begin(), num_derivs, 1.0);
      using Eigen::operator<<;
      eigen_state.resize(5,1);
      eigen_state << 1.0, 1.0, 1.0, 1.0, 1.0;
    }

    virtual void TearDown(){}

    Expression<double>::Ptr exp1, exp2, exp3;
    std::vector< Expression<double>::Ptr > exps;

    Expression<double>::Ptr deriv1, deriv2, deriv3, deriv4, deriv5;
    std::vector< Expression<double>::Ptr > derivs;

    size_t num_exps, num_derivs;

    std::vector<double> vector_state;
    Eigen::VectorXd eigen_state;
};

//void print_set(const std::set<int>& s)
//{
//  for (std::set<int>::iterator it=s.begin(); it!=s.end(); ++it)
//    std::cout << ' ' << *it;
//    std::cout << '\n';
//}

TEST_F(ExpressionArrayTest, Constructor)
{
  DoubleExpressionArray a;
  EXPECT_EQ(0, a.num_expressions());
  EXPECT_EQ(0, a.num_inputs());
  EXPECT_EQ(0, a.get_expressions().size());
  EXPECT_EQ(0, a.get_values().rows());
  EXPECT_EQ(1, a.get_values().cols());
  EXPECT_EQ(0, a.get_derivatives().rows());
  EXPECT_EQ(0, a.get_derivatives().cols());
}

TEST_F(ExpressionArrayTest, ListInteraction)
{
  DoubleExpressionArray a;
  a.set_expressions(exps);
  EXPECT_EQ(num_exps, a.num_expressions());
  EXPECT_EQ(num_derivs, a.num_inputs());
  ASSERT_EQ(num_exps, a.get_expressions().size());
  EXPECT_EQ(num_exps, a.get_values().rows());
  EXPECT_EQ(1, a.get_values().cols());
  EXPECT_EQ(num_exps, a.get_derivatives().rows());
  EXPECT_EQ(num_derivs, a.get_derivatives().cols());
  ASSERT_EQ(num_derivs, a.get_derivative_expressions(1).size());
  for(size_t i=0; i<num_exps; ++i)
    EXPECT_EQ(exps[i], a.get_expression(i));
}
