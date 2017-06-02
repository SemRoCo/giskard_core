/*
 * Copyright (C) 2015-2017 Georg Bartels <georg.bartels@cs.uni-bremen.de>
 * 
 * This file is part of giskard.
 * 
 * giskard is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <gtest/gtest.h>
#include <giskard_core/giskard_core.hpp>

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
      eigen_state.resize(5,1.0);
      eigen_state << 2.0, 2.0, 2.0, 2.0, 2.0;
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
  EXPECT_TRUE(a.has_expression(exp1));
  EXPECT_TRUE(a.has_expression(exp2));
  EXPECT_TRUE(a.has_expression(exp3));
  EXPECT_EQ(0, a.find_expression(exp1));
  EXPECT_EQ(1, a.find_expression(exp2));
  EXPECT_EQ(2, a.find_expression(exp3));
}

TEST_F(ExpressionArrayTest, PushPop)
{
  DoubleExpressionArray a;
  a.push_expression(exp1);
  EXPECT_EQ(1, a.num_expressions());
  EXPECT_EQ(2, a.num_inputs());
  EXPECT_EQ(exp1, a.get_expression(0));

  EXPECT_EQ(exp1, a.pop_expression());
  EXPECT_EQ(0, a.num_expressions());
  EXPECT_EQ(0, a.num_inputs());

  a.push_expression(exp1);
  EXPECT_EQ(1, a.num_expressions());
  EXPECT_EQ(2, a.num_inputs());
  EXPECT_EQ(exp1, a.get_expression(0));

  a.push_expression(exp2);
  EXPECT_EQ(2, a.num_expressions());
  EXPECT_EQ(5, a.num_inputs());
  EXPECT_EQ(exp1, a.get_expression(0));
  EXPECT_EQ(exp2, a.get_expression(1));

  a.push_expression(exp3);
  EXPECT_EQ(3, a.num_expressions());
  EXPECT_EQ(5, a.num_inputs());
  EXPECT_EQ(exp1, a.get_expression(0));
  EXPECT_EQ(exp2, a.get_expression(1));
  EXPECT_EQ(exp3, a.get_expression(2));

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
  EXPECT_TRUE(a.has_expression(exp1));
  EXPECT_TRUE(a.has_expression(exp2));
  EXPECT_TRUE(a.has_expression(exp3));
  EXPECT_EQ(0, a.find_expression(exp1));
  EXPECT_EQ(1, a.find_expression(exp2));
  EXPECT_EQ(2, a.find_expression(exp3));
}

TEST_F(ExpressionArrayTest, ValueCalculation)
{
  DoubleExpressionArray a;
  a.set_expressions(exps);

  a.update(vector_state); 
  Eigen::VectorXd values = a.get_values();
  ASSERT_EQ(values.rows(), num_exps);
  ASSERT_EQ(values.cols(), 1);
  EXPECT_DOUBLE_EQ(3.0, values(0));
  EXPECT_DOUBLE_EQ(7.0, values(1));
  EXPECT_DOUBLE_EQ(18.0, values(2));

  a.update(eigen_state);
  values = a.get_values();
  ASSERT_EQ(values.rows(), num_exps);
  ASSERT_EQ(values.cols(), 1);
  EXPECT_DOUBLE_EQ(6.0, values(0));
  EXPECT_DOUBLE_EQ(14.0, values(1));
  EXPECT_DOUBLE_EQ(36.0, values(2));
}

TEST_F(ExpressionArrayTest, DerivativeCalculation)
{
  DoubleExpressionArray a;
  a.set_expressions(exps);

  a.update(vector_state); 
  Eigen::MatrixXd derivatives = a.get_derivatives();
  ASSERT_EQ(derivatives.rows(), num_exps);
  ASSERT_EQ(derivatives.cols(), num_derivs);
  EXPECT_DOUBLE_EQ(derivatives(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(derivatives(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(derivatives(0, 2), 0.0);
  EXPECT_DOUBLE_EQ(derivatives(0, 3), 0.0);
  EXPECT_DOUBLE_EQ(derivatives(0, 4), 0.0);
  EXPECT_DOUBLE_EQ(derivatives(1, 0), 0.0);
  EXPECT_DOUBLE_EQ(derivatives(1, 1), 0.0);
  EXPECT_DOUBLE_EQ(derivatives(1, 2), 0.0);
  EXPECT_DOUBLE_EQ(derivatives(1, 3), 3.0);
  EXPECT_DOUBLE_EQ(derivatives(1, 4), 4.0);
  EXPECT_DOUBLE_EQ(derivatives(2, 0), 0.0);
  EXPECT_DOUBLE_EQ(derivatives(2, 1), 5.0);
  EXPECT_DOUBLE_EQ(derivatives(2, 2), 6.0);
  EXPECT_DOUBLE_EQ(derivatives(2, 3), 7.0);
  EXPECT_DOUBLE_EQ(derivatives(2, 4), 0.0);

  a.update(eigen_state); 
  derivatives = a.get_derivatives();
  ASSERT_EQ(derivatives.rows(), num_exps);
  ASSERT_EQ(derivatives.cols(), num_derivs);
  EXPECT_DOUBLE_EQ(derivatives(0, 0), 1.0);
  EXPECT_DOUBLE_EQ(derivatives(0, 1), 2.0);
  EXPECT_DOUBLE_EQ(derivatives(0, 2), 0.0);
  EXPECT_DOUBLE_EQ(derivatives(0, 3), 0.0);
  EXPECT_DOUBLE_EQ(derivatives(0, 4), 0.0);
  EXPECT_DOUBLE_EQ(derivatives(1, 0), 0.0);
  EXPECT_DOUBLE_EQ(derivatives(1, 1), 0.0);
  EXPECT_DOUBLE_EQ(derivatives(1, 2), 0.0);
  EXPECT_DOUBLE_EQ(derivatives(1, 3), 3.0);
  EXPECT_DOUBLE_EQ(derivatives(1, 4), 4.0);
  EXPECT_DOUBLE_EQ(derivatives(2, 0), 0.0);
  EXPECT_DOUBLE_EQ(derivatives(2, 1), 5.0);
  EXPECT_DOUBLE_EQ(derivatives(2, 2), 6.0);
  EXPECT_DOUBLE_EQ(derivatives(2, 3), 7.0);
  EXPECT_DOUBLE_EQ(derivatives(2, 4), 0.0);
}

TEST_F(ExpressionArrayTest, DerivativeExpressions)
{
  DoubleExpressionArray a;
  a.set_expressions(exps);
  ASSERT_EQ(3, a.num_expressions());
  a.update(vector_state); 

  std::vector< KDL::Expression<double>::Ptr > deriv_exps = a.get_derivative_expressions(0);
  ASSERT_EQ(deriv_exps.size(), 2);
  EXPECT_DOUBLE_EQ(deriv_exps[0]->value(), 1.0);
  EXPECT_DOUBLE_EQ(deriv_exps[1]->value(), 2.0);

  deriv_exps = a.get_derivative_expressions(1);
  ASSERT_EQ(deriv_exps.size(), 5);
  EXPECT_DOUBLE_EQ(deriv_exps[0]->value(), 0.0);
  EXPECT_DOUBLE_EQ(deriv_exps[1]->value(), 0.0);
  EXPECT_DOUBLE_EQ(deriv_exps[2]->value(), 0.0);
  EXPECT_DOUBLE_EQ(deriv_exps[3]->value(), 3.0);
  EXPECT_DOUBLE_EQ(deriv_exps[4]->value(), 4.0);

  deriv_exps = a.get_derivative_expressions(2);
  ASSERT_EQ(deriv_exps.size(), 4);
  EXPECT_DOUBLE_EQ(deriv_exps[0]->value(), 0.0);
  EXPECT_DOUBLE_EQ(deriv_exps[1]->value(), 5.0);
  EXPECT_DOUBLE_EQ(deriv_exps[2]->value(), 6.0);
  EXPECT_DOUBLE_EQ(deriv_exps[3]->value(), 7.0);
}
