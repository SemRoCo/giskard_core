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

class QPProblemBuilderTest : public ::testing::Test
{

  protected:
    virtual void SetUp()
    {
      mu = 0.6;

      controllable_lower.push_back(KDL::Constant(-0.1));
      controllable_lower.push_back(KDL::Constant(-0.3));
     
      controllable_upper.push_back(KDL::Constant(0.1));
      controllable_upper.push_back(KDL::Constant(0.3));

      controllable_weights.push_back(KDL::Constant(mu * 1.1));
      controllable_weights.push_back(KDL::Constant(mu * 1.2));
 
      KDL::Expression<double>::Ptr exp1 = KDL::cached<double>(KDL::input(0));
      KDL::Expression<double>::Ptr exp2 = KDL::cached<double>(KDL::input(1));
      KDL::Expression<double>::Ptr exp3 = KDL::cached<double>(KDL::Constant(2.0)*exp1 + exp2);

      soft_expressions.push_back(exp1);
      soft_expressions.push_back(exp2);
      soft_expressions.push_back(exp3);

      soft_lower.push_back(KDL::Constant(0.75));
      soft_lower.push_back(KDL::Constant(-1.5));
      soft_lower.push_back(KDL::Constant(0.3));
      
      soft_upper.push_back(KDL::Constant(1.1));
      soft_upper.push_back(KDL::Constant(-1.3));
      soft_upper.push_back(KDL::Constant(0.35));

      soft_weights.push_back(KDL::Constant(mu + 11));
      soft_weights.push_back(KDL::Constant(mu + 12));
      soft_weights.push_back(KDL::Constant(mu + 1.3));

      hard_expressions.push_back(exp1);
      hard_expressions.push_back(exp2);

      hard_lower.push_back(KDL::Constant(-3.0));
      hard_lower.push_back(KDL::Constant(-3.1));

      hard_upper.push_back(KDL::Constant(3.0));
      hard_upper.push_back(KDL::Constant(3.1));

      using Eigen::operator<<;
      initial_state.resize(2);
      initial_state << -1.77, 2.5;
    }

    virtual void TearDown(){}

    std::vector< KDL::Expression<double>::Ptr > controllable_lower, controllable_upper,
        controllable_weights, soft_expressions, soft_lower, soft_upper, soft_weights,
        hard_expressions, hard_lower, hard_upper;
    Eigen::VectorXd initial_state;

    double mu;

    void CompareMatrices(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b)
    {
      ASSERT_EQ(a.rows(), b.rows());
      ASSERT_EQ(a.cols(), b.cols());

      for(size_t row=0; row<a.rows(); ++row)
       for(size_t col=0; col<a.cols(); ++col)
         EXPECT_DOUBLE_EQ(a(row, col), b(row, col));
    }

    void CompareVectors(const Eigen::VectorXd& a, const Eigen::VectorXd& b)
    {
      ASSERT_EQ(a.rows(), b.rows());
      
      for(size_t row=0; row<a.rows(); ++row)
        EXPECT_DOUBLE_EQ(a(row), b(row));
    }
};

TEST_F(QPProblemBuilderTest, Constructor)
{
  giskard::core::QPProblemBuilder b;
  EXPECT_EQ(0, b.num_weights());
  EXPECT_EQ(0, b.num_constraints());
  EXPECT_EQ(0, b.num_soft_constraints());
  EXPECT_EQ(0, b.num_hard_constraints());
  EXPECT_EQ(0, b.num_soft_constraints_observables());
  EXPECT_EQ(0, b.num_hard_constraints_observables());
  EXPECT_EQ(0, b.num_controllables());

  EXPECT_EQ(0, b.get_ubA().rows());
  EXPECT_EQ(1, b.get_ubA().cols());

  EXPECT_EQ(0, b.get_lbA().rows());
  EXPECT_EQ(1, b.get_lbA().cols());

  EXPECT_EQ(0, b.get_ub().rows());
  EXPECT_EQ(1, b.get_ub().cols());

  EXPECT_EQ(0, b.get_lb().rows());
  EXPECT_EQ(1, b.get_lb().cols());

  EXPECT_EQ(0, b.get_g().rows());
  EXPECT_EQ(1, b.get_g().cols());

  EXPECT_EQ(0, b.get_A().rows());
  EXPECT_EQ(0, b.get_A().cols());

  EXPECT_EQ(0, b.get_H().rows());
  EXPECT_EQ(0, b.get_H().cols());
}

TEST_F(QPProblemBuilderTest, Init)
{
  giskard::core::QPProblemBuilder b;
  b.init(controllable_lower, controllable_upper, controllable_weights, soft_expressions,
      soft_lower, soft_upper, soft_weights, hard_expressions, hard_lower, hard_upper);

  EXPECT_EQ(5, b.num_weights());
  EXPECT_EQ(5, b.num_constraints());
  EXPECT_EQ(3, b.num_soft_constraints());
  EXPECT_EQ(2, b.num_hard_constraints());
  EXPECT_EQ(2, b.num_soft_constraints_observables());
  EXPECT_EQ(2, b.num_hard_constraints_observables());
  EXPECT_EQ(2, b.num_controllables());

  EXPECT_EQ(5, b.get_ubA().rows());
  EXPECT_EQ(1, b.get_ubA().cols());

  EXPECT_EQ(5, b.get_lbA().rows());
  EXPECT_EQ(1, b.get_lbA().cols());

  EXPECT_EQ(5, b.get_ub().rows());
  EXPECT_EQ(1, b.get_ub().cols());

  EXPECT_EQ(5, b.get_lb().rows());
  EXPECT_EQ(1, b.get_lb().cols());

  EXPECT_EQ(5, b.get_g().rows());
  EXPECT_EQ(1, b.get_g().cols());

  EXPECT_EQ(5, b.get_A().rows());
  EXPECT_EQ(5, b.get_A().cols());

  EXPECT_EQ(5, b.get_H().rows());
  EXPECT_EQ(5, b.get_H().cols());
}

TEST_F(QPProblemBuilderTest, Update)
{
  giskard::core::QPProblemBuilder b;
  b.init(controllable_lower, controllable_upper, controllable_weights, soft_expressions,
      soft_lower, soft_upper, soft_weights, hard_expressions, hard_lower, hard_upper);
  b.update(initial_state);

  EXPECT_EQ(5, b.num_weights());
  EXPECT_EQ(5, b.num_constraints());
  EXPECT_EQ(3, b.num_soft_constraints());
  EXPECT_EQ(2, b.num_hard_constraints());
  EXPECT_EQ(2, b.num_soft_constraints_observables());
  EXPECT_EQ(2, b.num_hard_constraints_observables());
  EXPECT_EQ(2, b.num_controllables());

  using Eigen::operator<<;
  Eigen::MatrixXd H(5,5);
  H << mu * 1.1, 0, 0, 0, 0,
       0, mu * 1.2, 0, 0., 0,
       0, 0, mu + 11, 0, 0,
       0, 0, 0, mu + 12, 0,
       0, 0, 0, 0, mu +1.3; 
  CompareMatrices(H, b.get_H());

  Eigen::MatrixXd A(5,5);
  A << 1, 0, 0, 0, 0,
       0, 1, 0, 0, 0,
       1, 0, 1, 0, 0,
       0, 1, 0, 1, 0,
       2, 1, 0, 0, 1; 
  CompareMatrices(A, b.get_A());

  CompareVectors(Eigen::VectorXd::Zero(5), b.get_g());

  Eigen::VectorXd lb(5);
  lb << -0.1, -0.3 , -1e9, -1e9, -1e9;
  CompareVectors(lb, b.get_lb());
     
  Eigen::VectorXd ub(5);
  ub << 0.1, 0.3 , 1e9, 1e9, 1e9;
  CompareVectors(ub, b.get_ub());
 
  Eigen::VectorXd lbA(5);
  lbA << -3.0, -3.1, 0.75, -1.5, 0.3;
  CompareVectors(lbA, b.get_lbA());
 
  Eigen::VectorXd ubA(5);
  ubA << 3.0, 3.1, 1.1, -1.3, 0.35;
  CompareVectors(lbA, b.get_lbA());
}
