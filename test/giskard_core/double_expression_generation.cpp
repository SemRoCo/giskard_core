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
  giskard_core::DoubleConstSpec descr;
  giskard_core::Scope scope;

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
  giskard_core::DoubleConstSpec d1, d2, d3;

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
  giskard_core::DoubleInputSpec descrS("someScalar");
  giskard_core::JointInputSpec  descrJ("someJoint");
  
  giskard_core::Scope scope;

  EXPECT_EQ(descrS.get_name(), "someScalar");
  EXPECT_EQ(descrJ.get_name(), "someJoint");
  EXPECT_EQ(descrS.get_type(), giskard_core::InputType::tScalar);
  EXPECT_EQ(descrJ.get_type(), giskard_core::InputType::tJoint);

  EXPECT_ANY_THROW(descrS.get_expression(scope));
  EXPECT_ANY_THROW(descrJ.get_expression(scope));

  ASSERT_NO_THROW(scope.add_joint_input(descrJ.get_name()));
  ASSERT_NO_THROW(scope.add_scalar_input(descrS.get_name()));

  KDL::Expression<double>::Ptr exprS;
  KDL::Expression<double>::Ptr exprJ;

  ASSERT_NO_THROW(exprS = descrS.get_expression(scope));
  ASSERT_NO_THROW(exprJ = descrJ.get_expression(scope));

  exprS->setInputValue(0, 0.1);
  exprJ->setInputValue(0, 0.2);

  EXPECT_LE(abs(exprJ->value()- 0.1), 0.00000001);
  EXPECT_LE(abs(exprS->value()- 0.1), 0.00000001);

  ASSERT_EQ(exprJ->number_of_derivatives(), 1);
  EXPECT_DOUBLE_EQ(exprJ->derivative(0), 1.0); 

  ASSERT_EQ(exprS->number_of_derivatives(), 2);
  EXPECT_DOUBLE_EQ(exprS->derivative(0), 0.0); 
  EXPECT_DOUBLE_EQ(exprS->derivative(1), 1.0); 
}

TEST_F(DoubleExpressionGenerationTest, InputEquality)
{
  giskard_core::DoubleInputSpec d1("in1"), d2("in2"), d3("in1");
  giskard_core::JointInputSpec j1("in1"), j2("in2"), j3("in1");
  giskard_core::Scope scope;

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

  EXPECT_TRUE(j1.equals(j1));
  EXPECT_FALSE(j1.equals(j2));
  EXPECT_TRUE(j1.equals(j3));

  EXPECT_EQ(j1, j1);
  EXPECT_NE(j1, j2);
  EXPECT_EQ(j1, j3);

  EXPECT_FALSE(j2.equals(j1));
  EXPECT_TRUE(j2.equals(j2));
  EXPECT_FALSE(j2.equals(j3));

  EXPECT_TRUE(j3.equals(j1));
  EXPECT_FALSE(j3.equals(j2));
  EXPECT_TRUE(j3.equals(j3));

  EXPECT_FALSE(d1.equals(j1));
  EXPECT_FALSE(d2.equals(j2));
}

TEST_F(DoubleExpressionGenerationTest, ReferenceEquality)
{
  giskard_core::DoubleReferenceSpec d1, d2, d3;
  giskard_core::Scope scope;

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
  giskard_core::DoubleConstSpecPtr const_descr1(new
      giskard_core::DoubleConstSpec());
  giskard_core::DoubleConstSpecPtr const_descr2(new
      giskard_core::DoubleConstSpec());
  giskard_core::DoubleInputSpecPtr input_descr(new
      giskard_core::DoubleInputSpec("in1"));

  giskard_core::DoubleAdditionSpec add_descr;
  giskard_core::Scope scope;

  const_descr1->set_value(-0.7);
  EXPECT_DOUBLE_EQ(const_descr1->get_value(), -0.7);

  const_descr2->set_value(0.3);
  EXPECT_DOUBLE_EQ(const_descr2->get_value(), 0.3);

  EXPECT_EQ(input_descr->get_name(), "in1");
  ASSERT_NO_THROW(scope.add_scalar_input(input_descr->get_name()));

  std::vector<giskard_core::DoubleSpecPtr> input_descrs;
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
  
  // TODO: I don't know what this tests and thus can't update the case for it
  // ASSERT_EQ(expr->number_of_derivatives(), 2);

  // expr->setInputValue(0, 0.0);
  // expr->setInputValue(1, 1.5);
  // EXPECT_DOUBLE_EQ(expr->value(), 1.1);
  // EXPECT_DOUBLE_EQ(expr->derivative(0), 0.0); 
  // EXPECT_DOUBLE_EQ(expr->derivative(1), 1.0); 
}

TEST_F(DoubleExpressionGenerationTest, AdditionEquality)
{
  giskard_core::DoubleConstSpecPtr dd1(new giskard_core::DoubleConstSpec());
  giskard_core::DoubleConstSpecPtr dd2(new giskard_core::DoubleConstSpec());
  giskard_core::DoubleInputSpecPtr dd3(new giskard_core::DoubleInputSpec("in1"));

  dd1->set_value(1.0);
  dd2->set_value(2.0);

  std::vector<giskard_core::DoubleSpecPtr> in1, in2, in3, in4, in5;
  in1.push_back(dd1);
  in2.push_back(dd2);
  in3.push_back(dd3);
  in4.push_back(dd1);
  in4.push_back(dd3);
  in5.push_back(dd1);

  giskard_core::DoubleAdditionSpec a1, a2, a3, a4, a5;
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

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr spec = node.as<giskard_core::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 3.0, 1e-10);
}

TEST_F(DoubleExpressionGenerationTest, DoubleMultiplication)
{
  std::string s = "{double-mul: [-0.5, 2.0, 3.5, 0.1]}";

  YAML::Node node = YAML::Load(s);

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr spec = node.as<giskard_core::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), -0.35, 1e-10);
}

TEST_F(DoubleExpressionGenerationTest, DoubleDivision)
{
  std::string s1 = "{double-div: [2.0]}";
  std::string s2 = "{double-div: [-1.0, 4.0]}";
  std::string s3 = "{double-div: [1.0, 2.0, 5.0]}";

  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr spec = node.as<giskard_core::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 0.5, 1e-10);

  node = YAML::Load(s2);

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  spec = node.as<giskard_core::DoubleSpecPtr>();

  exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), -0.25, 1e-10);

  node = YAML::Load(s3);

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  spec = node.as<giskard_core::DoubleSpecPtr>();

  exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 0.1, 1e-10);
}

TEST_F(DoubleExpressionGenerationTest, DoubleSubtraction)
{
  std::string s1 = "{double-sub: [-0.5]}";
  std::string s2 = "{double-sub: [0.5, 2.0]}";
  std::string s3 = "{double-sub: [1.1, 0.2, -0.3]}";

  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr spec = node.as<giskard_core::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 0.5, 1e-10);

  node = YAML::Load(s2);

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  spec = node.as<giskard_core::DoubleSpecPtr>();

  exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), -1.5, 1e-10);

  node = YAML::Load(s3);

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  spec = node.as<giskard_core::DoubleSpecPtr>();

  exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 1.2, 1e-10);
}

TEST_F(DoubleExpressionGenerationTest, XCoordOf)
{
  std::string s1 = "{x-coord: {vector3: [1.1, 2.2, 3.3]}}";

  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr spec = node.as<giskard_core::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 1.1, 1e-10);
}

TEST_F(DoubleExpressionGenerationTest, YCoordOf)
{
  std::string s1 = "{y-coord: {vector3: [1.1, 2.2, 3.3]}}";

  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr spec = node.as<giskard_core::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 2.2, 1e-10);
}

TEST_F(DoubleExpressionGenerationTest, ZCoordOf)
{
  std::string s1 = "{z-coord: {vector3: [1.1, 2.2, 3.3]}}";

  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr spec = node.as<giskard_core::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 3.3, 1e-10);
}

TEST_F(DoubleExpressionGenerationTest, VectorDot)
{
  std::string s1 = "{vector-dot: [{vector3: [1, 2, 3]}, {vector3: [4, 5, 6]}]}";

  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr spec = node.as<giskard_core::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 32, 1e-10);
}

TEST_F(DoubleExpressionGenerationTest, Minimum)
{
  std::string s1 = "{min: [1.2, -2.1]}";

  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr spec = node.as<giskard_core::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), -2.1, 1e-10);
}

TEST_F(DoubleExpressionGenerationTest, DoubleIf)
{
  std::string s1 = "{double-if: [0.1, 1.1, 2.2]}";
  std::string s2 = "{double-if: [-0.1, 1.1, 2.2]}";
  std::string s3 = "{double-if: [0.0, 1.1, 2.2]}";

  // CASE 1
  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr spec = node.as<giskard_core::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 1.1, 1e-10);

  // CASE 2
  node = YAML::Load(s2);
  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  spec = node.as<giskard_core::DoubleSpecPtr>();

  exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 2.2, 1e-10);

  // CASE 3
  node = YAML::Load(s3);
  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  spec = node.as<giskard_core::DoubleSpecPtr>();

  exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 1.1, 1e-10);
}

TEST_F(DoubleExpressionGenerationTest, Abs)
{
  std::string s1 = "{abs: 0.1}";
  std::string s2 = "{abs: -0.2}";
  std::string s3 = "{abs: 0.0}";

  // CASE 1
  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr spec = node.as<giskard_core::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 0.1, 1e-10);

  // CASE 2
  node = YAML::Load(s2);
  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  spec = node.as<giskard_core::DoubleSpecPtr>();

  exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 0.2, 1e-10);

  // CASE 3
  node = YAML::Load(s3);
  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  spec = node.as<giskard_core::DoubleSpecPtr>();

  exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), 0.0, 1e-10);
}

TEST_F(DoubleExpressionGenerationTest, Fmod)
{
  std::string s1 = "{fmod: [0.1, 1]}";
  std::string s2 = "{fmod: [-3.2, 2]}";
  std::string s3 = "{fmod: [2.1, 1.5]}";

  // CASE 1
  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  giskard_core::DoubleSpecPtr spec = node.as<giskard_core::DoubleSpecPtr>();

  KDL::Expression<double>::Ptr exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), std::fmod(0.1, 1.0), 1e-10);

  // CASE 2
  node = YAML::Load(s2);
  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  spec = node.as<giskard_core::DoubleSpecPtr>();

  exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), std::fmod(-3.2, 2.0), 1e-10);

  // CASE 3
  node = YAML::Load(s3);
  ASSERT_NO_THROW(node.as<giskard_core::DoubleSpecPtr>());
  spec = node.as<giskard_core::DoubleSpecPtr>();

  exp = spec->get_expression(giskard_core::Scope());
  ASSERT_TRUE(exp.get());

  EXPECT_NEAR(exp->value(), std::fmod(2.1, 1.5), 1e-10);
}
