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

class QPControllerTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      mu = 0.6;
      nWSR = 100;

      controllable_lower.push_back(KDL::Constant(-0.1));
      controllable_lower.push_back(KDL::Constant(-0.3));
     
      controllable_upper.push_back(KDL::Constant(0.1));
      controllable_upper.push_back(KDL::Constant(0.3));

      controllable_weights.push_back(KDL::Constant(mu * 1.1));
      controllable_weights.push_back(KDL::Constant(mu * 1.2));

      controllable_names.push_back("dof 1");
      controllable_names.push_back("dof 2");

      KDL::Expression<double>::Ptr exp1 = KDL::cached<double>(KDL::input(0));
      KDL::Expression<double>::Ptr exp2 = KDL::cached<double>(KDL::input(1));
      KDL::Expression<double>::Ptr exp3 = KDL::cached<double>(KDL::Constant(2.0)*exp1 + exp2);

      soft_expressions.push_back(exp1);
      soft_expressions.push_back(exp2);
      soft_expressions.push_back(exp3);

      using KDL::operator-;
      using KDL::operator*;
      soft_lower.push_back(KDL::Constant(2.0) * (KDL::Constant(0.75) - exp1));
      soft_lower.push_back(KDL::Constant(2.0) * (KDL::Constant(-1.5) - exp2));
      soft_lower.push_back(KDL::Constant(2.0) * (KDL::Constant(0.3) - exp3));
      
      soft_upper.push_back(KDL::Constant(2.0) * (KDL::Constant(1.1) - exp1));
      soft_upper.push_back(KDL::Constant(2.0) * (KDL::Constant(-1.3) - exp2));
      soft_upper.push_back(KDL::Constant(2.0) * (KDL::Constant(0.35) - exp3));

      soft_weights.push_back(KDL::Constant(mu + 11));
      soft_weights.push_back(KDL::Constant(mu + 12));
      soft_weights.push_back(KDL::Constant(mu + 13));

      soft_names.push_back("dof 1 goal");
      soft_names.push_back("dof 2 goal");
      soft_names.push_back("dof 1 and 2 combined goal");

      hard_expressions.push_back(exp1);
      hard_expressions.push_back(exp2);

      hard_lower.push_back(KDL::Constant(-3.0) - exp1);
      hard_lower.push_back(KDL::Constant(-3.1) - exp2);

      hard_upper.push_back(KDL::Constant(3.0) - exp1);
      hard_upper.push_back(KDL::Constant(3.1) - exp2);

      using Eigen::operator<<;
      initial_state.resize(2);
      initial_state << -1.77, 2.5;
    }

    virtual void TearDown(){}

    std::vector< KDL::Expression<double>::Ptr > controllable_lower, controllable_upper,
        controllable_weights, soft_expressions, soft_lower, soft_upper, soft_weights,
        hard_expressions, hard_lower, hard_upper;
    std::vector<std::string> soft_names, controllable_names;
    Eigen::VectorXd initial_state;

    double mu;
    int nWSR;
};

TEST_F(QPControllerTest, Constructor)
{
  giskard_core::QPController c;

  EXPECT_EQ(0, c.get_command().rows());
}

TEST_F(QPControllerTest, Init)
{
   giskard_core::QPController c;

   ASSERT_TRUE(c.init(controllable_lower, controllable_upper, controllable_weights, 
         controllable_names, soft_expressions, soft_lower, soft_upper, soft_weights, 
         soft_names, hard_expressions, hard_lower, hard_upper));

   ASSERT_TRUE(c.start(initial_state, nWSR));

   EXPECT_EQ(2, c.get_command().rows());
}

TEST_F(QPControllerTest, Update)
{
   // setup controller
   giskard_core::QPController c;
   ASSERT_TRUE(c.init(controllable_lower, controllable_upper, controllable_weights, 
         controllable_names, soft_expressions, soft_lower, soft_upper, soft_weights, 
         soft_names, hard_expressions, hard_lower, hard_upper));
   ASSERT_TRUE(c.start(initial_state, nWSR));

   // run several dozen simulation runs; enough to converge
   Eigen::VectorXd state = initial_state;
   for(size_t i=0; i<36; ++i)
   {
     ASSERT_TRUE(c.update(state, nWSR));
     ASSERT_EQ(2, c.get_command().rows());
     state += c.get_command();
   }

   // make sure the constraints are all fulfilled
   for(size_t i=0; i<soft_lower.size(); ++i)
     EXPECT_LE(soft_lower[i]->value(), 0.0);

   for(size_t i=0; i<soft_upper.size(); ++i)
     EXPECT_LE(0.0, soft_upper[i]->value());

   for(size_t i=0; i<hard_lower.size(); ++i)
     EXPECT_LE(hard_lower[i]->value(), 0.0);

   for(size_t i=0; i<hard_upper.size(); ++i)
     EXPECT_LE(0.0, hard_upper[i]->value());
}

TEST_F(QPControllerTest, CommandMap)
{
  // setup controller
  giskard_core::QPController c;
  ASSERT_TRUE(c.init(controllable_lower, controllable_upper, controllable_weights, 
       controllable_names, soft_expressions, soft_lower, soft_upper, soft_weights, 
       soft_names, hard_expressions, hard_lower, hard_upper));
  ASSERT_TRUE(c.start(initial_state, nWSR));

  auto cmd_map = c.get_command_map();
  for(size_t i = 0; i < controllable_names.size(); i++) {
    if (cmd_map.find(controllable_names[i]) == cmd_map.end())
      FAIL();   
  }
}


// Tests for all 'set_input' functions
TEST_F(QPControllerTest, SetInputs) {
  YAML::Node node = YAML::LoadFile("named_input_test.yaml");


  giskard_core::QPControllerSpec qp_spec;
  ASSERT_NO_THROW(qp_spec = node.as< giskard_core::QPControllerSpec >());

  // Build a controller
  giskard_core::QPController c = giskard_core::generate(qp_spec);

  const giskard_core::Scope& scope = c.get_scope();
  giskard_core::Scope::ScalarInputPtr   sIn;
  giskard_core::Scope::JointInputPtr    jIn;
  giskard_core::Scope::Vec3InputPtr     vIn;
  giskard_core::Scope::RotationInputPtr rIn;
  giskard_core::Scope::FrameInputPtr    fIn;
  giskard_core::Scope::RotationInputPtr rIn2;
  giskard_core::Scope::FrameInputPtr    fIn2;

  ASSERT_NO_THROW(jIn = scope.find_input<giskard_core::Scope::JointInput>("joint_input"));
  ASSERT_TRUE(!!jIn);

  ASSERT_NO_THROW(sIn = scope.find_input<giskard_core::Scope::ScalarInput>("scalar_input"));
  ASSERT_TRUE(!!sIn);

  ASSERT_NO_THROW(vIn = scope.find_input<giskard_core::Scope::Vec3Input>("vector_input"));
  ASSERT_TRUE(!!vIn);

  ASSERT_NO_THROW(rIn = scope.find_input<giskard_core::Scope::RotationInput>("rotation_input"));
  ASSERT_TRUE(!!rIn);

  ASSERT_NO_THROW(fIn = scope.find_input<giskard_core::Scope::FrameInput>("frame_input"));
  ASSERT_TRUE(!!fIn);

  ASSERT_NO_THROW(rIn2 = scope.find_input<giskard_core::Scope::RotationInput>("rotation_input2"));
  ASSERT_TRUE(!!rIn2);

  ASSERT_NO_THROW(fIn2 = scope.find_input<giskard_core::Scope::FrameInput>("frame_input2"));
  ASSERT_TRUE(!!fIn2);

  // Check that the input size is correct
  ASSERT_EQ(c.get_input_size(), 28);

  // Initialize input vector as too small
  Eigen::VectorXd inputs = Eigen::VectorXd::Zero(1);

  // Set non-existing variables
  EXPECT_THROW(c.set_input(inputs, "bla", 1.0), std::invalid_argument); 
  EXPECT_THROW(c.set_input(inputs, "foo", 1.0, 2, 3), std::invalid_argument); 
  EXPECT_THROW(c.set_input(inputs, "bar", 1.0, 2, 3, 7), std::invalid_argument); 
  EXPECT_THROW(c.set_input(inputs, "meh", 1.0, 2, 3, 7, 5, 7, 9), std::invalid_argument); 
  
  // Set variable on too small vector
  EXPECT_THROW(c.set_input(inputs, "scalar_input", 1.0), std::invalid_argument); 
  EXPECT_THROW(c.set_input(inputs, "vector_input", 1.0, 2, 3), std::invalid_argument); 
  EXPECT_THROW(c.set_input(inputs, "rotation_input", 1.0, 2, 3, 7), std::invalid_argument); 
  EXPECT_THROW(c.set_input(inputs, "frame_input", 1.0, 2, 3, 7, 5, 7, 9), std::invalid_argument); 

  // Resize input vector correctly
  inputs = Eigen::VectorXd::Zero(c.get_input_size());

  // Set variable on correct vector but wrong types
  EXPECT_THROW(c.set_input(inputs, "joint_input", 1.2, 1, 2), std::invalid_argument);
  EXPECT_THROW(c.set_input(inputs, "scalar_input", 3.5, 5, 1, 6), std::invalid_argument);
  EXPECT_THROW(c.set_input(inputs, "vector_input", 3.5), std::invalid_argument);
  EXPECT_THROW(c.set_input(inputs, "rotation_input", 1, 2, 3, 4, 5, 6, 7), std::invalid_argument);

  EXPECT_TRUE(inputs == Eigen::VectorXd::Zero(c.get_input_size()));

  // Test primitive set operations
  EXPECT_NO_THROW(c.set_input(inputs, "joint_input", 1.2));
  EXPECT_NO_THROW(c.set_input(inputs, "scalar_input", 3.5));
  EXPECT_NO_THROW(c.set_input(inputs, "vector_input", 1, 2, 3));
  EXPECT_NO_THROW(c.set_input(inputs, "rotation_input", 1, 0, 0, 1.56));
  EXPECT_NO_THROW(c.set_input(inputs, "frame_input", 1, 0, 0, 1.56, 1, 2, 3));

  ASSERT_TRUE(c.start(inputs, nWSR));

  KDL::Vector kdlVec(1,2,3);
  KDL::Rotation kdlRot = KDL::Rotation::Rot(KDL::Vector(1,0,0), 1.56);
  KDL::Rotation kdlRot2 = KDL::Rotation::Rot(KDL::Vector(0,1,0), -0.56);
  KDL::Frame kdlFrame(kdlRot, kdlVec);
  KDL::Frame kdlFrame2(kdlRot2, kdlVec);

  // Evaluate input
  EXPECT_DOUBLE_EQ(1.2, jIn->expr_->value());  
  EXPECT_DOUBLE_EQ(3.5, sIn->expr_->value());
  EXPECT_TRUE(KDL::Equal(kdlVec, vIn->expr_->value(), KDL::epsilon));
  EXPECT_TRUE(KDL::Equal(kdlRot, rIn->expr_->value(), KDL::epsilon));
  EXPECT_TRUE(KDL::Equal(kdlFrame, fIn->expr_->value(), KDL::epsilon));

  // Reset the inputs
  inputs = Eigen::VectorXd::Zero(c.get_input_size());

  // Test KDL operations
  EXPECT_NO_THROW(c.set_input(inputs, "vector_input", kdlVec));
  EXPECT_NO_THROW(c.set_input(inputs, "rotation_input", kdlRot));
  EXPECT_NO_THROW(c.set_input(inputs, "frame_input", kdlFrame));
  EXPECT_NO_THROW(c.set_input(inputs, "frame_input2", kdlRot2, kdlVec));
  
  ASSERT_TRUE(c.update(inputs, nWSR));

  EXPECT_TRUE(KDL::Equal(kdlVec, vIn->expr_->value(), KDL::epsilon));
  EXPECT_TRUE(KDL::Equal(kdlRot, rIn->expr_->value(), KDL::epsilon));
  EXPECT_TRUE(KDL::Equal(kdlFrame, fIn->expr_->value(), KDL::epsilon));
  EXPECT_TRUE(KDL::Equal(kdlFrame2, fIn2->expr_->value(), KDL::epsilon));

  // Reset the inputs
  inputs = Eigen::VectorXd::Zero(c.get_input_size());

  // Test eigen operations
  Eigen::Vector3d eigVec(1,2,3);
  Eigen::Quaterniond eigRot(Eigen::AngleAxisd( 1.56, Eigen::Vector3d(1,0,0)));
  Eigen::Quaterniond eigRot2(Eigen::AngleAxisd(-0.56, Eigen::Vector3d(0,1,0)));
  Eigen::Affine3d eigFrame = Eigen::Translation3d(eigVec) * eigRot;
  Eigen::Affine3d eigFrame2 = Eigen::Translation3d(eigVec) * eigRot2;

  EXPECT_NO_THROW(c.set_input(inputs, "vector_input", eigVec));
  EXPECT_NO_THROW(c.set_input(inputs, "rotation_input", eigRot));
  EXPECT_NO_THROW(c.set_input(inputs, "rotation_input2", Eigen::Vector3d(0,1,0), -0.56));
  EXPECT_NO_THROW(c.set_input(inputs, "frame_input", eigFrame));
  EXPECT_NO_THROW(c.set_input(inputs, "frame_input2", eigRot2, eigVec));

  ASSERT_TRUE(c.update(inputs, nWSR));

  EXPECT_TRUE(KDL::Equal(kdlVec, vIn->expr_->value(), KDL::epsilon));
  EXPECT_TRUE(KDL::Equal(kdlRot, rIn->expr_->value(), KDL::epsilon));
  EXPECT_TRUE(KDL::Equal(kdlFrame, fIn->expr_->value(), KDL::epsilon));
  EXPECT_TRUE(KDL::Equal(kdlFrame2, fIn2->expr_->value(), KDL::epsilon));

  inputs = Eigen::VectorXd::Zero(c.get_input_size());

  EXPECT_NO_THROW(c.set_input(inputs, "frame_input", Eigen::Vector3d(1,0,0), 1.56, eigVec));  

  ASSERT_TRUE(c.update(inputs, nWSR));

  EXPECT_TRUE(KDL::Equal(kdlFrame, fIn->expr_->value(), KDL::epsilon));
}

TEST_F(QPControllerTest, InputGetters) {
  YAML::Node node = YAML::LoadFile("named_input_test.yaml");

  giskard_core::QPControllerSpec qp_spec;
  ASSERT_NO_THROW(qp_spec = node.as< giskard_core::QPControllerSpec >());

  // Build a controller
  giskard_core::QPController c = giskard_core::generate(qp_spec);

  const giskard_core::Scope& scope = c.get_scope();

  EXPECT_EQ(scope.get_input_names(), c.get_input_names());
  EXPECT_EQ(scope.get_inputs(), c.get_inputs());
  EXPECT_EQ(scope.get_input_map(), c.get_input_map());

  EXPECT_EQ(scope.get_inputs<giskard_core::Scope::JointInput>(), c.get_inputs<giskard_core::Scope::JointInput>());
  EXPECT_EQ(scope.get_input_map<giskard_core::Scope::JointInput>(), c.get_input_map<giskard_core::Scope::JointInput>());

  EXPECT_EQ(scope.get_inputs<giskard_core::Scope::ScalarInput>(), c.get_inputs<giskard_core::Scope::ScalarInput>());
  EXPECT_EQ(scope.get_input_map<giskard_core::Scope::ScalarInput>(), c.get_input_map<giskard_core::Scope::ScalarInput>());

  EXPECT_EQ(scope.get_inputs<giskard_core::Scope::Vec3Input>(), c.get_inputs<giskard_core::Scope::Vec3Input>());
  EXPECT_EQ(scope.get_input_map<giskard_core::Scope::Vec3Input>(), c.get_input_map<giskard_core::Scope::Vec3Input>());
  
  EXPECT_EQ(scope.get_inputs<giskard_core::Scope::RotationInput>(), c.get_inputs<giskard_core::Scope::RotationInput>());
  EXPECT_EQ(scope.get_input_map<giskard_core::Scope::RotationInput>(), c.get_input_map<giskard_core::Scope::RotationInput>());
  
  EXPECT_EQ(scope.get_inputs<giskard_core::Scope::FrameInput>(), c.get_inputs<giskard_core::Scope::FrameInput>());
  EXPECT_EQ(scope.get_input_map<giskard_core::Scope::FrameInput>(), c.get_input_map<giskard_core::Scope::FrameInput>());
}