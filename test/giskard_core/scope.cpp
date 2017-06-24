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


class ScopeTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      double_a = KDL::input(0);
      double_b = KDL::Constant(1.0);
      frame_1 = KDL::frame(KDL::rot_x(KDL::Constant(M_PI/2.0)));
      frame_2 = KDL::frame(KDL::vector(KDL::Constant(1.0), KDL::Constant(2.0), KDL::Constant(3.0)));
      rot_1 = KDL::rot_x(KDL::Constant(M_PI/2.0));
      rot_2 = KDL::rot_x(KDL::Constant(-M_PI/2.0));
      vec_I = KDL::vector(KDL::Constant(1.0), KDL::Constant(0.0), KDL::Constant(0.0));
      vec_II = KDL::vector(KDL::Constant(2.0), KDL::Constant(0.0), KDL::Constant(0.0));
    }

    virtual void TearDown(){}

    KDL::Expression<double>::Ptr double_a, double_b;
    KDL::Expression<KDL::Frame>::Ptr frame_1, frame_2;
    KDL::Expression<KDL::Rotation>::Ptr rot_1, rot_2;
    KDL::Expression<KDL::Vector>::Ptr vec_I, vec_II;
};

TEST_F(ScopeTest, GetVectorNames)
{
  giskard_core::Scope scope;

  ASSERT_EQ(scope.get_vector_names().size(), 0);

  scope.add_vector_expression("I", vec_I);
  ASSERT_EQ(scope.get_vector_names().size(), 1);
  EXPECT_STREQ(scope.get_vector_names()[0].c_str(), "I");
 
  scope.add_vector_expression("II", vec_II);
  ASSERT_EQ(scope.get_vector_names().size(), 2);
  EXPECT_STREQ(scope.get_vector_names()[0].c_str(), "I");
  EXPECT_STREQ(scope.get_vector_names()[1].c_str(), "II");

  EXPECT_EQ(scope.get_double_names().size(), 0);
  EXPECT_EQ(scope.get_rotation_names().size(), 0);
  EXPECT_EQ(scope.get_frame_names().size(), 0);
}

TEST_F(ScopeTest, HasDouble)
{
  giskard_core::Scope scope;

  EXPECT_FALSE(scope.has_double_expression("a"));
  EXPECT_FALSE(scope.has_double_expression("b"));
  EXPECT_FALSE(scope.has_double_expression("c"));

  scope.add_double_expression("a", double_a);
  EXPECT_TRUE(scope.has_double_expression("a"));
  EXPECT_FALSE(scope.has_double_expression("b"));
  EXPECT_FALSE(scope.has_double_expression("c"));
 
  scope.add_double_expression("b", double_b);
  EXPECT_TRUE(scope.has_double_expression("a"));
  EXPECT_TRUE(scope.has_double_expression("b"));
  EXPECT_FALSE(scope.has_double_expression("c"));

  EXPECT_FALSE(scope.has_frame_expression("a"));
  EXPECT_FALSE(scope.has_frame_expression("b"));
  EXPECT_FALSE(scope.has_frame_expression("c"));

  EXPECT_FALSE(scope.has_rotation_expression("1"));
  EXPECT_FALSE(scope.has_rotation_expression("2"));
  EXPECT_FALSE(scope.has_rotation_expression("3"));
}

TEST_F(ScopeTest, GetDoubleNames)
{
  giskard_core::Scope scope;

  ASSERT_EQ(scope.get_double_names().size(), 0);

  scope.add_double_expression("a", double_a);
  ASSERT_EQ(scope.get_double_names().size(), 1);
  EXPECT_STREQ(scope.get_double_names()[0].c_str(), "a");
 
  scope.add_double_expression("b", double_b);
  ASSERT_EQ(scope.get_double_names().size(), 2);
  EXPECT_STREQ(scope.get_double_names()[0].c_str(), "a");
  EXPECT_STREQ(scope.get_double_names()[1].c_str(), "b");

  EXPECT_EQ(scope.get_vector_names().size(), 0);
  EXPECT_EQ(scope.get_rotation_names().size(), 0);
  EXPECT_EQ(scope.get_frame_names().size(), 0);
}

TEST_F(ScopeTest, HasFrame)
{
  giskard_core::Scope scope;

  EXPECT_FALSE(scope.has_frame_expression("1"));
  EXPECT_FALSE(scope.has_frame_expression("2"));
  EXPECT_FALSE(scope.has_frame_expression("3"));

  scope.add_frame_expression("1", frame_1);
  EXPECT_TRUE(scope.has_frame_expression("1"));
  EXPECT_FALSE(scope.has_frame_expression("2"));
  EXPECT_FALSE(scope.has_frame_expression("3"));

  scope.add_frame_expression("2", frame_2);
  EXPECT_TRUE(scope.has_frame_expression("1"));
  EXPECT_TRUE(scope.has_frame_expression("2"));
  EXPECT_FALSE(scope.has_frame_expression("3"));

  EXPECT_FALSE(scope.has_double_expression("1"));
  EXPECT_FALSE(scope.has_double_expression("2"));
  EXPECT_FALSE(scope.has_double_expression("3"));

  EXPECT_FALSE(scope.has_rotation_expression("1"));
  EXPECT_FALSE(scope.has_rotation_expression("2"));
  EXPECT_FALSE(scope.has_rotation_expression("3"));
}

TEST_F(ScopeTest, GetFrameNames)
{
  giskard_core::Scope scope;

  ASSERT_EQ(scope.get_frame_names().size(), 0);

  scope.add_frame_expression("1", frame_1);
  ASSERT_EQ(scope.get_frame_names().size(), 1);
  EXPECT_STREQ(scope.get_frame_names()[0].c_str(), "1");
 
  scope.add_frame_expression("2", frame_2);
  ASSERT_EQ(scope.get_frame_names().size(), 2);
  EXPECT_STREQ(scope.get_frame_names()[0].c_str(), "1");
  EXPECT_STREQ(scope.get_frame_names()[1].c_str(), "2");

  EXPECT_EQ(scope.get_vector_names().size(), 0);
  EXPECT_EQ(scope.get_rotation_names().size(), 0);
  EXPECT_EQ(scope.get_double_names().size(), 0);
}

TEST_F(ScopeTest, HasRotation)
{
  giskard_core::Scope scope;

  EXPECT_FALSE(scope.has_rotation_expression("1"));
  EXPECT_FALSE(scope.has_rotation_expression("2"));
  EXPECT_FALSE(scope.has_rotation_expression("3"));

  scope.add_rotation_expression("1", rot_1);
  EXPECT_TRUE(scope.has_rotation_expression("1"));
  EXPECT_FALSE(scope.has_rotation_expression("2"));
  EXPECT_FALSE(scope.has_rotation_expression("3"));

  scope.add_rotation_expression("2", rot_2);
  EXPECT_TRUE(scope.has_rotation_expression("1"));
  EXPECT_TRUE(scope.has_rotation_expression("2"));
  EXPECT_FALSE(scope.has_rotation_expression("3"));

  EXPECT_FALSE(scope.has_double_expression("1"));
  EXPECT_FALSE(scope.has_double_expression("2"));
  EXPECT_FALSE(scope.has_double_expression("3"));

  EXPECT_FALSE(scope.has_frame_expression("1"));
  EXPECT_FALSE(scope.has_frame_expression("2"));
  EXPECT_FALSE(scope.has_frame_expression("3"));
}

TEST_F(ScopeTest, GetRotationNames)
{
  giskard_core::Scope scope;

  ASSERT_EQ(scope.get_rotation_names().size(), 0);

  scope.add_rotation_expression("1", rot_1);
  ASSERT_EQ(scope.get_rotation_names().size(), 1);
  EXPECT_STREQ(scope.get_rotation_names()[0].c_str(), "1");
 
  scope.add_rotation_expression("2", rot_2);
  ASSERT_EQ(scope.get_rotation_names().size(), 2);
  EXPECT_STREQ(scope.get_rotation_names()[0].c_str(), "1");
  EXPECT_STREQ(scope.get_rotation_names()[1].c_str(), "2");

  EXPECT_EQ(scope.get_vector_names().size(), 0);
  EXPECT_EQ(scope.get_frame_names().size(), 0);
  EXPECT_EQ(scope.get_double_names().size(), 0);
}

TEST_F(ScopeTest, FindDouble)
{
  giskard_core::Scope scope;

  scope.add_double_expression("a", double_a);
  EXPECT_EQ(double_a, scope.find_double_expression("a"));

  scope.add_double_expression("b", double_b);
  EXPECT_EQ(double_a, scope.find_double_expression("a"));
  EXPECT_EQ(double_b, scope.find_double_expression("b"));
}

TEST_F(ScopeTest, FindFrame)
{
  giskard_core::Scope scope;

  scope.add_frame_expression("1", frame_1);
  EXPECT_EQ(frame_1, scope.find_frame_expression("1"));

  scope.add_frame_expression("2", frame_2);
  EXPECT_EQ(frame_1, scope.find_frame_expression("1"));
  EXPECT_EQ(frame_2, scope.find_frame_expression("2"));
}

TEST_F(ScopeTest, FindRotation)
{
  giskard_core::Scope scope;

  scope.add_rotation_expression("rot_1", rot_1);
  EXPECT_EQ(rot_1, scope.find_rotation_expression("rot_1"));

  scope.add_rotation_expression("rot_2", rot_2);
  EXPECT_EQ(rot_1, scope.find_rotation_expression("rot_1"));
  EXPECT_EQ(rot_2, scope.find_rotation_expression("rot_2"));
}


TEST_F(ScopeTest, ScalarInputs) {
  giskard_core::Scope scope;

  // Add a scalar input with name 'input1'
  EXPECT_NO_THROW(scope.add_scalar_input("input1"));

  // Check that the input size is correct
  ASSERT_EQ(scope.get_input_size(), 1);
  
  // Has input by name
  EXPECT_TRUE(scope.has_input("input1"));
  
  // Has input of matching type with name
  EXPECT_TRUE(scope.has_input<giskard_core::Scope::ScalarInput>("input1"));

  // Does not have input of matching type with mismatched name
  EXPECT_FALSE(scope.has_input<giskard_core::Scope::ScalarInput>("bla"));

  // Does not have input of mismatching type with name
  EXPECT_FALSE(scope.has_input<giskard_core::Scope::Vec3Input>("input1"));
  
  // Find general input with name
  EXPECT_NO_THROW(scope.find_input("input1"));

  // Find typed input with matching name
  EXPECT_NO_THROW(scope.find_input<giskard_core::Scope::ScalarInput>("input1"));

  // Don't find typed input with mismatched name
  EXPECT_ANY_THROW(scope.find_input<giskard_core::Scope::ScalarInput>("bla"));
  
  // Don't find input with mismatched type and matching name
  EXPECT_ANY_THROW(scope.find_input<giskard_core::Scope::Vec3Input>("input1"));

  // Allow adding the input a second time with matching type
  EXPECT_NO_THROW(scope.add_scalar_input("input1"));

  // Don't allow adding the input a second time with different type
  EXPECT_ANY_THROW(scope.add_vector_input("input1"));
}

TEST_F(ScopeTest, JointInputs) {
  giskard_core::Scope scope;

  // Add a scalar input with name 'input1'
  EXPECT_NO_THROW(scope.add_joint_input("joint1"));
  
  // Has input by name
  EXPECT_TRUE(scope.has_input("joint1"));
  
  // Has input of matching type with name
  EXPECT_TRUE(scope.has_input<giskard_core::Scope::JointInput>("joint1"));

  // Does not have input of matching type with mismatched name
  EXPECT_FALSE(scope.has_input<giskard_core::Scope::JointInput>("bla"));

  // Does not have input of mismatching type with name
  EXPECT_FALSE(scope.has_input<giskard_core::Scope::Vec3Input>("joint1"));
  
  // Find general input with name
  EXPECT_NO_THROW(scope.find_input("joint1"));

  // Find typed input with matching name
  EXPECT_NO_THROW(scope.find_input<giskard_core::Scope::JointInput>("joint1"));

  // Don't find typed input with mismatched name
  EXPECT_ANY_THROW(scope.find_input<giskard_core::Scope::JointInput>("bla"));
  
  // Don't find input with mismatched type and matching name
  EXPECT_ANY_THROW(scope.find_input<giskard_core::Scope::Vec3Input>("joint1"));

  // Allow adding the input a second time with matching type
  EXPECT_NO_THROW(scope.add_joint_input("joint1"));

  // Don't allow adding the input a second time with different type
  EXPECT_ANY_THROW(scope.add_vector_input("joint1"));

  // Don't allow adding a joint after an input of another type
  EXPECT_NO_THROW(scope.add_vector_input("someVec"));
  EXPECT_ANY_THROW(scope.add_joint_input("joint2"));
}

TEST_F(ScopeTest, VectorInputs) {
  giskard_core::Scope scope;

  // Add a scalar input with name 'input1'
  EXPECT_NO_THROW(scope.add_vector_input("vector1"));
  
  // Has input by name
  EXPECT_TRUE(scope.has_input("vector1"));
  
  // Has input of matching type with name
  EXPECT_TRUE(scope.has_input<giskard_core::Scope::Vec3Input>("vector1"));

  // Does not have input of matching type with mismatched name
  EXPECT_FALSE(scope.has_input<giskard_core::Scope::Vec3Input>("bla"));

  // Does not have input of mismatching type with name
  EXPECT_FALSE(scope.has_input<giskard_core::Scope::ScalarInput>("vector1"));
  
  // Find general input with name
  EXPECT_NO_THROW(scope.find_input("vector1"));

  // Find typed input with matching name
  EXPECT_NO_THROW(scope.find_input<giskard_core::Scope::Vec3Input>("vector1"));

  // Don't find typed input with mismatched name
  EXPECT_ANY_THROW(scope.find_input<giskard_core::Scope::Vec3Input>("bla"));
  
  // Don't find input with mismatched type and matching name
  EXPECT_ANY_THROW(scope.find_input<giskard_core::Scope::ScalarInput>("vector1"));

  // Allow adding the input a second time with matching type
  EXPECT_NO_THROW(scope.add_vector_input("vector1"));

  // Don't allow adding the input a second time with different type
  EXPECT_ANY_THROW(scope.add_scalar_input("vector1"));
}

TEST_F(ScopeTest, RotationInputs) {
  giskard_core::Scope scope;

  // Add a scalar input with name 'input1'
  EXPECT_NO_THROW(scope.add_rotation_input("rotation1"));
  
  // Has input by name
  EXPECT_TRUE(scope.has_input("rotation1"));
  
  // Has input of matching type with name
  EXPECT_TRUE(scope.has_input<giskard_core::Scope::RotationInput>("rotation1"));

  // Does not have input of matching type with mismatched name
  EXPECT_FALSE(scope.has_input<giskard_core::Scope::RotationInput>("bla"));

  // Does not have input of mismatching type with name
  EXPECT_FALSE(scope.has_input<giskard_core::Scope::Vec3Input>("rotation1"));
  
  // Find general input with name
  EXPECT_NO_THROW(scope.find_input("rotation1"));

  // Find typed input with matching name
  EXPECT_NO_THROW(scope.find_input<giskard_core::Scope::RotationInput>("rotation1"));

  // Don't find typed input with mismatched name
  EXPECT_ANY_THROW(scope.find_input<giskard_core::Scope::RotationInput>("bla"));
  
  // Don't find input with mismatched type and matching name
  EXPECT_ANY_THROW(scope.find_input<giskard_core::Scope::Vec3Input>("rotation1"));

  // Allow adding the input a second time with matching type
  EXPECT_NO_THROW(scope.add_rotation_input("rotation1"));

  // Don't allow adding the input a second time with different type
  EXPECT_ANY_THROW(scope.add_vector_input("rotation1"));
}

TEST_F(ScopeTest, FrameInputs) {
  giskard_core::Scope scope;

  // Add a scalar input with name 'input1'
  EXPECT_NO_THROW(scope.add_frame_input("frame1"));
  
  // Has input by name
  EXPECT_TRUE(scope.has_input("frame1"));
  
  // Has input of matching type with name
  EXPECT_TRUE(scope.has_input<giskard_core::Scope::FrameInput>("frame1"));

  // Does not have input of matching type with mismatched name
  EXPECT_FALSE(scope.has_input<giskard_core::Scope::FrameInput>("bla"));

  // Does not have input of mismatching type with name
  EXPECT_FALSE(scope.has_input<giskard_core::Scope::Vec3Input>("frame1"));
  
  // Find general input with name
  EXPECT_NO_THROW(scope.find_input("frame1"));

  // Find typed input with matching name
  EXPECT_NO_THROW(scope.find_input<giskard_core::Scope::FrameInput>("frame1"));

  // Don't find typed input with mismatched name
  EXPECT_ANY_THROW(scope.find_input<giskard_core::Scope::FrameInput>("bla"));
  
  // Don't find input with mismatched type and matching name
  EXPECT_ANY_THROW(scope.find_input<giskard_core::Scope::Vec3Input>("frame1"));

  // Allow adding the input a second time with matching type
  EXPECT_NO_THROW(scope.add_frame_input("frame1"));

  // Don't allow adding the input a second time with different type
  EXPECT_ANY_THROW(scope.add_vector_input("frame1"));
}

// This is not the best test
TEST_F(ScopeTest, GetInputs) {
  giskard_core::Scope scope;
  auto names  = scope.get_input_names();
  auto inputs = scope.get_inputs();
  auto inpMao = scope.get_input_map();
  
  auto inJ = scope.get_inputs<giskard_core::Scope::JointInput>();
  auto inS = scope.get_inputs<giskard_core::Scope::ScalarInput>();
  auto inV = scope.get_inputs<giskard_core::Scope::Vec3Input>();
  auto inR = scope.get_inputs<giskard_core::Scope::RotationInput>();
  auto inF = scope.get_inputs<giskard_core::Scope::FrameInput>();

  auto inJMap = scope.get_input_map<giskard_core::Scope::JointInput>();
  auto inSMap = scope.get_input_map<giskard_core::Scope::ScalarInput>();
  auto inVMap = scope.get_input_map<giskard_core::Scope::Vec3Input>();
  auto inRMap = scope.get_input_map<giskard_core::Scope::RotationInput>();
  auto inFMap = scope.get_input_map<giskard_core::Scope::FrameInput>();

  EXPECT_EQ(0, names.size());
  EXPECT_EQ(0, inputs.size());
  EXPECT_EQ(0, inpMao.size());

  EXPECT_EQ(0, inJ.size());
  EXPECT_EQ(0, inS.size());
  EXPECT_EQ(0, inV.size());
  EXPECT_EQ(0, inR.size());
  EXPECT_EQ(0, inF.size());

  EXPECT_EQ(0, inJMap.size());
  EXPECT_EQ(0, inSMap.size());
  EXPECT_EQ(0, inVMap.size());
  EXPECT_EQ(0, inRMap.size());
  EXPECT_EQ(0, inFMap.size());

  ASSERT_NO_THROW(scope.add_joint_input("joint1"));
  ASSERT_NO_THROW(scope.add_joint_input("joint2"));
  ASSERT_NO_THROW(scope.add_joint_input("joint3"));
  ASSERT_NO_THROW(scope.add_scalar_input("scalar1"));
  ASSERT_NO_THROW(scope.add_scalar_input("scalar2"));
  ASSERT_NO_THROW(scope.add_scalar_input("scalar3"));
  ASSERT_NO_THROW(scope.add_vector_input("vector1"));
  ASSERT_NO_THROW(scope.add_vector_input("vector2"));
  ASSERT_NO_THROW(scope.add_vector_input("vector3"));
  ASSERT_NO_THROW(scope.add_rotation_input("rotation1"));
  ASSERT_NO_THROW(scope.add_rotation_input("rotation2"));
  ASSERT_NO_THROW(scope.add_rotation_input("rotation3"));
  ASSERT_NO_THROW(scope.add_frame_input("frame1"));
  ASSERT_NO_THROW(scope.add_frame_input("frame2"));
  ASSERT_NO_THROW(scope.add_frame_input("frame3"));

  names  = scope.get_input_names();
  inputs = scope.get_inputs();
  inpMao = scope.get_input_map();
  
  inJ = scope.get_inputs<giskard_core::Scope::JointInput>();
  inS = scope.get_inputs<giskard_core::Scope::ScalarInput>();
  inV = scope.get_inputs<giskard_core::Scope::Vec3Input>();
  inR = scope.get_inputs<giskard_core::Scope::RotationInput>();
  inF = scope.get_inputs<giskard_core::Scope::FrameInput>();

  inJMap = scope.get_input_map<giskard_core::Scope::JointInput>();
  inSMap = scope.get_input_map<giskard_core::Scope::ScalarInput>();
  inVMap = scope.get_input_map<giskard_core::Scope::Vec3Input>();
  inRMap = scope.get_input_map<giskard_core::Scope::RotationInput>();
  inFMap = scope.get_input_map<giskard_core::Scope::FrameInput>();

  EXPECT_EQ(15, names.size());
  EXPECT_EQ(15, inputs.size());
  EXPECT_EQ(15, inpMao.size());

  EXPECT_EQ(3, inJ.size());
  EXPECT_EQ(3, inS.size());
  EXPECT_EQ(3, inV.size());
  EXPECT_EQ(3, inR.size());
  EXPECT_EQ(3, inF.size());

  EXPECT_EQ(3, inJMap.size());
  EXPECT_EQ(3, inSMap.size());
  EXPECT_EQ(3, inVMap.size());
  EXPECT_EQ(3, inRMap.size());
  EXPECT_EQ(3, inFMap.size());

}