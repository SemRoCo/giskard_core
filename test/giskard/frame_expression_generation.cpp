#include <gtest/gtest.h>
#include <giskard/giskard.hpp>


class FrameGenerationTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
    }

    virtual void TearDown(){}
};

TEST_F(FrameGenerationTest, ConstructorFrame)
{
  giskard::ConstDoubleSpecPtr rot_axis_x(new giskard::ConstDoubleSpec());
  rot_axis_x->set_value(1.0);
  giskard::ConstDoubleSpecPtr rot_axis_y(new giskard::ConstDoubleSpec());
  rot_axis_y->set_value(2.0);
  giskard::ConstDoubleSpecPtr rot_axis_z(new giskard::ConstDoubleSpec());
  rot_axis_z->set_value(3.0);
  giskard::ConstructorVectorSpecPtr rot_axis(new giskard::ConstructorVectorSpec());
  rot_axis->set(rot_axis_x, rot_axis_y, rot_axis_z);

  giskard::ConstDoubleSpecPtr rot_angle(new giskard::ConstDoubleSpec());
  rot_angle->set_value(-4.0);

  giskard::AxisAngleSpecPtr rot(new giskard::AxisAngleSpec());
  rot->set_axis(rot_axis);
  rot->set_angle(rot_angle);

  giskard::ConstDoubleSpecPtr trans_x(new giskard::ConstDoubleSpec());
  giskard::ConstDoubleSpecPtr trans_y(new giskard::ConstDoubleSpec());
  giskard::ConstDoubleSpecPtr trans_z(new giskard::ConstDoubleSpec());
  trans_x->set_value(-0.1);
  trans_y->set_value(-0.2);
  trans_z->set_value(-0.3);
  giskard::ConstructorVectorSpecPtr trans(new giskard::ConstructorVectorSpec());
  trans->set(trans_x, trans_y, trans_z);

  giskard::ConstructorFrameSpec spec;
  spec.set_translation(trans);
  spec.set_rotation(rot);

  giskard::Scope scope;

  KDL::Frame frame1 = spec.get_expression(scope)->value();
  KDL::Frame frame2 = KDL::Frame(KDL::Rotation::Rot(KDL::Vector(1.0, 2.0, 3.0), -4.0),
      KDL::Vector(-0.1, -0.2, -0.3));

  EXPECT_TRUE(KDL::Equal(frame1, frame2));
};

TEST_F(FrameGenerationTest, ConstructorEquality)
{
  giskard::ConstDoubleSpecPtr one = giskard::ConstDoubleSpecPtr(new giskard::ConstDoubleSpec());
  giskard::ConstDoubleSpecPtr zero = giskard::ConstDoubleSpecPtr(new giskard::ConstDoubleSpec());
  giskard::ConstDoubleSpecPtr pi = giskard::ConstDoubleSpecPtr(new giskard::ConstDoubleSpec());
  one->set_value(0.0);
  zero->set_value(1.0);
  pi->set_value(M_PI);

  giskard::ConstructorVectorSpecPtr trans1 = 
      giskard::ConstructorVectorSpecPtr(new giskard::ConstructorVectorSpec());
  giskard::ConstructorVectorSpecPtr trans2 = 
      giskard::ConstructorVectorSpecPtr(new giskard::ConstructorVectorSpec());
  giskard::ConstructorVectorSpecPtr axis1 = 
      giskard::ConstructorVectorSpecPtr(new giskard::ConstructorVectorSpec());
  giskard::ConstructorVectorSpecPtr axis2 = 
      giskard::ConstructorVectorSpecPtr(new giskard::ConstructorVectorSpec());
  trans1->set(one, one, one);
  trans2->set(zero, zero, one);
  axis1->set(one, zero, zero);
  axis2->set(zero, one, zero);

  giskard::AxisAngleSpecPtr rot1 =
      giskard::AxisAngleSpecPtr(new giskard::AxisAngleSpec());
  giskard::AxisAngleSpecPtr rot2 =
      giskard::AxisAngleSpecPtr(new giskard::AxisAngleSpec());
  
  giskard::ConstructorFrameSpec d1, d2, d3, d4, d5;

  d1.set_rotation(rot1);
  d1.set_translation(trans1);
  d2.set_rotation(rot1);
  d2.set_translation(trans2);
  d3.set_rotation(rot2);
  d3.set_translation(trans1);
  d4.set_rotation(rot2);
  d4.set_translation(trans2);
  d5.set_rotation(rot1);
  d5.set_translation(trans1);

  EXPECT_TRUE(d1.equals(d1));
  EXPECT_FALSE(d1.equals(d2));
  EXPECT_FALSE(d1.equals(d3));
  EXPECT_FALSE(d1.equals(d4));
  EXPECT_TRUE(d1.equals(d5));

  EXPECT_EQ(d1, d1);
  EXPECT_NE(d1, d2);
  EXPECT_NE(d1, d3);
  EXPECT_NE(d1, d4);
  EXPECT_EQ(d1, d5);
}

TEST_F(FrameGenerationTest, ConstructorMultiplicationEquality)
{
  giskard::ConstDoubleSpecPtr one = giskard::ConstDoubleSpecPtr(new giskard::ConstDoubleSpec());
  giskard::ConstDoubleSpecPtr zero = giskard::ConstDoubleSpecPtr(new giskard::ConstDoubleSpec());
  giskard::ConstDoubleSpecPtr pi = giskard::ConstDoubleSpecPtr(new giskard::ConstDoubleSpec());
  one->set_value(0.0);
  zero->set_value(1.0);
  pi->set_value(M_PI);

  giskard::ConstructorVectorSpecPtr trans1 = 
      giskard::ConstructorVectorSpecPtr(new giskard::ConstructorVectorSpec());
  giskard::ConstructorVectorSpecPtr trans2 = 
      giskard::ConstructorVectorSpecPtr(new giskard::ConstructorVectorSpec());
  giskard::ConstructorVectorSpecPtr axis1 = 
      giskard::ConstructorVectorSpecPtr(new giskard::ConstructorVectorSpec());
  giskard::ConstructorVectorSpecPtr axis2 = 
      giskard::ConstructorVectorSpecPtr(new giskard::ConstructorVectorSpec());
  trans1->set(one, one, one);
  trans2->set(zero, zero, one);
  axis1->set(one, zero, zero);
  axis2->set(zero, one, zero);

  giskard::AxisAngleSpecPtr rot1 =
      giskard::AxisAngleSpecPtr(new giskard::AxisAngleSpec());
  giskard::AxisAngleSpecPtr rot2 =
      giskard::AxisAngleSpecPtr(new giskard::AxisAngleSpec());
  
  giskard::ConstructorFrameSpecPtr d1 = 
      giskard::ConstructorFrameSpecPtr(new giskard::ConstructorFrameSpec());
  giskard::ConstructorFrameSpecPtr d2 = 
      giskard::ConstructorFrameSpecPtr(new giskard::ConstructorFrameSpec());

  d1->set_rotation(rot1);
  d1->set_translation(trans1);
  d2->set_rotation(rot2);
  d2->set_translation(trans2);

  std::vector<giskard::FrameSpecPtr> in1, in2, in3, in4, in5;
  in1.push_back(d1);
  in2.push_back(d2);
  in3.push_back(d1);
  in3.push_back(d1);
  in4.push_back(d1);
  in4.push_back(d2);
  in5.push_back(d1);

  giskard::MultiplicationFrameSpec a1, a2, a3, a4, a5;
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
