#include <gtest/gtest.h>
#include <giskard/giskard.hpp>


class RotationGenerationTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
    }

    virtual void TearDown(){}
};

TEST_F(RotationGenerationTest, AxisAngle)
{
  giskard::ConstDoubleSpecPtr x(new giskard::ConstDoubleSpec());
  x->set_value(1.0);
  giskard::ConstDoubleSpecPtr y(new giskard::ConstDoubleSpec());
  y->set_value(0.0);
  giskard::ConstDoubleSpecPtr z(new giskard::ConstDoubleSpec());
  z->set_value(0.0);
  giskard::ConstDoubleSpecPtr angle(new giskard::ConstDoubleSpec());
  angle->set_value(M_PI/2.0);

  giskard::ConstructorVectorSpecPtr axis(new giskard::ConstructorVectorSpec());
  axis->set(x, y, z);
 
  giskard::AxisAngleSpec spec;
  spec.set_axis(axis);
  spec.set_angle(angle);

  giskard::Scope scope;
  KDL::Rotation rot = spec.get_expression(scope)->value();
  KDL::Rotation rot2 = KDL::Rotation::Rot(KDL::Vector(1.0, 0.0, 0.0), M_PI/2.0);

  EXPECT_TRUE(KDL::Equal(rot, rot2));
}

TEST_F(RotationGenerationTest, AxisAngleEquality)
{
  giskard::ConstDoubleSpecPtr x(new giskard::ConstDoubleSpec());
  x->set_value(1.0);
  giskard::ConstDoubleSpecPtr y(new giskard::ConstDoubleSpec());
  y->set_value(0.0);
  giskard::ConstDoubleSpecPtr z(new giskard::ConstDoubleSpec());
  z->set_value(0.0);
  giskard::ConstDoubleSpecPtr angle(new giskard::ConstDoubleSpec());
  angle->set_value(M_PI/2.0);

  giskard::ConstructorVectorSpecPtr axis1(new giskard::ConstructorVectorSpec());
  giskard::ConstructorVectorSpecPtr axis2(new giskard::ConstructorVectorSpec());
  axis1->set(x, y, z);
  axis2->set(x, x, x);
 
  giskard::AxisAngleSpec s1, s2, s3, s4;
  s1.set_axis(axis1);
  s1.set_angle(angle);
  s2.set_axis(axis1);
  s2.set_angle(x);
  s3.set_axis(axis2);
  s3.set_angle(angle);
  s4.set_axis(axis1);
  s4.set_angle(angle);

  EXPECT_TRUE(s1.equals(s1));
  EXPECT_FALSE(s1.equals(s2));
  EXPECT_FALSE(s1.equals(s3));
  EXPECT_TRUE(s1.equals(s4));

  EXPECT_EQ(s1, s1);
  EXPECT_NE(s1, s2);
  EXPECT_NE(s1, s3);
  EXPECT_EQ(s1, s4);
}
