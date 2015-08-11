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
