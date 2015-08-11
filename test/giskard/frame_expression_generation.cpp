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
