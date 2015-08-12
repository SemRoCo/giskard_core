#include <gtest/gtest.h>
#include <giskard/giskard.hpp>


class VectorExpressionGenerationTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      x = giskard::ConstDoubleSpecPtr(new giskard::ConstDoubleSpec());
      y = giskard::ConstDoubleSpecPtr(new giskard::ConstDoubleSpec());
      z = giskard::ConstDoubleSpecPtr(new giskard::ConstDoubleSpec());

      x->set_value(1.1);
      y->set_value(2.2);
      z->set_value(3.3);
    }

    virtual void TearDown(){}

    giskard::ConstDoubleSpecPtr x, y, z;
};

TEST_F(VectorExpressionGenerationTest, Constructor)
{
  giskard::ConstructorVectorSpec descr;
  giskard::Scope scope;

  descr.set_x(x);
  descr.set_y(y);
  descr.set_z(z);

  KDL::Vector v = descr.get_expression(scope)->value();
  
  EXPECT_DOUBLE_EQ(v.x(), 1.1);
  EXPECT_DOUBLE_EQ(v.y(), 2.2);
  EXPECT_DOUBLE_EQ(v.z(), 3.3);
}

TEST_F(VectorExpressionGenerationTest, ConstructorEquality)
{
  giskard::ConstructorVectorSpec d1, d2, d3;

  d1.set(x, y, z);
  d2.set(x, y, y);
  d3.set(x, y, z);

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
