#include <gtest/gtest.h>
#include <giskard/giskard.hpp>


class ExpressionScopeTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      double_a = KDL::input(0);
      double_b = KDL::Constant(1.0);
      frame_1 = KDL::frame(KDL::rot_x(KDL::Constant(M_PI/2.0)));
      frame_2 = KDL::frame(KDL::vector(KDL::Constant(1.0), KDL::Constant(2.0), KDL::Constant(3.0)));
    }

    virtual void TearDown(){}

    KDL::Expression<double>::Ptr double_a, double_b;
    KDL::Expression<KDL::Frame>::Ptr frame_1, frame_2;
};

TEST_F(ExpressionScopeTest, HasDouble)
{
  giskard::ExpressionScope scope;

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
}

TEST_F(ExpressionScopeTest, HasFrame)
{
  giskard::ExpressionScope scope;

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
}

TEST_F(ExpressionScopeTest, FindDouble)
{
  giskard::ExpressionScope scope;

  scope.add_double_expression("a", double_a);
  EXPECT_EQ(double_a, scope.find_double_expression("a"));

  scope.add_double_expression("b", double_b);
  EXPECT_EQ(double_a, scope.find_double_expression("a"));
  EXPECT_EQ(double_b, scope.find_double_expression("b"));
}

TEST_F(ExpressionScopeTest, FindFrame)
{
  giskard::ExpressionScope scope;

  scope.add_frame_expression("1", frame_1);
  EXPECT_EQ(frame_1, scope.find_frame_expression("1"));

  scope.add_frame_expression("2", frame_2);
  EXPECT_EQ(frame_1, scope.find_frame_expression("1"));
  EXPECT_EQ(frame_2, scope.find_frame_expression("2"));
}
