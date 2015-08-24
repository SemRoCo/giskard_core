#include <gtest/gtest.h>
#include <giskard/giskard.hpp>


class VectorExpressionGenerationTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      x = giskard::DoubleConstSpecPtr(new giskard::DoubleConstSpec());
      y = giskard::DoubleConstSpecPtr(new giskard::DoubleConstSpec());
      z = giskard::DoubleConstSpecPtr(new giskard::DoubleConstSpec());

      x->set_value(1.1);
      y->set_value(2.2);
      z->set_value(3.3);
    }

    virtual void TearDown(){}

    giskard::DoubleConstSpecPtr x, y, z;
};

TEST_F(VectorExpressionGenerationTest, Constructor)
{
  giskard::VectorConstructorSpec descr;
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
  giskard::VectorConstructorSpec d1, d2, d3;

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

TEST_F(VectorExpressionGenerationTest, VectorSubtraction)
{
  std::string v1 = "{type: VECTOR3, inputs: [1.1, 2.2, 3.3]}";
  std::string v2 = "{type: VECTOR3, inputs: [-0.5, 0.5, 1.5]}";

  std::string s1 = "{type: VECTOR-SUBTRACTION, inputs: [" + v1 + "]}";
  std::string s2 = "{type: VECTOR-SUBTRACTION, inputs: [" + v1 + ", " + v2 + "]}";
  std::string s3 = "{type: VECTOR-SUBTRACTION, inputs: [" + v1 + ", " + v2 + ", " + v2 + "]}";

  // test 1
  YAML::Node node = YAML::Load(s1);

  ASSERT_NO_THROW(node.as<giskard::VectorSpecPtr>());
  giskard::VectorSpecPtr spec = node.as<giskard::VectorSpecPtr>();
  
  giskard::Scope scope;
  ASSERT_NO_THROW(spec->get_expression(scope));
  KDL::Expression<KDL::Vector>::Ptr exp = spec->get_expression(scope);
  
  ASSERT_TRUE(exp.get());
  KDL::Vector val1 = exp->value();
  KDL::Vector val2 = KDL::Vector(-1.1, -2.2, -3.3);
  EXPECT_TRUE(KDL::Equal(val1, val2));

  // test 2
  node = YAML::Load(s2);

  ASSERT_NO_THROW(node.as<giskard::VectorSpecPtr>());
  spec = node.as<giskard::VectorSpecPtr>();
  
  ASSERT_NO_THROW(spec->get_expression(scope));
  exp = spec->get_expression(scope);
  
  ASSERT_TRUE(exp.get());
  val1 = exp->value();
  val2 = KDL::Vector(1.6, 1.7, 1.8);
  EXPECT_TRUE(KDL::Equal(val1, val2));

  // test 3
  node = YAML::Load(s3);

  ASSERT_NO_THROW(node.as<giskard::VectorSpecPtr>());
  spec = node.as<giskard::VectorSpecPtr>();
  
  ASSERT_NO_THROW(spec->get_expression(scope));
  exp = spec->get_expression(scope);
  
  ASSERT_TRUE(exp.get());
  val1 = exp->value();
  val2 = KDL::Vector(2.1, 1.2, 0.3);
  EXPECT_TRUE(KDL::Equal(val1, val2));
}

TEST_F(VectorExpressionGenerationTest, VectorFrameMultiplication)
{
  std::string v1 = "{type: VECTOR3, inputs: [0.1, 0.2, 0.3]}";
  std::string v2 = "{type: VECTOR3, inputs: [1, 2, 3]}";
  std::string r = "{type: ROTATION, axis: {type: VECTOR3, inputs: [1, 0, 0]}, angle: 0}";
  std::string f = "{type: FRAME, translation: " + v1 + ", rotation: " + r + "}";
  std::string s1 = "{type: MULTIPLICATION, frame: " + f + ", vector: " + v2 + "}";

  YAML::Node node = YAML::Load(s1);
//std::cout << node << "\n\n";
  ASSERT_NO_THROW(node.as<giskard::VectorSpecPtr>());
  giskard::VectorSpecPtr spec = node.as<giskard::VectorSpecPtr>();
  
  ASSERT_NO_THROW(spec->get_expression(giskard::Scope()));
  KDL::Expression<KDL::Vector>::Ptr exp = spec->get_expression(giskard::Scope());
  
  ASSERT_TRUE(exp.get());
  KDL::Vector val1 = exp->value();
  KDL::Vector val2 = KDL::Vector(1.1, 2.2, 3.3);
  EXPECT_TRUE(KDL::Equal(val1, val2));
}
