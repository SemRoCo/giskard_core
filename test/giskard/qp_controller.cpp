#include <gtest/gtest.h>
#include <giskard/giskard.hpp>

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
    Eigen::VectorXd initial_state;

    double mu;
    int nWSR;
};

TEST_F(QPControllerTest, Constructor)
{
  giskard::QPController c;

  EXPECT_EQ(0, c.get_command().rows());
}

TEST_F(QPControllerTest, Init)
{
   giskard::QPController c;

   ASSERT_TRUE(c.init(controllable_lower, controllable_upper, controllable_weights, soft_expressions,
      soft_lower, soft_upper, soft_weights, hard_expressions, hard_lower, hard_upper));

   ASSERT_TRUE(c.start(initial_state, nWSR));

   EXPECT_EQ(2, c.get_command().rows());
}

TEST_F(QPControllerTest, Update)
{
   // setup controller
   giskard::QPController c;
   ASSERT_TRUE(c.init(controllable_lower, controllable_upper, controllable_weights, soft_expressions,
      soft_lower, soft_upper, soft_weights, hard_expressions, hard_lower, hard_upper));
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
