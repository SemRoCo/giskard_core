#include <gtest/gtest.h>
#include <giskard/giskard.hpp>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

class PR2FKTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      urdf::Model urdf;
      urdf.initFile("pr2.urdf"); tree;
      assert(kdl_parser::treeFromUrdfModel(urdf, tree));
   }

    virtual void TearDown(){}

    KDL::Tree tree;
};

TEST_F(PR2FKTest, SingleExpression)
{
  YAML::Node node = YAML::LoadFile("pr2_left_arm_single_expression.yaml");
  ASSERT_NO_THROW(node.as<giskard::FrameSpecPtr>());

  giskard::FrameSpecPtr spec = node.as<giskard::FrameSpecPtr>();
  ASSERT_NO_THROW(spec->get_expression(giskard::Scope()));
  
  KDL::Expression<KDL::Frame>::Ptr exp = spec->get_expression(giskard::Scope());
  ASSERT_TRUE(exp.get()); 

  std::string base = "torso_lift_link";
  std::string tip = "l_wrist_roll_link";
  KDL::Chain chain;
  ASSERT_TRUE(tree.getChain(base, tip, chain));
  ASSERT_EQ(chain.getNrOfJoints(), exp->number_of_derivatives());

  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  fk_solver = boost::shared_ptr<KDL::ChainFkSolverPos_recursive>(
      new KDL::ChainFkSolverPos_recursive(chain));

  for(int i=-11; i<12; ++i)
  {
    std::vector<double> exp_in;
    KDL::JntArray solver_in(exp->number_of_derivatives());
    for(size_t j=0; j<exp->number_of_derivatives(); ++j)
    {
      double value = 0.1*i;
      exp_in.push_back(value);
      solver_in(j) = value;
    }

    exp->setInputValues(exp_in);
    KDL::Frame exp_frame = exp->value();

    KDL::Frame solver_frame;
    ASSERT_GE(fk_solver->JntToCart(solver_in, solver_frame), 0);

    EXPECT_TRUE(KDL::Equal(exp_frame, solver_frame));
  }
}

TEST_F(PR2FKTest, ExpressionMap)
{
  YAML::Node node = YAML::LoadFile("pr2_left_arm_expression_map.yaml");

  ASSERT_NO_THROW(node.as< giskard::ScopeSpec >());
  giskard::ScopeSpec scope_spec = node.as<giskard::ScopeSpec>();
  
  ASSERT_NO_THROW(giskard::generate(scope_spec));
  giskard::Scope scope = giskard::generate(scope_spec);

  ASSERT_TRUE(scope.has_frame_expression("pr2_fk"));

  KDL::Expression<KDL::Frame>::Ptr exp = scope.find_frame_expression("pr2_fk");
  ASSERT_TRUE(exp.get()); 

  std::string base = "base_link";
  std::string tip = "l_wrist_roll_link";
  KDL::Chain chain;
  ASSERT_TRUE(tree.getChain(base, tip, chain));
  ASSERT_EQ(chain.getNrOfJoints(), exp->number_of_derivatives());

  boost::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
  fk_solver = boost::shared_ptr<KDL::ChainFkSolverPos_recursive>(
      new KDL::ChainFkSolverPos_recursive(chain));

  for(int i=-11; i<12; ++i)
  {
    std::vector<double> exp_in;
    KDL::JntArray solver_in(exp->number_of_derivatives());
    for(size_t j=0; j<exp->number_of_derivatives(); ++j)
    {
      double value = 0.1*i;
      exp_in.push_back(value);
      solver_in(j) = value;
    }

    exp->setInputValues(exp_in);
    KDL::Frame exp_frame = exp->value();

    KDL::Frame solver_frame;
    ASSERT_GE(fk_solver->JntToCart(solver_in, solver_frame), 0);

    EXPECT_TRUE(KDL::Equal(exp_frame, solver_frame));
  }

}
