/*
 * Copyright (C) 2017 Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

using namespace giskard_core;

class QPControllerProjectionTest : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
      ASSERT_TRUE(urdf.initFile("pr2.urdf"));
      root_link = "base_footprint";
      weights = {{Robot::default_joint_weight_key(), 0.001}, {"torso_lift_joint", 0.01}};
      thresholds = {{Robot::default_joint_velocity_key(), 0.5}, {"torso_lift_joint", 0.01}};
    }

    virtual void TearDown(){}

    urdf::Model urdf;
    std::map<std::string, double> weights, thresholds;
    std::string root_link;

};

// 1-DOF Control
TEST_F(QPControllerProjectionTest, TorsoLiftJointControl)
{
  // prepare parameters of QPControllerSpecGenerator
  ControlParams single_joint_params;
  single_joint_params.root_link = "base_link";
  single_joint_params.tip_link = "torso_lift_link";
  single_joint_params.p_gain = 10;
  single_joint_params.threshold_error = false;
  single_joint_params.threshold = 0.2;
  single_joint_params.weight = 1.0;
  single_joint_params.type = ControlParams::ControlType::Joint;
  std::string control_name = "torso_controller";
  QPControllerParams params(urdf, root_link, weights, thresholds, {{control_name, single_joint_params}});

  // build QPControllerSpecGenerator & get controller
  ASSERT_NO_THROW(QPControllerSpecGenerator gen(params));
  QPControllerSpecGenerator gen(params);
  ASSERT_NO_THROW(gen.get_spec());
  QPControllerSpec spec = gen.get_spec();
  ASSERT_NO_THROW(generate(spec));
  QPController controller = generate(spec);

  // prepare parameters of QPControllerProjection
  double period = 0.01;
  size_t min_num_trajectory_points = 50;
  size_t max_num_trajectory_points = 2000;
  int nWSR = 100;
  std::string joint_name = "torso_lift_joint";
  double convergence_threshold = 0.0001;
  std::map<std::string, double> convergence_thresholds = {{joint_name, convergence_threshold}};
  std::vector< std::string > observable_names = gen.get_observable_names();

  QPControllerProjectionParams projection_params(period, observable_names, convergence_thresholds,
      min_num_trajectory_points, max_num_trajectory_points, nWSR);

  // get projection object
  ASSERT_NO_THROW(QPControllerProjection projection(controller, projection_params));
  QPControllerProjection projection(controller, projection_params);

  // checking sane initial state
  EXPECT_EQ(0, projection.get_position_trajectories().size());
  EXPECT_EQ(0, projection.get_velocity_trajectories().size());
  ASSERT_EQ(controller.get_controllable_names().size(), projection.get_controllable_names().size());
  for (size_t i=0; i<controller.get_controllable_names().size(); ++i)
    EXPECT_STREQ(controller.get_controllable_names()[i].c_str(), projection.get_controllable_names()[i].c_str());
  ASSERT_EQ(projection_params.observable_names_.size(), projection.get_observable_names().size());
  for (size_t i=0; i<projection_params.observable_names_.size(); ++i)
    EXPECT_STREQ(projection_params.observable_names_[i].c_str(), projection.get_observable_names()[i].c_str());

  // check that controllable and observable names are OK
  ASSERT_EQ(projection.get_controllable_names().size(), 1);
  EXPECT_STREQ(joint_name.c_str(), projection.get_controllable_names()[0].c_str());
  ASSERT_EQ(projection.get_observable_names().size(), 2);
  EXPECT_STREQ(joint_name.c_str(), projection.get_observable_names()[0].c_str());
  EXPECT_STREQ(create_input_name(control_name, joint_name).c_str(), projection.get_observable_names()[1].c_str());

  // run projection
  double start_config = 0.2;
  double goal_config = 0.05;
  std::map< std::string, double > initial_state =
          {{joint_name, start_config}, {create_input_name(control_name, joint_name), goal_config}};
  ASSERT_NO_THROW(projection.run(initial_state));
  // check trajectory
  EXPECT_EQ(projection.get_position_trajectories().size(), projection.get_velocity_trajectories().size());
  // minimum length OK
  EXPECT_LE(projection_params.min_num_trajectory_points_, projection.get_position_trajectories().size());
  EXPECT_LE(projection_params.min_num_trajectory_points_, projection.get_velocity_trajectories().size());
  // maximum length OK
  EXPECT_GE(projection_params.max_num_trajectory_points_, projection.get_position_trajectories().size());
  EXPECT_GE(projection_params.max_num_trajectory_points_, projection.get_velocity_trajectories().size());
  // initial state OK
  EXPECT_DOUBLE_EQ(0.2, projection.get_position_trajectories()[0](0));
  EXPECT_DOUBLE_EQ(0.0, projection.get_velocity_trajectories()[0](0));
  // end state OK
  EXPECT_LT(std::abs(goal_config - projection.get_position_trajectories().back()(0)), convergence_threshold);
  EXPECT_LT(projection.get_velocity_trajectories().back()(0), convergence_threshold);
  // intermediate states OK
  for (auto const & vel: projection.get_velocity_trajectories())
    EXPECT_LE(std::abs(vel(0)), 0.01);
  for (size_t i=0; i<projection.get_position_trajectories().size() -1; ++i)
    EXPECT_NEAR((projection.get_position_trajectories()[i+1](0) - projection.get_position_trajectories()[i](0))/period,
                projection.get_velocity_trajectories()[i+1](0), KDL::epsilon);

  std::cout << "position trajectory: " << projection.get_position_trajectories().size() << std::endl;
  for (auto const & trajectory_pos: projection.get_position_trajectories())
  {
    using namespace Eigen;
    std::cout << trajectory_pos << ",  ";
  }
  std::cout << "\nvelocity trajectory: " << projection.get_velocity_trajectories().size() << std::endl;
  for (auto const & trajectory_vel: projection.get_velocity_trajectories())
  {
    using namespace Eigen;
    std::cout << trajectory_vel << ", ";
  }
  std::cout << std::endl;
}

// N-DOF Control
TEST_F(QPControllerProjectionTest, RightArmJointControl)
{
  // TODO: implement me
}

