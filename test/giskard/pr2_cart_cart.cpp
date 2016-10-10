/*
 * Copyright (C) 2015 Georg Bartels <georg.bartels@cs.uni-bremen.de>
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
#include <giskard/giskard.hpp>

class PR2CartCartControlTest : public ::testing::Test
{
  protected:
    typedef Eigen::Matrix< double, 6, 1 > Vector6d;
    Vector6d to_eigen(const KDL::Frame& f)
    {
      Vector6d result;
      result[0] = f.p.x();
      result[1] = f.p.y();
      result[2] = f.p.z();
      f.M.GetEulerZYX(result[3], result[4], result[5]);
      return result;
    }

    virtual void SetUp()
    {
      std::map<std::string, double> q_map;
      q_map["l_elbow_flex_joint"] = -0.150245;
      q_map["l_forearm_roll_joint"] = 2.05374;
      q_map["l_shoulder_lift_joint"] = 1.29519;
      q_map["l_shoulder_pan_joint"] = 1.85677;
      q_map["l_upper_arm_roll_joint"] = 1.49425;
      q_map["l_wrist_flex_joint"] = -0.240708;
      q_map["l_wrist_roll_joint"] = 6.11928;
      q_map["r_elbow_flex_joint"] = -1.64225;
      q_map["r_forearm_roll_joint"] = 5.2686;
      q_map["r_shoulder_lift_joint"] = 0.573401;
      q_map["r_shoulder_pan_joint"] = -0.976377;
      q_map["r_upper_arm_roll_joint"] = -0.954544;
      q_map["r_wrist_flex_joint"] = -1.50746;
      q_map["r_wrist_roll_joint"] = 1.90604;
      q_map["torso_lift_joint"] = 0.300026;
      std::vector<std::string> q_name_list;
      q_name_list.push_back("torso_lift_joint");
      q_name_list.push_back("l_shoulder_pan_joint");
      q_name_list.push_back("l_shoulder_lift_joint");
      q_name_list.push_back("l_upper_arm_roll_joint");
      q_name_list.push_back("l_elbow_flex_joint");
      q_name_list.push_back("l_forearm_roll_joint");
      q_name_list.push_back("l_wrist_flex_joint");
      q_name_list.push_back("l_wrist_roll_joint");
      q_name_list.push_back("r_shoulder_pan_joint");
      q_name_list.push_back("r_shoulder_lift_joint");
      q_name_list.push_back("r_upper_arm_roll_joint");
      q_name_list.push_back("r_elbow_flex_joint");
      q_name_list.push_back("r_forearm_roll_joint");
      q_name_list.push_back("r_wrist_flex_joint");
      q_name_list.push_back("r_wrist_roll_joint");

      KDL::Frame l_arm_goal = KDL::Frame(
//          KDL::Rotation::Quaternion(0.521898465403, 0.515981172047, -0.46705391091, 0.493199823811),
          KDL::Rotation::Quaternion(0.293821000071, 0.27801803703, -0.635756695443, 0.657410552874),
          KDL::Vector(0.0834884427791, 0.505313166742, 0.176484549611));
      KDL::Frame r_arm_goal = KDL::Frame(
          KDL::Rotation::Quaternion(-0.192002947387, 0.00391840785855, 0.718301246071, 0.668702350951),
          KDL::Vector(0.450343470565, -0.297483269345, 0.94000437575));

      q.resize(1 + 7 + 7 + 6 + 6);
      for (size_t i=0; i<q_name_list.size(); ++i)
        q(i) = q_map[q_name_list[i]];
      q.block<6,1>(q_name_list.size(), 0) = to_eigen(l_arm_goal);
      q.block<6,1>(q_name_list.size() + 6, 0) = to_eigen(r_arm_goal);

//      using namespace Eigen;
//      std::cout << q << std::endl;
      nWSR = 1000;
    }

    virtual void TearDown(){}

    Eigen::VectorXd q;
    int nWSR;
};

void print_double(const giskard::QPController& controller, const std::string& name)
{
  std::cout << name << ": " << controller.get_scope().find_double_expression(name)->value() << std::endl;
}

void print_vector(const giskard::QPController& controller, const std::string& name)
{
  using namespace KDL;
  std::cout << name << ": " << controller.get_scope().find_vector_expression(name)->value() << std::endl;
}

void print_rotation(const giskard::QPController& controller, const std::string& name)
{
  double x, y, z, w;
  controller.get_scope().find_rotation_expression(name)->value().GetQuaternion(x, y, z, w);
  std::cout << name << ": " << x << ", " << y << ", " << z << ", " << w << std::endl;
}


TEST_F(PR2CartCartControlTest, SlerpControl)
{
  YAML::Node node = YAML::LoadFile("pr2_cart_cart_control.yaml");
  ASSERT_NO_THROW(node.as<giskard::QPControllerSpec>());

  giskard::QPControllerSpec spec = node.as<giskard::QPControllerSpec>();
  ASSERT_NO_THROW(giskard::generate(spec));

  giskard::QPController controller = giskard::generate(spec);

  ASSERT_NO_THROW(controller.start(q, nWSR));

//  std::vector<std::string> doubles, rotations, vectors;
//  doubles.push_back("l_rot_scaling");
//  doubles.push_back("l_rot_error");
//
//  rotations.push_back("l_rot");
//  rotations.push_back("l_intermediate_goal_rot");
//  rotations.push_back("l_goal_rot");
//
//  vectors.push_back("l_rot_control2");
//
//  for (size_t i=0; i<doubles.size(); ++i)
//    print_double(controller, doubles[i]);
//  std::cout << "a\n";
//  for (size_t i=0; i<rotations.size(); ++i)
//    print_rotation(controller, rotations[i]);
//  for (size_t i=0; i<vectors.size(); ++i)
//    print_vector(controller, vectors[i]);
//  std::cout << "a\n";

  // TODO: finish test-case
}
