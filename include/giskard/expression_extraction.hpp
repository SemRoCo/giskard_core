/*
 * Copyright (C) 2015 Jannik Buckelo <jannikbu@cs.uni-bremen.de>
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

#ifndef GISKARD_EXPRESSION_EXTRACTION_HPP
#define GISKARD_EXPRESSION_EXTRACTION_HPP

#include <yaml-cpp/yaml.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <boost/shared_ptr.hpp>

namespace giskard
{

  inline boost::shared_ptr<YAML::Node> get_origin_node(const KDL::Segment& seg)
  {
    boost::shared_ptr<YAML::Node> joint_origin_ptr( new YAML::Node());
    (*joint_origin_ptr)["vector3"].push_back(seg.getFrameToTip().p[0]);
    (*joint_origin_ptr)["vector3"].push_back(seg.getFrameToTip().p[1]);
    (*joint_origin_ptr)["vector3"].push_back(seg.getFrameToTip().p[2]);

    return joint_origin_ptr;
  }

  inline YAML::Node extract_expression(const std::string& start_link, const std::string& end_link, const KDL::Tree& robot_tree)
  {
    KDL::Chain chain;
    if (!robot_tree.getChain(start_link, end_link, chain))
    {
      throw InvalidChain(start_link, end_link);
    }

    std::string var_suffix = "_var";
    std::string frame_suffix = "_frame";
    std::string fk_name = "fk";

    YAML::Node input_vars;
    YAML::Node joint_transforms;
    YAML::Node frame_mul;

    int input_var_index = 0;
    boost::shared_ptr<YAML::Node> null_vector_ptr(new YAML::Node);
    *null_vector_ptr = YAML::Load("{vector3: [0, 0, 0]}");
    std::vector<KDL::Segment> prev_fixed_joints;
    for (std::vector<KDL::Segment>::const_iterator it = chain.segments.begin(); it != chain.segments.end(); ++it)
    {
      KDL::Joint joint = it->getJoint();
      if (joint.getType() == KDL::Joint::None)
      {
        prev_fixed_joints.push_back(*it);
        continue;
      }

      std::string var_name = joint.getName() + var_suffix;
      std::string frame_name = joint.getName() + frame_suffix;

      // Set input variable definition
      YAML::Node input_var;
      input_var[joint.getName() + var_suffix]["input-var"] = input_var_index;
      input_var_index++;
      input_vars.push_back(input_var);

      // Set joint transform
      YAML::Node translation;
      YAML::Node rotation;
      boost::shared_ptr<YAML::Node> joint_origin_ptr;
      if (!KDL::Equal(it->getFrameToTip().p, joint.JointOrigin()))
      {
        joint_origin_ptr = null_vector_ptr;
      }
      else
      {
        joint_origin_ptr = get_origin_node(*it);
      }
      // Add previous fixed joints to origin
      while (!prev_fixed_joints.empty())
      {
        boost::shared_ptr<YAML::Node> add(new YAML::Node);
        (*add)["vector-add"].push_back(*joint_origin_ptr);
        (*add)["vector-add"].push_back(*get_origin_node(prev_fixed_joints.back()));
        joint_origin_ptr = add;
        prev_fixed_joints.pop_back();
      }
      YAML::Node joint_origin = *joint_origin_ptr;
      if (!KDL::Equal(it->getFrameToTip().p, joint.JointOrigin()))
      {
        prev_fixed_joints.push_back(*it);
      }
      // Set joint axis
      YAML::Node joint_axis;
      joint_axis["vector3"].push_back(joint.JointAxis()[0]);
      joint_axis["vector3"].push_back(joint.JointAxis()[1]);
      joint_axis["vector3"].push_back(joint.JointAxis()[2]);
      rotation["axis-angle"].push_back(joint_axis);
      if (joint.getType() == KDL::Joint::TransAxis)
      {
        YAML::Node scale_vec;
        scale_vec["scale-vector"].push_back(var_name);
        scale_vec["scale-vector"].push_back(joint_axis);
        translation["vector-add"].push_back(joint_origin);
        translation["vector-add"].push_back(scale_vec);
        rotation["axis-angle"].push_back(0);
      }
      else if (joint.getType() == KDL::Joint::RotAxis)
      {
        translation = joint_origin;
        rotation["axis-angle"].push_back(var_name);
      }
      // Create frame and add to list
      YAML::Node joint_transform;
      joint_transform[frame_name]["frame"].push_back(rotation);
      joint_transform[frame_name]["frame"].push_back(translation);
      joint_transforms.push_back(joint_transform);

      // Set fk
      frame_mul.push_back(frame_name);
    }

    // Add frame for remaining fixed joints
    if (!prev_fixed_joints.empty())
    {
      boost::shared_ptr<YAML::Node> fixed_joint_origin_ptr = null_vector_ptr;
      while (!prev_fixed_joints.empty())
      {
        boost::shared_ptr<YAML::Node> add(new YAML::Node);
        (*add)["vector-add"].push_back(*fixed_joint_origin_ptr);
        (*add)["vector-add"].push_back(*get_origin_node(prev_fixed_joints.back()));
        fixed_joint_origin_ptr = add;
        prev_fixed_joints.pop_back();
      }
      YAML::Node rotation = YAML::Load("{axis-angle: [{vector3: [1,0,0]}, 0]}");
      YAML::Node joint_transform;
      joint_transform["dummy_frame"]["frame"].push_back(rotation);
      joint_transform["dummy_frame"]["frame"].push_back(*fixed_joint_origin_ptr);
      joint_transforms.push_back(joint_transform);
      frame_mul.push_back("dummy_frame");
    }

    // Merge nodes
    YAML::Node node;
    for (YAML::const_iterator it = input_vars.begin(); it != input_vars.end(); ++it)
    {
      node.push_back(*it);
    }
    for (YAML::const_iterator it = joint_transforms.begin(); it != joint_transforms.end(); ++it)
    {
      node.push_back(*it);
    }
    YAML::Node fk_def;
    fk_def[fk_name]["frame-mul"] = frame_mul;
    node.push_back(fk_def);

    return node;
  }

  inline YAML::Node extract_expression(std::string start_link, std::string end_link, std::string urdf_path)
  {
    urdf::Model urdf;
    if (!urdf.initFile(urdf_path))
    {
      throw InvalidUrdf(urdf_path);
    }
    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(urdf, tree);
    return extract_expression(start_link, end_link, tree);
  }

}

#endif // GISKARD_EXPRESSION_EXTRACTION_HPP
