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

namespace giskard
{
  inline void add_input_varibale_definitions(const KDL::Chain& chain, YAML::Node& node)
  {
    for (int i = 0; i < chain.segments.size(); i++)
    {
      YAML::Node var_name;
      YAML::Node var_input;
      var_input["input-var"] = i;
      var_name[chain.segments[i].getJoint().getName() + "_var"] = var_input;
      node.push_back(var_name);
    }
  }

  inline void get_axis_angle(const KDL::Joint& joint, YAML::Node& node)
  {
    YAML::Node axis_angle_vec3;
    for(int i = 0; i < 3; i++)
    {
      axis_angle_vec3["vector3"].push_back(joint.JointAxis()[i]);
    }
    node["axis-angle"].push_back(axis_angle_vec3);

    // if it is a revolute joint reference the input variable
    if (joint.getType() == KDL::Joint::RotAxis)
      node["axis-angle"].push_back(joint.getName() + "_var");
    else
      node["axis-angle"].push_back(0);
    // TODO: remove me
    // node["axis_angle"].push_back(joint.getTypeName());
  }

  inline void get_translation(const KDL::Joint& joint, YAML::Node& node)
  {
    // if it is a prismatic joint add an offset to the origin
    if (joint.getType() == KDL::Joint::TransAxis)
    {
      for(int i = 0; i < 3; i++)
      {
        if (joint.JointAxis()[i] == 1)
        {
          YAML::Node add;
          add["double-add"].push_back(joint.JointOrigin()[i]);
          add["double-add"].push_back(joint.getName() + "_var");
          node["vector3"].push_back(add);
        }
        else
          node["vector3"].push_back(joint.JointOrigin()[i]);
      }
    }
    else
    {
      for(int i = 0; i < 3; i++)
      {
        node["vector3"].push_back(joint.JointOrigin()[i]);
      }
    }
  }

  inline void add_joint_transform_definitions(const KDL::Chain& chain, YAML::Node& node)
  {
    for (int i = 0; i < chain.segments.size(); i++)
    {
      YAML::Node frame_name;
      YAML::Node frame;
      YAML::Node axis_angle;
      YAML::Node translation;
      get_axis_angle(chain.segments[i].getJoint(), axis_angle);
      get_translation(chain.segments[i].getJoint(), translation);
      frame["frame"].push_back(axis_angle);
      frame["frame"].push_back(translation);
      frame_name[chain.segments[i].getJoint().getName() + "_frame"] = frame;

      node.push_back(frame_name);
    }
  }

  inline void add_fk_definition(const KDL::Chain& chain, YAML::Node& node)
  {
    YAML::Node fk_name;
    YAML::Node frame_mul;
    for (int i = 0; i < chain.segments.size(); i++)
    {
      frame_mul["frame-mul"].push_back(chain.segments[i].getJoint().getName() + "_frame");
    }
    fk_name["fk"] = frame_mul;
    node.push_back(fk_name);
  }

  inline bool get_scope_yaml(std::string start_link, std::string end_link, const KDL::Tree& robot_tree, YAML::Node& node)
  {
    KDL::Chain chain;
    if (!robot_tree.getChain(start_link, end_link, chain))
    {
      std::cout << "Couldn't get chain: " << start_link << " -> " << end_link << std::endl;
      return false;
    }

    add_input_varibale_definitions(chain, node);
    add_joint_transform_definitions(chain, node);
    add_fk_definition(chain, node);

    return true;
  }

  inline bool extract_expression(std::string start_link, std::string end_link, std::string urdf_path, YAML::Node& node)
  {
    urdf::Model urdf;
    urdf.initFile(urdf_path);
    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(urdf, tree);
    return get_scope_yaml(start_link, end_link, tree, node);
  }

}

#endif // GISKARD_EXPRESSION_EXTRACTION_HPP
