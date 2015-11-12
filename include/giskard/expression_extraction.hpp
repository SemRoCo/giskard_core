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
  class ExpressionExtractor
  {
    public:
      static inline YAML::Node extract(const std::string& start_link, const std::string& end_link, const KDL::Chain& chain)
      {
        std::string var_suffix = "_var";
        std::string frame_suffix = "_frame";
        std::string expression_name = "fk";

        std::vector<YAML::Node> input_vars;
        std::vector<YAML::Node> joint_frames;
        YAML::Node frame_mul;
        YAML::Node axis_angle_null;
        axis_angle_null["axis-angle"].push_back(get_vector3(1, 0, 0));
        axis_angle_null["axis-angle"].push_back(0);

        int input_var_index = 0;
        for (std::vector<KDL::Segment>::const_iterator it = chain.segments.begin(); it != chain.segments.end(); ++it)
        {
          KDL::Joint joint = it->getJoint();

          std::string var_name = joint.getName() + var_suffix;
          std::string frame_name = joint.getName() + frame_suffix;

          YAML::Node joint_frame;
          if (joint.getType() != KDL::Joint::None)
          {
            // Set input variable definition
            YAML::Node input_var;
            input_var[joint.getName() + var_suffix]["input-var"] = input_var_index;
            input_var_index++;
            input_vars.push_back(input_var);

            // Set joint transform
            YAML::Node translation;
            YAML::Node rotation;

            // Set joint axis
            YAML::Node joint_axis = get_vector3(joint.JointAxis());
            if (joint.getType() == KDL::Joint::TransAxis)
            {
              YAML::Node scale_vec;
              scale_vec["scale-vector"].push_back(var_name);
              scale_vec["scale-vector"].push_back(joint_axis);
              translation["vector-add"].push_back(scale_vec);
              translation["vector-add"].push_back(get_vector3(joint.JointOrigin()));
              rotation = axis_angle_null;
            }
            else if (joint.getType() == KDL::Joint::RotAxis)
            {
              translation = get_vector3(joint.JointOrigin());
              rotation["axis-angle"].push_back(joint_axis);
              rotation["axis-angle"].push_back(var_name);
            }

            // Create frame and add to list
            YAML::Node joint_transform;
            joint_transform["frame"].push_back(rotation);
            joint_transform["frame"].push_back(translation);
            joint_frame[frame_name]["frame-mul"].push_back(joint_transform);
          }

          std::vector<YAML::Node> f_tip_nodes = get_frame_tip_nodes(*it);
          for (std::vector<YAML::Node>::iterator f_tip_it = f_tip_nodes.begin(); f_tip_it != f_tip_nodes.end(); ++f_tip_it)
          {
            joint_frame[frame_name]["frame-mul"].push_back(*f_tip_it);
          }

          if (joint_frame.size() > 0)
          {
            joint_frames.push_back(joint_frame);
            frame_mul.push_back(frame_name);
          }
        }

        // Merge nodes
        YAML::Node node;
        for (std::vector<YAML::Node>::iterator it = input_vars.begin(); it != input_vars.end(); ++it)
        {
          node.push_back(*it);
        }
        for (std::vector<YAML::Node>::iterator it = joint_frames.begin(); it != joint_frames.end(); ++it)
        {
          node.push_back(*it);
        }
        YAML::Node fk_def;
        fk_def[expression_name]["frame-mul"] = frame_mul;
        node.push_back(fk_def);

        return node;
      }

      static inline YAML::Node extract(const std::string& start_link, const std::string& end_link, const KDL::Tree& robot_tree)
      {
        KDL::Chain chain;
        if (!robot_tree.getChain(start_link, end_link, chain))
        {
          throw InvalidChain(start_link, end_link);
        }
        return extract(start_link, end_link, chain);
      }

      static inline YAML::Node extract(const std::string& start_link, const std::string& end_link, const std::string& urdf_path)
      {
        urdf::Model urdf;
        if (!urdf.initFile(urdf_path))
        {
          throw InvalidUrdf(urdf_path);
        }
        KDL::Tree tree;
        kdl_parser::treeFromUrdfModel(urdf, tree);
        return extract(start_link, end_link, tree);
      }

    private:
      static inline std::vector<YAML::Node> get_frame_tip_nodes(const KDL::Segment& seg)
      {
        std::vector<YAML::Node> frames;
        KDL::Frame frame = seg.getFrameToTip() * seg.getJoint().pose(0).Inverse();

        // Set xyz
        if(!KDL::Equal(frame.p, KDL::Vector::Zero()))
        {
          YAML::Node xyz_frame;
          YAML::Node axis_angle_null;
          axis_angle_null["axis-angle"].push_back(get_vector3(1, 0, 0));
          axis_angle_null["axis-angle"].push_back(0);
          xyz_frame["frame"].push_back(axis_angle_null);
          xyz_frame["frame"].push_back(get_vector3(frame.p));
          frames.push_back(xyz_frame);
        }

        // Set rpy
        if(!KDL::Equal(frame.M, KDL::Rotation::Identity()))
        {
          std::vector<double> rpy(3);
          frame.M.GetRPY(rpy[0],rpy[1],rpy[2]);
          for (std::vector<double>::size_type i = rpy.size() - 1;
                i != (std::vector<double>::size_type) -1; i--)
          {
            if(rpy[i] != 0)
            {
              YAML::Node rpy_frame;
              YAML::Node axis_angle;
              KDL::Vector axis;
              axis[i] = 1;
              axis_angle["axis-angle"].push_back(get_vector3(axis));
              axis_angle["axis-angle"].push_back(rpy[i]);
              rpy_frame["frame"].push_back(axis_angle);
              rpy_frame["frame"].push_back(get_vector3(0, 0, 0));
              frames.push_back(rpy_frame);
            }
          }
        }

        return frames;
      }

      static inline YAML::Node get_vector3(const KDL::Vector& p)
      {
        return get_vector3(p[0], p[1], p[2]);
      }

      static inline YAML::Node get_vector3(const double x, const double y, const double z)
      {
        YAML::Node vec3;
        vec3["vector3"].push_back(x);
        vec3["vector3"].push_back(y);
        vec3["vector3"].push_back(z);
        return vec3;
      }
  };

  static YAML::Node extract_expression(const std::string& start_link, const std::string& end_link, const KDL::Chain& chain)
  {
    return ExpressionExtractor::extract(start_link, end_link, chain);
  }

  static YAML::Node extract_expression(const std::string& start_link, const std::string& end_link, const KDL::Tree& robot_tree)
  {
    return ExpressionExtractor::extract(start_link, end_link, robot_tree);
  }

  static YAML::Node extract_expression(const std::string& start_link, const std::string& end_link, const std::string& urdf_path)
  {
    return ExpressionExtractor::extract(start_link, end_link, urdf_path);
  }
}

#endif // GISKARD_EXPRESSION_EXTRACTION_HPP
