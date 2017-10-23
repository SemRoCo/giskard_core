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

#ifndef GISKARD_CORE_ROBOT_HPP
#define GISKARD_CORE_ROBOT_HPP

#include <urdf_model/model.h>
#include <giskard_core/specifications.hpp>
#include <set>
#include <algorithm>


namespace giskard_core
{
  inline VectorSpecPtr to_vector_spec(const urdf::Vector3& vector)
  {
    return vector_constructor_spec(
        double_const_spec(vector.x), double_const_spec(vector.y), double_const_spec(vector.z));
  }

  inline RotationSpecPtr to_quaternion_spec(const urdf::Rotation& rot)
  {
    return quaternion_spec(rot.x, rot.y, rot.z, rot.w);
  }

  inline FrameSpecPtr to_frame_spec(const urdf::Pose& pose)
  {
    return frame_constructor_spec(to_vector_spec(pose.position), to_quaternion_spec(pose.rotation));
  }

  class Robot
  {
    public:
      Robot(const urdf::Model& robot_model, const std::string& root_link,
          const std::vector<std::string>& tip_links) :
        robot_model_( robot_model ), root_link_( root_link )
      {
        robot_model_.initTree(parent_link_tree_);

        for (size_t i=0; i<tip_links.size(); ++i)
          init_kinematic_chain(root_link, tip_links[i]);
      }

      FrameSpecPtr get_fk_spec(const std::string& root_link, const std::string& tip_link) const
      {
        if (root_link_.compare(root_link) != 0)
          throw std::runtime_error("Expected root link '" + root_link_ + "', instead got '" + root_link + "'.");

        if (fk_map_.find(tip_link) == fk_map_.end())
          throw std::runtime_error("Could not find fk specification for tip link '" + tip_link + "'.");

        return fk_map_.find(tip_link)->second;
      }

      const std::string& get_root_link() const
      {
        return root_link_;
      }

      std::vector<ControllableConstraintSpecPtr> get_controllable_constraints() const
      {
        // TODO: implement me
        return std::vector<ControllableConstraintSpecPtr>();
      }

      std::vector<HardConstraintSpecPtr> get_hard_constraints() const
      {
        // TODO: implement me
        return std::vector<HardConstraintSpecPtr>();
      }

      std::vector<ScopeEntry> get_scope() const
      {
        std::vector<ScopeEntry> scope;
        
        for (std::map<std::string, FrameSpecPtr>::const_iterator it=fk_map_.begin(); it!=fk_map_.end(); ++it)
          scope.push_back(ScopeEntry(it->first, it->second));

        return scope;
      }

      DoubleInputSpecPtr get_joint(const std::string& joint_name) const
      {
        if (joint_map_.find(joint_name) == joint_map_.end())
          throw std::runtime_error("Could not find joint with name '" + joint_name + "'.");

        return joint_map_.find(joint_name)->second;
      }

      size_t get_number_of_joints() const
      {
        return joint_map_.size();
      }

    protected:
      urdf::Model robot_model_;
      std::map<std::string, std::string> parent_link_tree_;
      std::map<std::string, FrameSpecPtr> fk_map_;
      std::map<std::string, giskard_core::DoubleInputSpecPtr> joint_map_;
      std::string root_link_;

      void init_kinematic_chain(const std::string& root, const std::string& tip)
      {
        std::vector<std::string> moveable_joints_names = chain_joint_names(root, tip, false);
        std::vector<std::string> all_joints_names = chain_joint_names(root, tip, true);

        // create and add input expressions for new moveable joints
        for (size_t i=0; i<moveable_joints_names.size(); ++i)
          if (joint_map_.find(moveable_joints_names[i]) == joint_map_.end())
            joint_map_.insert(std::pair<std::string, DoubleInputSpecPtr>(moveable_joints_names[i], input(joint_map_.size())));

        // create and add frame expression for new kinematic chain
        std::vector<FrameSpecPtr> joint_transforms = { frame_constructor_spec() };
        for (size_t i=0; i<all_joints_names.size(); ++i)
        {
          if ( robot_model_.joints_.find(all_joints_names[i]) == robot_model_.joints_.end() )
            throw std::runtime_error("Could not find joint with name '" + all_joints_names[i] + "'.");

          std::vector<FrameSpecPtr> new_transforms = extract_joint_transforms(
              robot_model_.joints_.find(all_joints_names[i])->second);

          joint_transforms.insert(joint_transforms.end(), new_transforms.begin(), new_transforms.end());
        }

        fk_map_.insert(std::pair<std::string, FrameSpecPtr>(tip, frame_multiplication_spec(joint_transforms)));

        // TODO: fill controllable constraints
        // TODO: fill hard constraints
      }

      std::vector<std::string> chain_joint_names(const std::string& root, const std::string& tip, bool add_fixed_joints=true)
      {
        std::vector<std::string> chain_joints;

        std::string current_link_name = tip;
        while (current_link_name.compare(root) != 0)
        {
          if (robot_model_.links_.find(current_link_name) == robot_model_.links_.end() )
            throw std::runtime_error("Could not find link with name '" + current_link_name + "'.");

          if (!robot_model_.links_.find(current_link_name)->second)
            throw std::runtime_error("Link associated with name '" + current_link_name + "' is an empty shared_ptr.");

          urdf::JointSharedPtr parent_joint =
            robot_model_.links_.find(current_link_name)->second->parent_joint;

          if (!parent_joint)
            throw std::runtime_error("Parent joint of link with name '" + current_link_name + "' is an empty shared_ptr.");

          if (add_fixed_joints || (parent_joint->type != urdf::Joint::FIXED))
            chain_joints.push_back(parent_joint->name);

          if (parent_link_tree_.find(current_link_name) == parent_link_tree_.end())
            throw std::runtime_error("Could not find parent link of link with name '" + current_link_name + "'.");

          current_link_name = parent_link_tree_.find(current_link_name)->second;
        }

        std::reverse(chain_joints.begin(), chain_joints.end());
        return chain_joints;
      }

      std::vector<FrameSpecPtr> extract_joint_transforms(urdf::JointSharedPtr& joint) 
      {
        if ( !joint )
          throw std::runtime_error("Given joint is an empty shared_ptr.");

        std::vector<FrameSpecPtr> frame_specs;
        
        // translate fixed origin pose
        frame_specs.push_back(to_frame_spec(joint->parent_to_joint_origin_transform));
        
        // translate actual joint transform
        switch( joint->type ) 
        {
          case urdf::Joint::FIXED:
            // no-op
            break;
          case urdf::Joint::PRISMATIC:
            frame_specs.push_back(frame_constructor_spec(
                vector_double_mul(to_vector_spec(joint->axis), get_joint(joint->name))));
            break;
          case urdf::Joint::REVOLUTE: case urdf::Joint::CONTINUOUS:
            frame_specs.push_back(frame_constructor_spec(
                  vector_constructor_spec(),
                  axis_angle_spec(to_vector_spec(joint->axis), get_joint(joint->name))));
            break;
          case urdf::Joint::PLANAR: case urdf::Joint::FLOATING: case urdf::Joint::UNKNOWN:
            throw std::runtime_error("Joint with name '" + joint->name + "' has unsupported type.");
            break;
          default:
            throw std::runtime_error("Joint with name '" + joint->name + "' has unmodeled type.");
            break;
        }

        return frame_specs;
      }
  };
}

#endif // GISKARD_CORE_ROBOT_HPP
