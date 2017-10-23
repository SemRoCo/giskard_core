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
  class Robot
  {
    public:
      Robot(const urdf::Model& robot_model, const std::string& root_link,
          const std::vector<std::string>& tip_links) :
        robot_model_( robot_model )
      {
        robot_model_.initTree(parent_link_tree_);

        for (size_t i=0; i<tip_links.size(); ++i)
          init_kinematic_chain(root_link, tip_links[i]);
      }

      FrameSpecPtr get_kinematic_chain(const std::string& root, const std::string& tip) const
      {
        // TODO: implement me
        return frame_constructor_spec();
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
        return scope_;
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
      // TODO: add some useful members
      urdf::Model robot_model_;
      std::map<std::string, std::string> parent_link_tree_;
      std::vector<ScopeEntry> scope_;
      std::map<std::string, giskard_core::DoubleInputSpecPtr> joint_map_;

      void init_kinematic_chain(const std::string& root, const std::string& tip)
      {
        std::vector<std::string> joint_names = chain_joint_names(root, tip);

        // create and add input expressions for new moveable joints
        for (size_t i=0; i<joint_names.size(); ++i)
          if (joint_map_.find(joint_names[i]) == joint_map_.end())
          {
            if (robot_model_.joints_.find(joint_names[i]) == robot_model_.joints_.end())
              throw std::runtime_error("Could not find joint with name '" + joint_names[i] + "'.");

            if (!robot_model_.joints_.find(joint_names[i])->second)
              throw std::runtime_error("Joint with name '" + joint_names[i] + "' is an empty shared_ptr.");

            if (robot_model_.joints_.find(joint_names[i])->second->type != urdf::Joint::FIXED)
              joint_map_.insert(std::pair<std::string, DoubleInputSpecPtr>(joint_names[i], input(joint_map_.size())));
          }

        ScopeEntry new_entry;
        new_entry.name = tip;
        // TODO: replace me with actual FK expression
        new_entry.spec = frame_constructor_spec();
        scope_.push_back(new_entry);

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
  };
}

#endif // GISKARD_CORE_ROBOT_HPP
