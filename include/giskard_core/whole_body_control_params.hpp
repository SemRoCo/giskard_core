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
#ifndef GISKARD_CORE_WHOLE_BODY_CONTROL_PARAMS_HPP
#define GISKARD_CORE_WHOLE_BODY_CONTROL_PARAMS_HPP

#include <giskard_core/robot.hpp>

namespace giskard_core
{
    class ControlParams
    {
      public:
        // TODO: add name?
        double p_gain, threshold, weight;
        bool threshold_error;
        std::string root_link, tip_link;
        // TODO: add reference frame
        enum ControlType {UNKNOWN, JOINT, CARTPOS, CARTROT};
        ControlType type;
    };

    class WholeBodyControlParams
    {
      public:
        WholeBodyControlParams(const urdf::Model& robot_model, const std::string& root_link,
            const std::map<std::string, double>& joint_weights, const std::map<std::string, double>& joint_thresholds,
            const std::map<std::string, ControlParams>& control_params) :
          robot_model_(robot_model), root_link_(root_link), joint_weights_(joint_weights),
          joint_thresholds_(joint_thresholds), control_params_(control_params) {}

        urdf::Model robot_model_;
        std::string root_link_;
        std::map<std::string, double> joint_weights_;
        std::map<std::string, double> joint_thresholds_;
        std::map<std::string, ControlParams> control_params_;

        Robot create_robot() const
        {
          std::vector<std::string> tip_links;
          for(auto const & pair: control_params_)
            tip_links.push_back(pair.second.tip_link);
          return Robot(robot_model_, root_link_, tip_links, joint_weights_, joint_thresholds_);
        }
    };

    class ControllerSpecGenerator
    {
      public:
        ControllerSpecGenerator(const WholeBodyControlParams& params) :
          robot_( params.create_robot() ), params_( params )
        {
          init();
        }

        const std::vector<DoubleInputSpecPtr>& get_goal_inputs(const std::string& control_name) const
        {
          if (goal_inputs_.count(control_name) == 0)
            throw std::runtime_error("Could not find goal inputs for control with name '" + control_name + "'.");

          return goal_inputs_.find(control_name)->second;
        }

        const QPControllerSpec& get_spec() const
        {
          return qp_spec_;
        }

        const WholeBodyControlParams& get_control_params() const
        {
          return params_;
        }

      protected:
        Robot robot_;
        WholeBodyControlParams params_;
        QPControllerSpec qp_spec_;
        // TODO: refactor this into map<string, map<string, DoubleInputSpecPtr> > by providing internal function to map to unique names
        std::map<std::string, std::vector<DoubleInputSpecPtr> > goal_inputs_;

        void init()
        {
          qp_spec_.scope_.clear();
          init_goal_inputs();
          init_soft_constraints();
          qp_spec_.controllable_constraints_ = robot_.get_controllable_constraints();
          qp_spec_.hard_constraints_ = robot_.get_hard_constraints();
        }

        void init_goal_inputs()
        {
          for (auto const & control : params_.control_params_)
            goal_inputs_.insert(std::make_pair(control.first,
                get_goal_inputs(control.second, robot_.get_number_of_joints() + goal_inputs_.size())));
        }

        // TODO: turns this into a map
        std::vector<DoubleInputSpecPtr> get_goal_inputs(const ControlParams& params, size_t start_index) const
        {
          std::vector<DoubleInputSpecPtr> result;
          switch(params.type)
          {
            case ControlParams::ControlType::JOINT:
              for (auto const & joint_name : robot_.chain_joint_names(params.root_link, params.tip_link))
                // TODO: use name of controller
                result.push_back(input(start_index));
              break;
            //TODO: complete me
            default:
              throw std::runtime_error("Asked to create inputs for unsupported control type");
          }

          return result;
        }

        void init_soft_constraints()
        {
          for (auto const & control : params_.control_params_)
          {
            std::vector<SoftConstraintSpec> new_soft_constraints = get_soft_constraints(control);
            qp_spec_.soft_constraints_.insert(qp_spec_.soft_constraints_.end(),
                new_soft_constraints.begin(), new_soft_constraints.end());
          }
        }

        std::vector<SoftConstraintSpec> get_soft_constraints(const std::pair<std::string, ControlParams>& params) const
        {
          std::vector<SoftConstraintSpec> result;
          switch (params.second.type)
          {
            case ControlParams::ControlType::JOINT:
              for (auto const & joint_name: robot_.chain_joint_names(params.second.root_link, params.second.tip_link))
              {
                  std::cout << "TODO\n";
              }
          }

            // TODO: complete me
            return {};
        }
    };

}

#endif //GISKARD_CORE_WHOLE_BODY_CONTROL_PARAMS_HPP
