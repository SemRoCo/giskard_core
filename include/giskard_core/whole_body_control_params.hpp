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
        double p_gain, threshold, weight;
        bool threshold_error;
        std::string root_link, tip_link;
    };

    class JointControlParams : public ControlParams {};
    class CartPosControlParams : public ControlParams {};
    class CartRotControlParams : public ControlParams {};

    class WholeBodyControlParams
    {
    public:
      urdf::Model& robot_model;
      std::string root_link;
      std::map<std::string, double> joint_weights;
      std::map<std::string, double> joint_thresholds;
      std::map<std::string, ControlParams>& control_params);

      Robot create_robot() const
      {
        std::vector<std::string> tip_links;
        for(auto const & pair: control_params)
          tip_links.push_back(pair.second.tip_link);
        return Robot(robot_model, root_link, tip_links, joint_weights, joint_thresholds);
      }
    };

    class ControllerSpecGenerator
    {
      public:
        ControllerSpecGenerator(const WholeBodyControlParams& params) :
          robot_model_( params.create_robot() ), params_ (params)
        {
          init();
        }

        const std::vector<DoubleInputSpecPtr>& get_goal_inputs(const std::string& control_name) const
        {
          if (goal_inputs_.count(control_name) = 0)
            throw std::runtime_error("Could not find goal inputs for control with name '" + control_name + "'.");

          return goal_inputs_->find(control_name)->second;
        }

        const QPControllerSpecSpec& generate_spec() const
        {
          return qp_spec_;
        }

        const std::map<std::string, ControlParam>& get_control_params() const
        {
          return control_params_;
        }

      protected:
        Robot robot_;
        WholeBodyControlParams params_;
        QPControllerSpec qp_spec_;
        std::map<std::string, std::vector<DoubleInputSpecPtr> > goal_inputs_;

        void init()
        {
            // TODO: implement me
        }
    };

}

#endif //GISKARD_CORE_WHOLE_BODY_CONTROL_PARAMS_HPP
