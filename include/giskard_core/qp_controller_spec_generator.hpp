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
#ifndef GISKARD_CORE_QP_CONTROLLER_SPEC_GENERATOR_HPP
#define GISKARD_CORE_QP_CONTROLLER_SPEC_GENERATOR_HPP

#include <giskard_core/robot.hpp>

namespace giskard_core
{
    class ControlParams
    {
      public:
        double p_gain, threshold, weight;
        bool threshold_error;
        std::string root_link, tip_link;
        // TODO: add reference frame
        enum ControlType {Unknown, Joint, Translation3D, Rotation3D};
        ControlType type;
    };

    class QPControllerParams
    {
      public:
        QPControllerParams(const urdf::Model& robot_model, const std::string& root_link,
            const std::map<std::string, double>& joint_weights, const std::map<std::string, double>& joint_thresholds,
            const std::map<std::string, ControlParams>& control_params) :
          robot_(robot_model, root_link, get_tip_links(control_params), joint_weights, joint_thresholds),
          control_params_(control_params)
        {}

        Robot robot_;
        std::map<std::string, ControlParams> control_params_;

      protected:
        static std::vector<std::string> get_tip_links(const std::map<std::string, ControlParams>& control_params)
        {
          std::vector<std::string> tip_links;
          for(auto const & pair: control_params)
            tip_links.push_back(pair.second.tip_link);
          return tip_links;
        }
    };

    class QPControllerSpecGenerator
    {
      public:
        QPControllerSpecGenerator(const QPControllerParams& params) :
          params_( params )
        {
          init();
        }

        const std::map<std::string, DoubleInputSpecPtr>& get_goal_inputs(const std::string& control_name) const
        {
          if (goal_inputs_.count(control_name) == 0)
            throw std::runtime_error("Could not find goal inputs for control with name '" + control_name + "'.");

          return goal_inputs_.find(control_name)->second;
        }

        const DoubleInputSpecPtr& get_goal_input(const std::string& control_name, const std::string& dim_name) const
        {
          const std::map<std::string, DoubleInputSpecPtr> goal_inputs = get_goal_inputs(control_name);

          if (goal_inputs.count(dim_name) == 0)
            throw std::runtime_error("Could not find goal input for dimension '" + dim_name + "' of control '" + control_name + "'.");

          return goal_inputs.find(dim_name)->second;
        }

        const QPControllerSpec& get_spec() const
        {
          return qp_spec_;
        }

        const QPControllerParams& get_control_params() const
        {
          return params_;
        }

        std::vector<std::string> get_controllable_names() const
        {
          std::vector<std::string> result;

          for (auto const & controllable_constraint: get_spec().controllable_constraints_)
          {
            result.push_back(controllable_constraint.name_);
          }

          return result;
        }

        std::vector<std::string> get_observable_names() const
        {
          // reserve space
          std::vector<std::string> observable_names(get_spec().controllable_constraints_.size() + num_goal_inputs());

          // copy over controllable names
          for (size_t i=0; i<get_spec().controllable_constraints_.size(); ++i)
            observable_names[i] = get_spec().controllable_constraints_[i].name_;

          // copy over input names
          for (auto const & control_goal_inputs: goal_inputs_)
            for (auto const& goal_input: control_goal_inputs.second)
            {
//               std::cout << "Adding observable '" << create_input_name(control_goal_inputs.first, goal_input.first) <<
//                         "' at index " << goal_input.second->get_input_num() << std::endl;
               observable_names[goal_input.second->get_input_num()] =
                  create_input_name(control_goal_inputs.first, goal_input.first);

            }

          return observable_names;
        }

        size_t num_controllables() const
        {
          return qp_spec_.controllable_constraints_.size();
        }

        size_t num_goal_inputs() const
        {
          size_t num_goal_inputs = 0;
          for (auto const & goal_input: goal_inputs_)
            num_goal_inputs += goal_input.second.size();

          return num_goal_inputs;
        }

        size_t num_observables() const
        {
          return num_controllables() + num_goal_inputs();
        }

        static double pi()
        {
          static double result = 3.14159265359;
          return result;
        }

        static std::vector<std::string> rotation3d_names()
        {
          static std::vector<std::string> result = {"axis_x", "axis_y", "axis_z", "angle"};
          return result;
        }

        static std::vector<std::string> translation3d_names()
        {
          static std::vector<std::string> result = {"x", "y", "z"};
          return result;
        }

        static std::string create_input_name(const std::string& controller_name, const std::string& dimension_name)
        {
          return controller_name + "_goal_" + dimension_name;
        }

        static std::vector<std::string> create_input_names(const std::string& controller_name, const ControlParams::ControlType& type)
        {
            std::vector<std::string> result;
            switch (type)
            {
                case ControlParams::ControlType::Joint:
                  throw std::runtime_error("Function create_input_names does not support ControlType Joint.");
                case ControlParams::ControlType::Unknown:
                  throw std::runtime_error("Function create_input_names does not support ControlType Unknown.");
                case ControlParams::ControlType::Translation3D:
                {
                  for (auto const & trans_name: translation3d_names())
                    result.push_back(create_input_name(controller_name, trans_name));
                  break;
                }
                case ControlParams::ControlType::Rotation3D:
                {
                  for (auto const & rot_name: rotation3d_names())
                    result.push_back(create_input_name(controller_name, rot_name));
                  break;
                }
                default:
                  throw std::runtime_error("Function create_input_names ran into unsupported ControlType.");
            }
            return result;
        };

      protected:
        QPControllerParams params_;
        QPControllerSpec qp_spec_;
        std::map<std::string, std::map<std::string, DoubleInputSpecPtr> > goal_inputs_;

        void init()
        {
          qp_spec_.scope_.clear();
          qp_spec_.controllable_constraints_ = params_.robot_.get_controllable_constraints();
          qp_spec_.hard_constraints_ = params_.robot_.get_hard_constraints();
          init_goal_inputs();
          init_soft_constraints();
        }

        void init_goal_inputs()
        {
          for (auto const & control : params_.control_params_)
            goal_inputs_.insert(std::make_pair(control.first,
//                create_goal_inputs(control.second, params_.robot_.get_number_of_joints() + goal_inputs_.size())));
                create_goal_inputs(control.second, num_observables())));
        }

        std::map<std::string, DoubleInputSpecPtr> create_goal_inputs(const ControlParams& params, size_t start_index) const
        {
          std::map<std::string, DoubleInputSpecPtr> result;
          switch(params.type)
          {
            case ControlParams::ControlType::Joint:
              for (auto const & joint_name : params_.robot_.chain_joint_names(params.root_link, params.tip_link, false))
                result.insert(std::make_pair(joint_name, input(start_index++)));
              break;
            case ControlParams::ControlType::Translation3D:
              for (auto const & translation_name : QPControllerSpecGenerator::translation3d_names())
                result.insert(std::make_pair(translation_name, input(start_index++)));
              break;
            case ControlParams::ControlType::Rotation3D:
              for (auto const & rotation_name : QPControllerSpecGenerator::rotation3d_names())
                result.insert(std::make_pair(rotation_name, input(start_index++)));
              break;
            //TODO: complete me for other cases
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
          std::vector<SoftConstraintSpec> specs;
          switch (params.second.type)
          {
            case ControlParams::ControlType::Joint:
            {
              std::set<std::string> continuous_joints_names =
                  params_.robot_.continuous_joints_names(params.second.root_link, params.second.tip_link);
              for (auto const &joint_name: params_.robot_.chain_joint_names(params.second.root_link, params.second.tip_link, false))
              {
                SoftConstraintSpec spec;
                DoubleInputSpecPtr goal_spec = get_goal_inputs(params.first).find(joint_name)->second;
                DoubleInputSpecPtr joint_spec = params_.robot_.get_joint(joint_name);
                spec.lower_ = joint_control_spec(goal_spec, joint_spec, params.second,
                                           continuous_joints_names.find(joint_name) !=
                                           continuous_joints_names.end());
                spec.upper_ = spec.lower_;
                spec.expression_ = joint_spec;
                spec.weight_ = double_const_spec(params.second.weight);
                spec.name_ = create_input_name(params.first, joint_name);
                specs.push_back(spec);
              }
              break;
            }
            case ControlParams::ControlType::Translation3D:
            {
              VectorSpecPtr goal_spec = vector_constructor_spec(
                  get_goal_inputs(params.first).find(translation3d_names()[0])->second,
                  get_goal_inputs(params.first).find(translation3d_names()[1])->second,
                  get_goal_inputs(params.first).find(translation3d_names()[2])->second);
              VectorSpecPtr fk_spec = origin(params_.robot_.get_fk_spec(params.second.root_link, params.second.tip_link));
              VectorSpecPtr control_spec = translation3d_control_spec(goal_spec, fk_spec, params.second);
              for (auto const & translation_name: QPControllerSpecGenerator::translation3d_names())
              {
                SoftConstraintSpec spec;
                spec.name_ = create_input_name(params.first, translation_name);
                spec.weight_ = double_const_spec(params.second.weight);
                if (translation_name.compare(translation3d_names()[0]) == 0)
                {
                  spec.expression_ = x_coord(fk_spec);
                  spec.lower_ = x_coord(control_spec);
                  spec.upper_ = spec.lower_;
                }
                else if (translation_name.compare(translation3d_names()[1]) == 0)
                {
                  spec.expression_ = y_coord(fk_spec);
                  spec.lower_ = y_coord(control_spec);
                  spec.upper_ = spec.lower_;
                }
                else if (translation_name.compare(translation3d_names()[2]) == 0)
                {
                  spec.expression_ = z_coord(fk_spec);
                  spec.lower_ = z_coord(control_spec);
                  spec.upper_ = spec.lower_;
                }
                else
                  throw std::runtime_error("Could not generate soft constraint for unknown Cartesian position name '"
                                           + translation_name + "'.");
                specs.push_back(spec);
              }
              break;
            }
            case ControlParams::ControlType::Rotation3D:
            {
              // TODO: replace this by using a new expression similar to KDL::Rot(KDL::Vector)
              RotationSpecPtr goal_spec = axis_angle_spec(
                  vector_constructor_spec(
                      get_goal_inputs(params.first).find(rotation3d_names()[0])->second,
                      get_goal_inputs(params.first).find(rotation3d_names()[1])->second,
                      get_goal_inputs(params.first).find(rotation3d_names()[2])->second),
                  get_goal_inputs(params.first).find(rotation3d_names()[3])->second);
              RotationSpecPtr fk_spec = orientation_of_spec(params_.robot_.get_fk_spec(params.second.root_link, params.second.tip_link));
              VectorSpecPtr control_spec = rotation3d_control_spec(goal_spec, fk_spec, params.second);
              for (size_t i=0; i<QPControllerSpecGenerator::rotation3d_names().size() -1; ++i)
              {
                std::string rotation_name = QPControllerSpecGenerator::rotation3d_names()[i];
                SoftConstraintSpec spec;
                spec.name_ = create_input_name(params.first, rotation_name);
                spec.weight_ = double_const_spec(params.second.weight);
                if (rotation_name.compare(rotation3d_names()[0]) == 0)
                {
                  spec.expression_ = x_coord(rot_vector(fk_spec));
                  spec.lower_ = x_coord(control_spec);
                  spec.upper_ = spec.lower_;
                }
                else if (rotation_name.compare(rotation3d_names()[1]) == 0)
                {
                  spec.expression_ = y_coord(rot_vector(fk_spec));
                  spec.lower_ = y_coord(control_spec);
                  spec.upper_ = spec.lower_;
                }
                else if (rotation_name.compare(rotation3d_names()[2]) == 0)
                {
                  spec.expression_ = z_coord(rot_vector(fk_spec));
                  spec.lower_ = z_coord(control_spec);
                  spec.upper_ = spec.lower_;
                }
                else
                  throw std::runtime_error("Could not generate soft constraint for unknown dimension '" + rotation_name + "'.");
                specs.push_back(spec);
              }
              break;
            }
            // TODO: cover other cases
            default:
              throw std::runtime_error("Could not generate soft constraint for unknown control type for control '" + params.first + "'.");
          }

          return specs;
        }

        VectorSpecPtr translation3d_control_spec(const VectorSpecPtr& goal, const VectorSpecPtr& state,
            const ControlParams& params) const
        {
            VectorSpecPtr error_vector = vector_sub_spec({goal, state});
            if (params.threshold_error)
            {
              DoubleSpecPtr threshold = double_const_spec(params.threshold);
              DoubleSpecPtr error = vector_norm(error_vector);
              DoubleSpecPtr scale = double_if(double_sub_spec({threshold, error}), double_const_spec(1.0), double_div({threshold, error}));
              error_vector = vector_double_mul(error_vector, scale);
            }

            return vector_double_mul(error_vector, double_const_spec(params.p_gain));
        }

        VectorSpecPtr rotation3d_control_spec(const RotationSpecPtr& goal, const RotationSpecPtr& state,
            const ControlParams& params) const
        {
            DoubleSpecPtr interpolation_value = double_const_spec(1.0);
            if (params.threshold_error)
            {
              RotationSpecPtr delta_rot = rotation_multiplication_spec({inverse_rotation_spec(state), goal});
              DoubleSpecPtr rot_error = vector_norm(rot_vector(delta_rot));
              interpolation_value =
                  double_if(double_sub_spec({double_const_spec(params.threshold), rot_error}),
                            double_const_spec(1.0),
                            double_div({double_const_spec(params.threshold), rot_error}));
            }
            RotationSpecPtr intermediate_goal = slerp_spec(state, goal, interpolation_value);
            DoubleSpecPtr p_gain = double_const_spec(params.p_gain);
            VectorSpecPtr error_rot_vector =
                    rotate_vector(state, rot_vector(rotation_multiplication_spec({inverse_rotation_spec(state), intermediate_goal})));
            return vector_double_mul(error_rot_vector, p_gain);
        }

        DoubleSpecPtr joint_control_spec(const DoubleSpecPtr& goal, const DoubleSpecPtr& state, const ControlParams& params,
            bool continuous_joint = false) const
        {
          DoubleSpecPtr error_exp = double_sub_spec({goal, state});
          if (continuous_joint)
          {
            DoubleSpecPtr pi= double_const_spec(QPControllerSpecGenerator::pi())  ;
            DoubleSpecPtr two_pi= double_const_spec(2.0*QPControllerSpecGenerator::pi())  ;
            DoubleSpecPtr error_unnormalized = error_exp;
            DoubleSpecPtr error_normalized = fmod(double_add_spec({fmod(error_unnormalized, two_pi), two_pi}), two_pi);
            error_exp = double_if(double_sub_spec({error_normalized, pi}), double_sub_spec({error_normalized, two_pi}), error_normalized);
          }
          DoubleSpecPtr control_exp = double_mul_spec({double_const_spec(params.p_gain), error_exp});
          if (params.threshold_error)
            throw std::runtime_error("Thresholding of joint controllers not implemented, yet.");
          return control_exp;
        }
    };

}

#endif //GISKARD_CORE_QP_CONTROLLER_SPEC_GENERATOR_HPP