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
        // TODO: add reference frame
        enum ControlType {UNKNOWN, JOINT, Translation3D, Rotation3D};
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

        const std::map<std::string, DoubleInputSpecPtr>& get_goal_inputs(const std::string& control_name) const
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

        static double pi()
        {
          static double result = 3.14159265359;
          return result;
        }

        static std::vector<std::string> rotation3d_names()
        {
          static std::vector<std::string> result = {"axis-x", "axis-y", "axis-z", "angle"};
          return result;
        }

        static std::vector<std::string> translation3d_names()
        {
          static std::vector<std::string> result = {"x", "y", "z"};
          return result;
        }

      protected:
        Robot robot_;
        WholeBodyControlParams params_;
        QPControllerSpec qp_spec_;
        std::map<std::string, std::map<std::string, DoubleInputSpecPtr> > goal_inputs_;

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

        std::map<std::string, DoubleInputSpecPtr> get_goal_inputs(const ControlParams& params, size_t start_index) const
        {
          std::map<std::string, DoubleInputSpecPtr> result;
          switch(params.type)
          {
            case ControlParams::ControlType::JOINT:
              for (auto const & joint_name : robot_.chain_joint_names(params.root_link, params.tip_link, false))
                result.insert(std::make_pair(joint_name, input(start_index++)));
              break;
            case ControlParams::ControlType::Translation3D:
              for (auto const & translation_name : ControllerSpecGenerator::translation3d_names())
                result.insert(std::make_pair(translation_name, input(start_index++)));
              break;
            case ControlParams::ControlType::Rotation3D:
              for (auto const & rotation_name : ControllerSpecGenerator::rotation3d_names())
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
            case ControlParams::ControlType::JOINT:
            {
              std::set<std::string> continuous_joints_names =
                  robot_.continuous_joints_names(params.second.root_link, params.second.tip_link);
              for (auto const &joint_name: robot_.chain_joint_names(params.second.root_link, params.second.tip_link, false))
              {
                SoftConstraintSpec spec;
                DoubleInputSpecPtr goal_spec = get_goal_inputs(params.first).find(joint_name)->second;
                DoubleInputSpecPtr joint_spec = robot_.get_joint(joint_name);
                spec.lower_ = joint_control_spec(goal_spec, joint_spec, params.second,
                                           continuous_joints_names.find(joint_name) !=
                                           continuous_joints_names.end());
                spec.upper_ = spec.lower_;
                spec.expression_ = joint_spec;
                spec.weight_ = double_const_spec(params.second.weight);
                spec.name_ = params.first + "_" + joint_name;
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
              VectorSpecPtr fk_spec = origin(robot_.get_fk_spec(params.second.root_link, params.second.tip_link));
              VectorSpecPtr control_spec = translation3d_control_spec(goal_spec, fk_spec, params.second);
              for (auto const & translation_name: ControllerSpecGenerator::translation3d_names())
              {
                SoftConstraintSpec spec;
                spec.name_ = params.first + "_" + translation_name;
                spec.weight_ = double_const_spec(params.second.weight);
                if (translation_name.compare("x") == 0)
                {
                  spec.expression_ = x_coord(fk_spec);
                  spec.lower_ = x_coord(control_spec);
                  spec.upper_ = spec.lower_;
                }
                else if (translation_name.compare("y") == 0)
                {
                  spec.expression_ = y_coord(fk_spec);
                  spec.lower_ = y_coord(control_spec);
                  spec.upper_ = spec.lower_;
                }
                else if (translation_name.compare("z") == 0)
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
              RotationSpecPtr fk_spec = orientation_of_spec(robot_.get_fk_spec(params.second.root_link, params.second.tip_link));
              VectorSpecPtr control_spec = rotation3d_control_spec(goal_spec, fk_spec, params.second);
              for (size_t i=0; i<ControllerSpecGenerator::rotation3d_names().size() -1; ++i)
              {
                std::string rotation_name = ControllerSpecGenerator::rotation3d_names()[i];
                SoftConstraintSpec spec;
                spec.name_ = params.first + "_" + rotation_name;
                spec.weight_ = double_const_spec(params.second.weight);
                if (rotation_name.compare("axis-x") == 0)
                {
                  spec.expression_ = x_coord(rot_vector(fk_spec));
                  spec.lower_ = x_coord(control_spec);
                  spec.upper_ = spec.lower_;
                }
                else if (rotation_name.compare("axis-y") == 0)
                {
                  spec.expression_ = y_coord(rot_vector(fk_spec));
                  spec.lower_ = y_coord(control_spec);
                  spec.upper_ = spec.lower_;
                }
                else if (rotation_name.compare("axis-z") == 0)
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
                // TODO: complete me
//            - r_rot_error: {vector-norm: {rot-vector: {rotation-mul: [{inverse-rotation: r_rot}, r_goal_rot]}}}
//            - r_rot_scaling:
//                double-if:
//                - {double-sub: [rot_thresh, r_rot_error]}
//                - 1
//                - {double-div: [rot_thresh, r_rot_error]}
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
            DoubleSpecPtr pi= double_const_spec(ControllerSpecGenerator::pi())  ;
            DoubleSpecPtr two_pi= double_const_spec(2.0*ControllerSpecGenerator::pi())  ;
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

#endif //GISKARD_CORE_WHOLE_BODY_CONTROL_PARAMS_HPP
