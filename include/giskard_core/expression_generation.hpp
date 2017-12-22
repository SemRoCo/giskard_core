/*
 * Copyright (C) 2015-2017 Georg Bartels <georg.bartels@cs.uni-bremen.de>
 *                         Adrian Röfer <aroefer@uni-bremen.de>
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

#ifndef GISKARD_CORE_EXPRESSION_GENERATION_HPP
#define GISKARD_CORE_EXPRESSION_GENERATION_HPP

#include <giskard_core/scope.hpp>
#include <giskard_core/qp_controller.hpp>
#include <giskard_core/specifications.hpp>

namespace giskard_core
{

  inline giskard_core::Scope generate(const giskard_core::ScopeSpec& scope_spec)
  {
    giskard_core::Scope scope;

    for(size_t i=0; i<scope_spec.size(); ++i)
    {
      std::string name = scope_spec[i].name;
      giskard_core::SpecPtr spec = scope_spec[i].spec;

      if(boost::dynamic_pointer_cast<giskard_core::DoubleSpec>(spec).get())
        scope.add_double_expression(name,
            boost::dynamic_pointer_cast<giskard_core::DoubleSpec>(spec)->get_expression(scope));
      else if(boost::dynamic_pointer_cast<giskard_core::VectorSpec>(spec).get())
        scope.add_vector_expression(name,
            boost::dynamic_pointer_cast<giskard_core::VectorSpec>(spec)->get_expression(scope));
      else if(boost::dynamic_pointer_cast<giskard_core::FrameSpec>(spec).get())
        scope.add_frame_expression(name,
            boost::dynamic_pointer_cast<giskard_core::FrameSpec>(spec)->get_expression(scope));
      else if(boost::dynamic_pointer_cast<giskard_core::RotationSpec>(spec).get())
        scope.add_rotation_expression(name,
            boost::dynamic_pointer_cast<giskard_core::RotationSpec>(spec)->get_expression(scope));
      else if(boost::dynamic_pointer_cast<giskard_core::AliasReferenceSpec>(spec).get())
      {
        // generation of alias references is extraordinarily convoluted;
        // it is a feature that was added relatively late... sorry!
        KDL::ExpressionBase::Ptr exp =
          boost::dynamic_pointer_cast<giskard_core::AliasReferenceSpec>(spec)->get_expression(scope);
        if (boost::dynamic_pointer_cast<KDL::Expression<double>>(exp).get())
          scope.add_double_expression(name, boost::dynamic_pointer_cast<KDL::Expression<double>>(exp));
        else if (boost::dynamic_pointer_cast<KDL::Expression<KDL::Vector>>(exp).get())
          scope.add_vector_expression(name, boost::dynamic_pointer_cast<KDL::Expression<KDL::Vector>>(exp));
        else if (boost::dynamic_pointer_cast<KDL::Expression<KDL::Rotation>>(exp).get())
          scope.add_rotation_expression(name, boost::dynamic_pointer_cast<KDL::Expression<KDL::Rotation>>(exp));
        else if (boost::dynamic_pointer_cast<KDL::Expression<KDL::Frame>>(exp).get())
          scope.add_frame_expression(name, boost::dynamic_pointer_cast<KDL::Expression<KDL::Frame>>(exp));
        else
          throw std::domain_error("Error during generation of alias reference! Could not cast into any existing expression type. This is an giskard-internal error that should not happen.");
      }
      else
        throw std::domain_error("Scope generation: found entry of non-supported type.");
    }

    return scope;
  }

  inline giskard_core::QPController generate(const giskard_core::QPControllerSpec& spec)
  {
    giskard_core::Scope scope = generate(spec.scope_);

    // generate controllable constraints
    std::vector< KDL::Expression<double>::Ptr > controllable_lower, controllable_upper,
        controllable_weight;
    std::vector<std::string> controllable_name;
    for(size_t i=0; i<spec.controllable_constraints_.size(); ++i)
    {
      if(spec.controllable_constraints_[i].input_number_ != i)
        // TODO: replace boost::lexical_cast<std::string>() with std::to_string() that is available as of C++11
        throw std::invalid_argument("QPController generation: controllable constraint at position " + 
            boost::lexical_cast<std::string>(i) + " has not input number " + 
            boost::lexical_cast<std::string>(i) + ". Instead it has incorrect input number: " +
            boost::lexical_cast<std::string>(spec.controllable_constraints_[i].input_number_));

      controllable_lower.push_back(spec.controllable_constraints_[i].lower_->get_expression(scope));
      controllable_upper.push_back(spec.controllable_constraints_[i].upper_->get_expression(scope));
      controllable_weight.push_back(spec.controllable_constraints_[i].weight_->get_expression(scope));
      controllable_name.push_back(spec.controllable_constraints_[i].name_);
    }

    // generate soft constraints
    std::vector< KDL::Expression<double>::Ptr > soft_lower, soft_upper,
        soft_weight, soft_exp;
    std::vector< std::string> soft_name;
    for(size_t i=0; i<spec.soft_constraints_.size(); ++i)
    {
      soft_lower.push_back(spec.soft_constraints_[i].lower_->get_expression(scope));
      soft_upper.push_back(spec.soft_constraints_[i].upper_->get_expression(scope));
      soft_weight.push_back(spec.soft_constraints_[i].weight_->get_expression(scope));
      soft_exp.push_back(spec.soft_constraints_[i].expression_->get_expression(scope));
      soft_name.push_back(spec.soft_constraints_[i].name_);
    }

    // generate hard constraints
    std::vector< KDL::Expression<double>::Ptr > hard_lower, hard_upper, hard_exp;
    for(size_t i=0; i<spec.hard_constraints_.size(); ++i)
    {
      hard_lower.push_back(spec.hard_constraints_[i].lower_->get_expression(scope));
      hard_upper.push_back(spec.hard_constraints_[i].upper_->get_expression(scope));
      hard_exp.push_back(spec.hard_constraints_[i].expression_->get_expression(scope));
    }

    giskard_core::QPController controller;
   
    if(!(controller.init(controllable_lower, controllable_upper, controllable_weight,
                           controllable_name, soft_exp, soft_lower, soft_upper, 
                           soft_weight, soft_name, hard_exp, hard_lower, hard_upper)))
      throw std::runtime_error("QPController generation: Init of controller failed.");

    controller.set_scope(scope);

    return controller;
  }
}

#endif // GISKARD_CORE_EXPRESSION_GENERATION_HPP
