/*
 * Copyright (C) 2015 Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

#ifndef GISKARD_EXPRESSION_GENERATION_HPP
#define GISKARD_EXPRESSION_GENERATION_HPP

#include <giskard/scope.hpp>
#include <giskard/qp_controller.hpp>
#include <giskard/specifications.hpp>

namespace giskard
{

  inline giskard::Scope generate(const giskard::ScopeSpec& scope_spec)
  {
    giskard::Scope scope;

    for(size_t i=0; i<scope_spec.size(); ++i)
    {
      std::string name = scope_spec[i].name;
      giskard::SpecPtr spec = scope_spec[i].spec;

      if(boost::dynamic_pointer_cast<giskard::DoubleSpec>(spec).get())
        scope.add_double_expression(name,
            boost::dynamic_pointer_cast<giskard::DoubleSpec>(spec)->get_expression(scope));
      else if(boost::dynamic_pointer_cast<giskard::VectorSpec>(spec).get())
        scope.add_vector_expression(name,
            boost::dynamic_pointer_cast<giskard::VectorSpec>(spec)->get_expression(scope));
      else if(boost::dynamic_pointer_cast<giskard::FrameSpec>(spec).get())
        scope.add_frame_expression(name,
            boost::dynamic_pointer_cast<giskard::FrameSpec>(spec)->get_expression(scope));
      else if(boost::dynamic_pointer_cast<giskard::RotationSpec>(spec).get())
        scope.add_rotation_expression(name,
            boost::dynamic_pointer_cast<giskard::RotationSpec>(spec)->get_expression(scope));
      else
        // found non-supported type of specification in scope
        // TODO: issue warning, instead
        assert(false);
    }

    return scope;
  }

  inline giskard::QPController generate(const giskard::QPControllerSpec& spec)
  {
    giskard::Scope scope = generate(spec.scope_);

    // generate controllable constraints
    std::vector< KDL::Expression<double>::Ptr > controllable_lower, controllable_upper,
        controllable_weight;
    for(size_t i=0; i<spec.controllable_constraints_.size(); ++i)
    {
      // TODO: throw an exception, instead
      assert(spec.controllable_constraints_[i].input_number_ == i);

      controllable_lower.push_back(spec.controllable_constraints_[i].lower_->get_expression(scope));
      controllable_upper.push_back(spec.controllable_constraints_[i].upper_->get_expression(scope));
      controllable_weight.push_back(spec.controllable_constraints_[i].weight_->get_expression(scope));
    }

    // generate soft constraints
    std::vector< KDL::Expression<double>::Ptr > soft_lower, soft_upper,
        soft_weight, soft_exp;
    for(size_t i=0; i<spec.soft_constraints_.size(); ++i)
    {
      soft_lower.push_back(spec.soft_constraints_[i].lower_->get_expression(scope));
      soft_upper.push_back(spec.soft_constraints_[i].upper_->get_expression(scope));
      soft_weight.push_back(spec.soft_constraints_[i].weight_->get_expression(scope));
      soft_exp.push_back(spec.soft_constraints_[i].expression_->get_expression(scope));
    }

    // generate hard constraints
    std::vector< KDL::Expression<double>::Ptr > hard_lower, hard_upper, hard_exp;
    for(size_t i=0; i<spec.hard_constraints_.size(); ++i)
    {
      hard_lower.push_back(spec.hard_constraints_[i].lower_->get_expression(scope));
      hard_upper.push_back(spec.hard_constraints_[i].upper_->get_expression(scope));
      hard_exp.push_back(spec.hard_constraints_[i].expression_->get_expression(scope));
    }

    giskard::QPController controller;
   
    // TODO: throw an exception, instead
    assert(controller.init(controllable_lower, controllable_upper, controllable_weight,
                           soft_exp, soft_lower, soft_upper, soft_weight,
                           hard_exp, hard_lower, hard_upper));

    return controller;
  }
}

#endif // GISKARD_EXPRESSION_GENERATION_HPP
