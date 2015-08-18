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
      else
        // found non-supported type of specification in scope
        // TODO: issue warning, instead
        assert(false);
    }

    return scope;
  }

  inline giskard::QPController generate(const giskard::QPControllerSpec& spec)
  {
    giskard::QPController controller;

//TODO: implement me
    return controller;
  }
}

#endif // GISKARD_EXPRESSION_GENERATION_HPP
