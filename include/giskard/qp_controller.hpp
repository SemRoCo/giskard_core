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

#ifndef GISKARD_QP_CONTROLLER_HPP
#define GISKARD_QP_CONTROLLER_HPP

#include <giskard/qp_problem_builder.hpp>
#include <qpOASES.hpp>

namespace giskard
{
  class QPController
  {
    public:
      typedef typename std::vector< KDL::Expression<double>::Ptr > DoubleExpressionVector;
      
      bool init(const DoubleExpressionVector& controllable_lower_bounds,
          const DoubleExpressionVector& controllable_upper_bounds, const DoubleExpressionVector& controllable_weights,
          const DoubleExpressionVector& soft_expressions, const DoubleExpressionVector& soft_lower_bounds,
          const DoubleExpressionVector& soft_upper_bounds, const DoubleExpressionVector& soft_weights,
          const DoubleExpressionVector& hard_expressions, const DoubleExpressionVector& hard_lower_bounds,
          const DoubleExpressionVector& hard_upper_bounds)
      {
        qp_builder_.init(controllable_lower_bounds, controllable_upper_bounds,
            controllable_weights, soft_expressions, soft_lower_bounds,
            soft_upper_bounds, soft_weights, hard_expressions,
            hard_lower_bounds, hard_upper_bounds);

        qp_problem_ = qpOASES::SQProblem(qp_builder_.num_weights(), qp_builder_.num_constraints());
        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = qpOASES::PL_NONE;
        qp_problem_.setOptions(options);

        xdot_full_.resize(qp_builder_.num_weights());

        xdot_partial_.resize(qp_builder_.num_controllables());

        return true;
      }

      bool start(const Eigen::VectorXd& observables, int nWSR)
      {
        qp_builder_.update(observables);

        qpOASES::returnValue return_value = qp_problem_.init(qp_builder_.get_H().data(), qp_builder_.get_g().data(), 
            qp_builder_.get_A().data(), qp_builder_.get_lb().data(), qp_builder_.get_ub().data(),
            qp_builder_.get_lbA().data(), qp_builder_.get_ubA().data(), nWSR);

        if(return_value != qpOASES::SUCCESSFUL_RETURN)
        {
          std::cout << "Init of QP-Problem returned without success! Printing internals." << std::endl;
          qp_builder_.print_internals();
          std::cout << "nWSR: " << nWSR << std::endl;
        }
        
        return return_value == qpOASES::SUCCESSFUL_RETURN;
      }
      
 
      bool update(const Eigen::VectorXd& observables, int nWSR)
      {
       qp_builder_.update(observables);

       if( qp_problem_.hotstart(qp_builder_.get_H().data(), qp_builder_.get_g().data(), 
           qp_builder_.get_A().data(), qp_builder_.get_lb().data(), qp_builder_.get_ub().data(),
           qp_builder_.get_lbA().data(), qp_builder_.get_ubA().data(), nWSR)
           != qpOASES::SUCCESSFUL_RETURN )
          return false;

        qp_problem_.getPrimalSolution(xdot_full_.data());
        xdot_partial_ = xdot_full_.segment(0, qp_builder_.num_controllables());

        return true;
      }

      const Eigen::VectorXd& get_command() const
      {
        return xdot_partial_;
      }

      const QPProblemBuilder& get_qp_builder() const
      {
        return qp_builder_;
      }

    private:
      giskard::QPProblemBuilder qp_builder_;
      qpOASES::SQProblem qp_problem_;
      Eigen::VectorXd xdot_full_, xdot_partial_;
  };

}

#endif // GISKARD_QP_CONTROLLER_HPP
