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

#ifndef GISKARD_QP_PROBLEM_BUILDER_HPP
#define GISKARD_QP_PROBLEM_BUILDER_HPP

#include <giskard/expressiontree.hpp>

namespace giskard
{
  class QPProblemBuilder
  {
    public:
      typedef typename std::vector< KDL::Expression<double>::Ptr > DoubleExpressionVector;
      typedef typename Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Matrix;
      typedef typename Eigen::VectorXd Vector;
     
      void init(const DoubleExpressionVector& controllable_lower_bounds,
          const DoubleExpressionVector& controllable_upper_bounds, const DoubleExpressionVector& controllable_weights,
          const DoubleExpressionVector& soft_expressions, const DoubleExpressionVector& soft_lower_bounds,
          const DoubleExpressionVector& soft_upper_bounds, const DoubleExpressionVector& soft_weights,
          const DoubleExpressionVector& hard_expressions, const DoubleExpressionVector& hard_lower_bounds,
          const DoubleExpressionVector& hard_upper_bounds)
      {
        set_expressions(controllable_lower_bounds, controllable_upper_bounds,
            controllable_weights, soft_expressions, soft_lower_bounds,
            soft_upper_bounds, soft_weights, hard_expressions,
            hard_lower_bounds, hard_upper_bounds);
 
        create_output_matrices();
      }

      void update(const Vector& observables)
      {
        update_expressions(observables);
        copy_values();
      }

      const Matrix& get_H() const
      {
        return H_;
      }

      const Matrix& get_A() const
      {
        return A_;
      }

      const Vector& get_g() const
      {
        return g_;
      }

      const Vector& get_lb() const
      {
        return lb_;
      }     

      const Vector& get_ub() const
      {
        return ub_;
      }     

      const Vector& get_lbA() const
      {
        return lbA_;
      }     

      const Vector& get_ubA() const
      {
        return ubA_;
      }     

      size_t num_controllables() const
      {
        return controllable_weights_.num_expressions();
      }

      size_t num_hard_constraints() const
      {
        return hard_expressions_.num_expressions();
      }

      size_t num_hard_constraints_observables() const
      {
        return hard_expressions_.num_inputs();
      }

      size_t num_soft_constraints() const
      {
        return soft_expressions_.num_expressions();
      }

      size_t num_soft_constraints_observables() const
      {
        return soft_expressions_.num_inputs();
      }

      size_t num_constraints() const
      {
        return num_soft_constraints() + num_hard_constraints();
      }

      size_t num_weights() const
      {
        return num_controllables() + num_soft_constraints();
      }

      const DoubleExpressionVector& get_soft_lower_bounds() const
      {
        return soft_lower_bounds_.get_expressions();
      }

      const DoubleExpressionVector& get_soft_upper_bounds() const
      {
        return soft_upper_bounds_.get_expressions();
      }

      const DoubleExpressionVector& get_soft_expressions() const
      {
        return soft_expressions_.get_expressions();
      }

      const DoubleExpressionVector& get_soft_weights() const
      {
        return soft_weights_.get_expressions();
      }
    
    private:
      KDL::DoubleExpressionArray controllable_lower_bounds_, controllable_upper_bounds_,
         controllable_weights_, soft_expressions_, soft_lower_bounds_, soft_upper_bounds_,
         soft_weights_, hard_expressions_, hard_lower_bounds_, hard_upper_bounds_;

      Matrix H_, A_;
      Vector g_, lb_, ub_, lbA_, ubA_;

      void set_expressions(const DoubleExpressionVector& controllable_lower_bounds,
          const DoubleExpressionVector& controllable_upper_bounds, const DoubleExpressionVector& controllable_weights,
          const DoubleExpressionVector& soft_expressions, const DoubleExpressionVector& soft_lower_bounds,
          const DoubleExpressionVector& soft_upper_bounds, const DoubleExpressionVector& soft_weights,
          const DoubleExpressionVector& hard_expressions, const DoubleExpressionVector& hard_lower_bounds,
          const DoubleExpressionVector& hard_upper_bounds)
      {
        controllable_lower_bounds_.set_expressions(controllable_lower_bounds);
        controllable_upper_bounds_.set_expressions(controllable_upper_bounds);
        controllable_weights_.set_expressions(controllable_weights);

        soft_expressions_.set_expressions(soft_expressions);
        soft_lower_bounds_.set_expressions(soft_lower_bounds);
        soft_upper_bounds_.set_expressions(soft_upper_bounds);
        soft_weights_.set_expressions(soft_weights);

        hard_expressions_.set_expressions(hard_expressions);
        hard_lower_bounds_.set_expressions(hard_lower_bounds);
        hard_upper_bounds_.set_expressions(hard_upper_bounds);
      }

      void create_output_matrices()
      {
        H_ = Eigen::MatrixXd::Zero(num_weights(), num_weights());

        A_ = Eigen::MatrixXd::Zero(num_constraints(), num_weights());
        A_.block(num_hard_constraints(), num_controllables(), num_soft_constraints(), num_soft_constraints()) =
            Eigen::MatrixXd::Identity(num_soft_constraints(), num_soft_constraints());
 
        g_ = Eigen::VectorXd::Zero(num_weights());
        lb_ = Eigen::VectorXd::Zero(num_weights());
        ub_ = Eigen::VectorXd::Zero(num_weights());
        lbA_ = Eigen::VectorXd::Zero(num_constraints());
        ubA_ = Eigen::VectorXd::Zero(num_constraints());
      }

      void update_expressions(const Vector& observables)
      {
        update_expressions(controllable_lower_bounds_, observables);
        update_expressions(controllable_upper_bounds_, observables);
        update_expressions(controllable_weights_, observables);

        update_expressions(soft_expressions_, observables);
        update_expressions(soft_lower_bounds_, observables);
        update_expressions(soft_upper_bounds_, observables);
        update_expressions(soft_weights_, observables);

        update_expressions(hard_expressions_, observables);
        update_expressions(hard_lower_bounds_, observables);
        update_expressions(hard_upper_bounds_, observables);
      }

      void copy_values()
      {
        H_.diagonal().segment(0, num_controllables()) =
            controllable_weights_.get_values();
        H_.diagonal().segment(num_controllables(), num_soft_constraints()) =
            soft_weights_.get_values();

        size_t cols_to_copy = std::min(num_hard_constraints_observables(), num_controllables());
        A_.block(0, 0, num_hard_constraints(), cols_to_copy) =
            hard_expressions_.get_derivatives().block(0, 0, num_hard_constraints(), cols_to_copy);
        cols_to_copy = std::min(num_soft_constraints_observables(), num_controllables());
        A_.block(num_hard_constraints(), 0, num_soft_constraints(), cols_to_copy) = 
            soft_expressions_.get_derivatives().block(0, 0, num_soft_constraints(), cols_to_copy);

        lb_.segment(0, num_controllables()) = controllable_lower_bounds_.get_values();
        // TODO: try to get rid of these constants
        lb_.segment(num_controllables(), num_soft_constraints()) = 
            -1e+9 * Eigen::VectorXd::Ones(num_soft_constraints());
        ub_.segment(0, num_controllables()) = controllable_upper_bounds_.get_values();
        ub_.segment(num_controllables(), num_soft_constraints()) = 
            1e+9 * Eigen::VectorXd::Ones(num_soft_constraints());

        lbA_.segment(0, num_hard_constraints()) = hard_lower_bounds_.get_values();
        lbA_.segment(num_hard_constraints(), num_soft_constraints()) = soft_lower_bounds_.get_values();
        ubA_.segment(0, num_hard_constraints()) = hard_upper_bounds_.get_values();
        ubA_.segment(num_hard_constraints(), num_soft_constraints()) = soft_upper_bounds_.get_values();
      }

      void update_expressions(KDL::DoubleExpressionArray& expressions, const Vector& values) const
      {
        expressions.update(values.segment(0, expressions.num_inputs()));
      }
  };
} 

#endif // GISKARD_QP_PROBLEM_BUILDER_HPP
