/*
 * Copyright (C) 2015-2017 Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

#ifndef GISKARD_CORE_QP_CONTROLLER_HPP
#define GISKARD_CORE_QP_CONTROLLER_HPP

#include <giskard_core/qp_problem_builder.hpp>
#include <giskard_core/scope.hpp>
#include <boost/lexical_cast.hpp>
#include <qpOASES.hpp>

namespace giskard_core
{
  class QPController
  {
    public:
      typedef typename std::vector< KDL::Expression<double>::Ptr > DoubleExpressionVector;
      typedef typename std::vector< std::string> StringVector;
      
      bool init(const DoubleExpressionVector& controllable_lower_bounds,
          const DoubleExpressionVector& controllable_upper_bounds, const DoubleExpressionVector& controllable_weights,
          const StringVector& controllable_names, const DoubleExpressionVector& soft_expressions,
          const DoubleExpressionVector& soft_lower_bounds, const DoubleExpressionVector& soft_upper_bounds,
          const DoubleExpressionVector& soft_weights, const StringVector& soft_names,
          const DoubleExpressionVector& hard_expressions, const DoubleExpressionVector& hard_lower_bounds,
          const DoubleExpressionVector& hard_upper_bounds)
      {
        qp_builder_.init(controllable_lower_bounds, controllable_upper_bounds,
            controllable_weights, soft_expressions, soft_lower_bounds,
            soft_upper_bounds, soft_weights, hard_expressions,
            hard_lower_bounds, hard_upper_bounds);

        qp_problem_ = qpOASES::SQProblem(qp_builder_.num_weights(), qp_builder_.num_constraints());
        qpOASES::Options options;
        // NOTE: In the past, I was using setting "reliable", and found a curious
        //       bug: One trying to solve an already solved problem, the solver
        //       would never finish and run out of working set iterations. The
        //       corresponding test-case is broken flying cup. Switching to
        //       "default" solved this on qpOASES 3.1.
        // NOTE: Even earlier, I was using setting "MPC" that left to weird behavior
        //       for orientation control. It seemed as if the solver returned 
        //       inaccurate solutions. We (Alexis and Georg) decided to swith
        //       away from "MPC" to improve this behavior. That was also for
        //       qpOASES 3.1. However, now I cannot reproduce that problem.
        options.setToDefault();
        options.printLevel = qpOASES::PL_NONE;
        qp_problem_.setOptions(options);

        xdot_full_.resize(qp_builder_.num_weights());

        xdot_control_.resize(qp_builder_.num_controllables());

        xdot_slack_.resize(qp_builder_.num_soft_constraints());

        if( controllable_names.size() != qp_builder_.num_controllables() )
          throw std::runtime_error("Received " + boost::lexical_cast<std::string>(controllable_names_.size()) + 
              " controllable names, but " + boost::lexical_cast<std::string>(qp_builder_.num_controllables()) + 
              " controllables were specified.");
        controllable_names_ = controllable_names;

        if( soft_names.size() != qp_builder_.num_soft_constraints() )
          throw std::runtime_error("Received " + boost::lexical_cast<std::string>(soft_names.size()) + 
              " soft constraint names, but " + boost::lexical_cast<std::string>(qp_builder_.num_soft_constraints()) + 
              " soft constraints were specified.");
        soft_constraint_names_ = soft_names;

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
          std::cout << "Init of QP-Problem returned without success! ERROR MESSAGE: " << 
            qpOASES::MessageHandling::getErrorCodeMessage(return_value) << std::endl;
          std::cout << "Printing internals." << std::endl;
          qp_builder_.print_internals();
          std::cout << "nWSR: " << nWSR << std::endl;
          qp_builder_.are_internals_valid();
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
        xdot_control_ = xdot_full_.segment(0, qp_builder_.num_controllables());
        xdot_slack_ = xdot_full_.segment(qp_builder_.num_controllables(), qp_builder_.num_soft_constraints());

        return true;
      }

      const Eigen::VectorXd& get_command() const
      {
        return xdot_control_;
      }

      const Eigen::VectorXd& get_slack() const
      {
        return xdot_slack_;
      }

      const QPProblemBuilder& get_qp_builder() const
      {
        return qp_builder_;
      }

      const std::vector<std::string>& get_controllable_names() const
      {
        return controllable_names_;
      }

      const std::vector<std::string>& get_soft_constraint_names() const
      {
        return soft_constraint_names_;
      }

      const giskard_core::Scope& get_scope() const
      {
        return scope_;
      }

      void set_scope(const giskard_core::Scope& scope)
      {
        scope_ = scope;
      }

      size_t num_controllables() const
      {
        return get_controllable_names().size();
      }

      size_t num_soft_constraints() const
      {
        return get_soft_constraint_names().size();
      }

      size_t num_observables() const
      {
        return qp_builder_.num_observables();
      }

      // Returns the commands associated with the names of their corresponding controllables aka joints
      std::map<std::string, double> get_command_map() const
      {
        std::map<std::string, double> out;
        if (xdot_control_.size() != controllable_names_.size())
          throw std::domain_error("QP-Controller: There are unequal amounts of controllables and commands! This is severly wrong and should never happen!");
        for (size_t i = 0; i < controllable_names_.size(); i++)
          out[controllable_names_[i]] = xdot_control_[i];
        
        return out;
      }

      // Set scalar input
      void set_input(Eigen::VectorXd& inputVector, const std::string& name, double value) const {
        giskard_core::Scope::InputPtr ptr = scope_.find_input(name);
        if (ptr->get_type() == tScalar || ptr->get_type() == tJoint){
            if(ptr->idx_ < inputVector.size()) {
              inputVector[ptr->idx_] = value;
              return;
            }
            throw std::invalid_argument("Can't set scalar or joint input with name '" + name + 
                  "' because the input vector is too small. Needed size: " + 
                  std::to_string(ptr->idx_ + 1) + " Actual size: " + std::to_string(inputVector.size()));
        }
        throw std::invalid_argument("Can't set scalar or joint input with name '" + name + "' as it does not exist.");
      }

      // Set vector input using Eigen::Vector3d
      void set_input(Eigen::VectorXd& inputVector, const std::string& name, Eigen::Vector3d value) const {        
            set_input(inputVector, name, value[0], value[1], value[2]);
      }

      // Set vector input using KDL::Vector
      void set_input(Eigen::VectorXd& inputVector, const std::string& name, KDL::Vector value) const {        
            set_input(inputVector, name, value[0], value[1], value[2]);
      }

      void set_input(Eigen::VectorXd& inputVector, const std::string& name, double x, double y, double z) const {
        giskard_core::Scope::Vec3InputPtr ptr = scope_.find_input<giskard_core::Scope::Vec3Input>(name);
          if(ptr->idx_ + 2 < inputVector.size()) {         
            inputVector[ptr->idx_] = x;
            inputVector[ptr->idx_ + 1] = y;
            inputVector[ptr->idx_ + 2] = z;
            return;
         }
         throw std::invalid_argument("Can't set vec3 input with name '" + name + 
                  "' because the input vector is too small. Needed size: " + 
                  std::to_string(ptr->idx_ + 3) + " Actual size: " + std::to_string(inputVector.size()));
      }

      // Set rotation input using a KDL::Rotation
      void set_input(Eigen::VectorXd& inputVector, const std::string& name, KDL::Rotation value) const {
        KDL::Vector axis;
        double angle = value.GetRotAngle(axis, KDL::epsilon);
        set_input(inputVector, name, axis[0], axis[1], axis[2], angle);
      }

      // Set rotation input using an Eigen::Quaterniond
      void set_input(Eigen::VectorXd& inputVector, const std::string& name, Eigen::Quaterniond value) const {
        Eigen::AngleAxisd aa(value);
        Eigen::Vector3d ax = aa.axis();
        set_input(inputVector, name, ax[0], ax[1], ax[2], aa.angle());
      }

      // Set rotation input using an Eigen::Vector3d as axis and an angle
      void set_input(Eigen::VectorXd& inputVector, const std::string& name, Eigen::Vector3d axis, double angle) const {
        set_input(inputVector, name, axis[0], axis[1], axis[2], angle);
      }

      // Set rotation input using axis and angle as individual scalars
      void set_input(Eigen::VectorXd& inputVector, const std::string& name, double x, double y, double z, double angle) const {
        giskard_core::Scope::RotationInputPtr ptr = scope_.find_input<giskard_core::Scope::RotationInput>(name);
        if (ptr->idx_ + 3 < inputVector.size()) {
          inputVector[ptr->idx_] = x;
          inputVector[ptr->idx_ + 1] = y;
          inputVector[ptr->idx_ + 2] = z;
          inputVector[ptr->idx_ + 3] = angle;

          return;
        }
        throw std::invalid_argument("Can't set rotation input with name '" + name + 
                "' because the input vector is too small. Needed size: " + 
                std::to_string(ptr->idx_ + 4) + " Actual size: " + std::to_string(inputVector.size()));
      }

      // Set frame input using a KDL::Frame
      void set_input(Eigen::VectorXd& inputVector, const std::string& name, KDL::Frame value) const {  
        KDL::Vector translation = value.p;
        KDL::Rotation rotation  = value.M;
        KDL::Vector axis;
        double angle = rotation.GetRotAngle(axis, KDL::epsilon);
        set_input(inputVector, name, axis[0], axis[1], axis[2], angle, translation[0], translation[1], translation[2]);
      }

      // Set frame input using a KDL::Rotation and KDL::Vector
      void set_input(Eigen::VectorXd& inputVector, const std::string& name, KDL::Rotation rotation, KDL::Vector translation) const {
        KDL::Vector axis;
        double angle = rotation.GetRotAngle(axis, KDL::epsilon);
        set_input(inputVector, name, axis[0], axis[1], axis[2], angle, translation[0], translation[1], translation[2]);
      }

      // Set frame input using an Eigen::Affine3d
      void set_input(Eigen::VectorXd& inputVector, const std::string& name, Eigen::Affine3d value) const {  
        Eigen::AngleAxisd aa(value.rotation());
        Eigen::Vector3d axis = aa.axis();
        Eigen::Vector3d translation = value.translation();
        set_input(inputVector, name, axis[0], axis[1], axis[2], aa.angle(), translation[0], translation[1], translation[2]);
      }

      // Set frame input using an Eigen::Quaterniond and Eigen::Vector3d
      void set_input(Eigen::VectorXd& inputVector, const std::string& name, Eigen::Quaterniond rotation, Eigen::Vector3d translation) const {
        Eigen::AngleAxisd aa(rotation);
        Eigen::Vector3d axis = aa.axis();
        set_input(inputVector, name, axis[0], axis[1], axis[2], aa.angle(), translation[0], translation[1], translation[2]);
      }

      // Set frame input using an Eigen::Vector3d axis, an angle and an Eigen::Vector3d translation
      void set_input(Eigen::VectorXd& inputVector, const std::string& name, Eigen::Vector3d axis, double angle, Eigen::Vector3d translation) const {
        set_input(inputVector, name, axis[0], axis[1], axis[2], angle, translation[0], translation[1], translation[2]);
      }

      // Set frame input using an axis, angle and translation, encoded as seven scalars
      void set_input(Eigen::VectorXd& inputVector, const std::string& name, double rx, double ry, double rz, double angle, double x, double y, double z) const {
        giskard_core::Scope::FrameInputPtr ptr = scope_.find_input<giskard_core::Scope::FrameInput>(name);
        if (ptr->idx_ + 6 < inputVector.size()) {

          inputVector[ptr->idx_] = rx;
          inputVector[ptr->idx_ + 1] = ry;
          inputVector[ptr->idx_ + 2] = rz;
          inputVector[ptr->idx_ + 3] = angle;
          inputVector[ptr->idx_ + 4] = x;
          inputVector[ptr->idx_ + 5] = y;
          inputVector[ptr->idx_ + 6] = z;
          return;
        }
        throw std::invalid_argument("Can't set frame input with name '" + name + 
                "' because the input vector is too small. Needed size: " + 
                std::to_string(ptr->idx_ + 7) + " Actual size: " + std::to_string(inputVector.size()));
      }

      // Shorthand for getting input size of scope
      size_t get_input_size() const {
        return scope_.get_input_size();
      }

      // Shorthand for getting names of all inputs in scope
      std::vector<std::string> get_input_names() const {
        return scope_.get_input_names();
      }

      // Shorthand for getting names of inputs in scope filtered by enum type
      std::vector<std::string> get_input_names(const InputType type) const {
        return scope_.get_input_names(type);
      }

      // Shorthand for getting names of inputs in scope filtered by type
      template<typename T>
      std::vector<std::string> get_input_names() const {
        return scope_.get_input_names<T>();
      }

      // Shorthand for getting all input structures in scope
      std::vector<Scope::InputPtr> get_inputs() const {
        return scope_.get_inputs();
      }

      // Shorthand for getting input structures in scope filtered by enum type
      std::vector<Scope::InputPtr> get_inputs(const InputType type) const {
        return scope_.get_inputs(type);
      }

      // Shorthand for getting input structures filtered by type in scope
      template<typename T>
      std::vector<boost::shared_ptr<T>> get_inputs() const {
        return scope_.get_inputs<T>();
      }

      // Shorthand for getting input structures mapped to their names in scope
      std::map<std::string, const Scope::InputPtr&> get_input_map() const {
        return scope_.get_input_map();
      }

      // Shorthand for getting input structures mapped to their names in scope and filtered by their enum type
      std::map<std::string, const Scope::InputPtr&> get_input_map(const InputType type) const {
        return scope_.get_input_map(type);
      }

      // Shorthand for getting input structures mapped to their names and filtered by their types in scope
      template<typename T>
      std::map<std::string, boost::shared_ptr<T>> get_input_map() const {
        return scope_.get_input_map<T>();
      }

    private:
      giskard_core::QPProblemBuilder qp_builder_;
      qpOASES::SQProblem qp_problem_;
      Eigen::VectorXd xdot_full_, xdot_control_, xdot_slack_;
      std::vector<std::string> controllable_names_, soft_constraint_names_;
      giskard_core::Scope scope_;
  };

}

#endif // GISKARD_CORE_QP_CONTROLLER_HPP
