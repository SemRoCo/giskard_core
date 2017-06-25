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

#ifndef GISKARD_CORE_SCOPE_HPP
#define GISKARD_CORE_SCOPE_HPP

#include <string>
#include <map>
#include <stdexcept>
#include <giskard_core/expressiontree.hpp>

namespace giskard_core
{
  enum InputType {
    tScalar,
    tJoint,
    tVector3,
    tRotation,
    tFrame
  };

  class Scope
  {
    public:
      struct AInput {
        AInput(std::string name, size_t idx) : name_(name), idx_(idx) {}
        virtual InputType get_type() const = 0;
        const std::string name_;
        const size_t idx_;
      };

      struct JointInput : public AInput {
        JointInput(std::string name, size_t idx, KDL::Expression<double>::Ptr expr) 
          : AInput(name, idx), expr_(expr) {}
        
        static std::string type_string() { return "joint"; }
        InputType get_type() const { return tJoint; };
        const KDL::Expression<double>::Ptr expr_;
      };

      struct ScalarInput : public AInput {
        ScalarInput(std::string name, size_t idx, KDL::Expression<double>::Ptr expr) 
          : AInput(name, idx), expr_(expr) {}

        static std::string type_string() { return "scalar"; }
        InputType get_type() const { return tScalar; };
        const KDL::Expression<double>::Ptr expr_;
      };

      struct Vec3Input : public AInput {
        Vec3Input(std::string name, size_t idx, KDL::Expression<KDL::Vector>::Ptr expr) 
          : AInput(name, idx), expr_(expr) {}
        
        static std::string type_string() { return "vec3"; }
        InputType get_type() const { return tVector3; };
        const KDL::Expression<KDL::Vector>::Ptr expr_;
      };

      struct RotationInput : public AInput {
        RotationInput(std::string name, size_t idx, KDL::Expression<KDL::Rotation>::Ptr expr) 
          : AInput(name, idx), expr_(expr) {}
        
        static std::string type_string() { return "rotation"; }
        InputType get_type() const { return tRotation; };
        const KDL::Expression<KDL::Rotation>::Ptr expr_;
      };

      struct FrameInput : public AInput {
        FrameInput(std::string name, size_t idx, KDL::Expression<KDL::Frame>::Ptr expr) 
          : AInput(name, idx), expr_(expr) {}
        
        static std::string type_string() { return "frame"; }
        InputType get_type() const { return tFrame; };
        const KDL::Expression<KDL::Frame>::Ptr expr_;
      };
      
      typedef typename boost::shared_ptr<AInput> InputPtr;
      typedef typename boost::shared_ptr<const AInput> ConstInputPtr;
      typedef typename boost::shared_ptr<JointInput> JointInputPtr;
      typedef typename boost::shared_ptr<ScalarInput> ScalarInputPtr;
      typedef typename boost::shared_ptr<Vec3Input> Vec3InputPtr;
      typedef typename boost::shared_ptr<RotationInput> RotationInputPtr;
      typedef typename boost::shared_ptr<FrameInput> FrameInputPtr;

      Scope() 
      : bJointvectorCompleted(false)
      , nextInputIndex(0) {}

      const KDL::Expression<double>::Ptr& find_double_expression(const std::string& reference_name) const
      {
        if(!has_double_expression(reference_name))
          throw std::invalid_argument("Could not find double expression with name: "+ reference_name);

        return double_references_.find(reference_name)->second;
      }

      const KDL::Expression<KDL::Vector>::Ptr& find_vector_expression(const std::string& reference_name) const
      {
        if(!has_vector_expression(reference_name))
          throw std::invalid_argument("Could not find vector expression with name: "+ reference_name);

        return vector_references_.find(reference_name)->second;
      }

      const KDL::Expression<KDL::Rotation>::Ptr& find_rotation_expression(const std::string& reference_name) const
      {
        if(!has_rotation_expression(reference_name))
          throw std::invalid_argument("Could not find rotation expression with name: "+ reference_name);

        return rotation_references_.find(reference_name)->second;
      }

      const KDL::Expression<KDL::Frame>::Ptr& find_frame_expression(const std::string& reference_name) const
      {
        if(!has_frame_expression(reference_name))
          throw std::invalid_argument("Could not find frame expression with name: "+ reference_name);

        return frame_references_.find(reference_name)->second;
      }

      const InputPtr& find_input(const std::string& input_name) const {
        if (!has_input(input_name))
          throw std::invalid_argument("Could not find input with name: " + input_name);
        return inputs_.find(input_name)->second;
      }

      template<typename T>
      const boost::shared_ptr<T> find_input(const std::string& input_name) const {
        if (!has_input<T>(input_name))
          throw std::invalid_argument("Could not find input with name: " + input_name + " of type '" + T::type_string() + "'");
        return boost::dynamic_pointer_cast<T>(inputs_.find(input_name)->second);
      }

      bool has_double_expression(const std::string& expression_name) const
      {
        return (double_references_.find(expression_name) != double_references_.end());
      }

      bool has_vector_expression(const std::string& expression_name) const
      {
        return (vector_references_.find(expression_name) != vector_references_.end());
      }

      bool has_rotation_expression(const std::string& expression_name) const
      {
        return (rotation_references_.find(expression_name) != rotation_references_.end());
      }

      bool has_frame_expression(const std::string& expression_name) const
      {
        return (frame_references_.find(expression_name) != frame_references_.end());
      }

      bool has_input(const std::string& input_name) const {
        return inputs_.find(input_name) != inputs_.end();
      }

      template<typename T>
      bool has_input(const std::string& input_name) const {
        auto it = inputs_.find(input_name);
        if (it != inputs_.end()) {
          return !!boost::dynamic_pointer_cast<T>(it->second);
        }
        return false;
      }

      void add_double_expression(const std::string& reference_name, const KDL::Expression<double>::Ptr& expression)
      {
        if(has_double_expression(reference_name))
          throw std::invalid_argument("Could not add double expression to scope because name already taken: "
              + reference_name);

        double_references_[reference_name] = expression;
      }

      void add_vector_expression(const std::string& reference_name, const KDL::Expression<KDL::Vector>::Ptr& expression)
      {
        if(has_vector_expression(reference_name))
          throw std::invalid_argument("Could not add vector expression to scope because name already taken: "
              + reference_name);

        vector_references_[reference_name] = expression;
      }

      void add_rotation_expression(const std::string& reference_name, const KDL::Expression<KDL::Rotation>::Ptr& expression)
      {
        if(has_rotation_expression(reference_name))
          throw std::invalid_argument("Could not add rotation expression to scope because name already taken: "
              + reference_name);

        rotation_references_[reference_name] = expression;
      }

      void add_frame_expression(const std::string& reference_name, const KDL::Expression<KDL::Frame>::Ptr& expression)
      {
        if(has_frame_expression(reference_name))
          throw std::invalid_argument("Could not add frame expression to scope because name already taken: "
              + reference_name);

        frame_references_[reference_name] = expression;
      }

      void add_joint_input(const std::string& name) {
        auto it = inputs_.find(name);
        if (it != inputs_.end()) {
          if (it->second->get_type() != tJoint)
            throw std::invalid_argument("Can't add joint input with name '" + name + "'. The name is already taken.");
        } else {
          if (bJointvectorCompleted)
            throw std::invalid_argument("Can't add joint input with name '" + name + "'. Joints always need to be the first inputs in the controller.");


          KDL::Expression<double>::Ptr expr = KDL::input(nextInputIndex);
          inputs_[name] = JointInputPtr(new JointInput(name, nextInputIndex, expr));
          nextInputIndex++;
        }
      }

      void add_scalar_input(const std::string& name) {
        auto it = inputs_.find(name);
        if (it != inputs_.end()) {
          if (it->second->get_type() != tScalar)
            throw std::invalid_argument("Can't add scalar input with name '" + name + "'. The name is already taken.");
        } else {
          bJointvectorCompleted = true;

          KDL::Expression<double>::Ptr expr = KDL::input(nextInputIndex);
          inputs_[name] = ScalarInputPtr(new ScalarInput(name, nextInputIndex, expr));
          nextInputIndex++;
        }
      }

      void add_vector_input(const std::string& name) {
        auto it = inputs_.find(name);
        if (it != inputs_.end()) {
          if (it->second->get_type() != tVector3)
            throw std::invalid_argument("Can't add vector input with name '" + name + "'. The name is already taken.");
        } else {
          bJointvectorCompleted = true;
          KDL::Expression<KDL::Vector>::Ptr expr = KDL::vector(KDL::input(nextInputIndex), 
                                                         KDL::input(nextInputIndex + 1), 
                                                         KDL::input(nextInputIndex + 2));
          inputs_[name] = Vec3InputPtr(new Vec3Input(name, nextInputIndex, expr));
          nextInputIndex += 3;
        }
      }

      void add_rotation_input(const std::string& name) {
        auto it = inputs_.find(name);
        if (it != inputs_.end()) {
          if (it->second->get_type() != tRotation)
            throw std::invalid_argument("Can't add rotation input with name '" + name + "'. The name is already taken.");
        } else {
          bJointvectorCompleted = true;
          KDL::Expression<KDL::Rotation>::Ptr expr = KDL::rotVec(KDL::vector(KDL::input(nextInputIndex), 
                                                                     KDL::input(nextInputIndex + 1), 
                                                                     KDL::input(nextInputIndex + 2)),
                                                         KDL::input(nextInputIndex + 3));
          inputs_[name] = RotationInputPtr(new RotationInput(name, nextInputIndex, expr));
          nextInputIndex += 4;
        }
      }

      void add_frame_input(const std::string& name) {
        auto it = inputs_.find(name);
        if (it != inputs_.end()) {
          if (it->second->get_type() != tFrame)
            throw std::invalid_argument("Can't add frame input with name '" + name + "'. The name is already taken.");
        } else {
          bJointvectorCompleted = true;
          KDL::Expression<KDL::Frame>::Ptr expr = KDL::frame(KDL::rotVec(KDL::vector(KDL::input(nextInputIndex), 
                                                                                    KDL::input(nextInputIndex + 1), 
                                                                                    KDL::input(nextInputIndex + 2)),
                                                                        KDL::input(nextInputIndex + 3)),
                                                            KDL::vector(KDL::input(nextInputIndex + 4),
                                                                        KDL::input(nextInputIndex + 5),
                                                                        KDL::input(nextInputIndex + 6)));
          inputs_[name] = FrameInputPtr(new FrameInput(name, nextInputIndex, expr));
          nextInputIndex += 7;
        }
      }

      // Get the size of the observable vector
      size_t get_input_size() const {
        return nextInputIndex;
      }

      std::vector<std::string> get_double_names() const
      {
        std::vector<std::string> result;
        for (auto it=double_references_.begin(); it!=double_references_.end(); it++)
          result.push_back(it->first);
        return result;
      }

      std::vector<std::string> get_vector_names() const
      {
        std::vector<std::string> result;
        for (auto it=vector_references_.begin(); it!=vector_references_.end(); it++)
          result.push_back(it->first);
        return result;
      }

      std::vector<std::string> get_rotation_names() const
      {
        std::vector<std::string> result;
        for (auto it=rotation_references_.begin(); it!=rotation_references_.end(); it++)
          result.push_back(it->first);
        return result;
      }

      std::vector<std::string> get_frame_names() const
      {
        std::vector<std::string> result;
        for (auto it=frame_references_.begin(); it!=frame_references_.end(); it++)
          result.push_back(it->first);
        return result;
      }

      // Get the names of all inputs
      std::vector<std::string> get_input_names() const {
        std::vector<std::string> result;
        for (auto it = inputs_.begin(); it != inputs_.end(); it++)
          result.push_back(it->first);
        return result;
      }

      // Get the names of all inputs of a specific enum type
      std::vector<std::string> get_input_names(const InputType type) const {
        std::vector<std::string> result;
        for (auto it = inputs_.begin(); it != inputs_.end(); it++)
          if (it->second->get_type() == type)
            result.push_back(it->first);
        return result;
      }

      // Get the names of all inputs of a specific type
      template<typename T>
      std::vector<std::string> get_input_names() const {
        std::vector<std::string> result;
        for (auto it = inputs_.begin(); it != inputs_.end(); it++)
          if (!!boost::dynamic_pointer_cast<T>(it->second))
            result.push_back(it->first);
        return result;
      }

      // Get all input structures
      std::vector<InputPtr> get_inputs() const {
        std::vector<InputPtr> out;
        for (auto it = inputs_.begin(); it != inputs_.end(); it++) {
          out.push_back(it->second);
        }
        return out;
      }

      // Get input structures filtered by enum type
      std::vector<InputPtr> get_inputs(const InputType type) const {
        std::vector<InputPtr> out;
        for (auto it = inputs_.begin(); it != inputs_.end(); it++) {
          if (it->second->get_type() == type)
            out.push_back(it->second);
        }
        return out;
      }

      // Get input structures filtered by type
      template<typename T>
      std::vector<boost::shared_ptr<T>> get_inputs() const {
        std::vector<boost::shared_ptr<T>> out;
        for (auto it = inputs_.begin(); it != inputs_.end(); it++) {
          boost::shared_ptr<T> temp = boost::dynamic_pointer_cast<T>(it->second);
          if (temp)
            out.push_back(temp);
        }
        return out;
      }

      // Get input structures mapped by their names
      std::map<std::string, const InputPtr&> get_input_map() const {
        std::map<std::string, const InputPtr&> out;
        for (auto it = inputs_.begin(); it != inputs_.end(); it++)
          out.insert(std::pair<std::string, const InputPtr&>(it->first, it->second));

        return out;
      }

      // Get input structures mapped by their names and filtered by their enum types
      std::map<std::string, const InputPtr&> get_input_map(const InputType type) const {
        std::map<std::string, const InputPtr&> out;
        for (auto it = inputs_.begin(); it != inputs_.end(); it++)
          if (it->second->get_type() == type)
            out.insert(std::pair<std::string, const InputPtr&>(it->first, it->second));

        return out;
      }

      // Get input structures mapped by their names and filtered by their types
      template<typename T>
      std::map<std::string, boost::shared_ptr<T>> get_input_map() const {
        std::map<std::string, boost::shared_ptr<T>> out;
        for (auto it = inputs_.begin(); it != inputs_.end(); it++) {
          boost::shared_ptr<T> temp = boost::dynamic_pointer_cast<T>(it->second);
          if (temp)
            out[it->first] = temp;
        }
        return out;
      }

private:
      bool bJointvectorCompleted;
      size_t nextInputIndex;
      std::map<std::string, InputPtr> inputs_;

      std::map< std::string, KDL::Expression<double>::Ptr > double_references_;
      std::map< std::string, KDL::Expression<KDL::Vector>::Ptr > vector_references_;
      std::map< std::string, KDL::Expression<KDL::Rotation>::Ptr > rotation_references_;
      std::map< std::string, KDL::Expression<KDL::Frame>::Ptr > frame_references_;
  };
}

#endif // GISKARD_CORE_SCOPE_HPP
