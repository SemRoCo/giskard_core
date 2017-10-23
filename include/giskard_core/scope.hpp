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
  class Scope
  {
    public:
      const KDL::Expression<double>::Ptr& find_double_expression(const std::string& reference_name) const
      {
        if(!has_double_expression(reference_name))
          throw std::invalid_argument("Could not find double expression with name: "+ reference_name);

        std::map< std::string, KDL::Expression<double>::Ptr >::const_iterator it =
            double_references_.find(reference_name);

        return it->second;
      }

      const KDL::Expression<KDL::Vector>::Ptr& find_vector_expression(const std::string& reference_name) const
      {
        if(!has_vector_expression(reference_name))
          throw std::invalid_argument("Could not find vector expression with name: "+ reference_name);

        std::map< std::string, KDL::Expression<KDL::Vector>::Ptr >::const_iterator it =
            vector_references_.find(reference_name);

        return it->second;
      }

      const KDL::Expression<KDL::Rotation>::Ptr& find_rotation_expression(const std::string& reference_name) const
      {
        if(!has_rotation_expression(reference_name))
          throw std::invalid_argument("Could not find rotation expression with name: "+ reference_name);


        std::map< std::string, KDL::Expression<KDL::Rotation>::Ptr >::const_iterator it =
            rotation_references_.find(reference_name);

        return it->second;
      }

      const KDL::Expression<KDL::Frame>::Ptr& find_frame_expression(const std::string& reference_name) const
      {
        if(!has_frame_expression(reference_name))
          throw std::invalid_argument("Could not find frame expression with name: "+ reference_name);

        std::map< std::string, KDL::Expression<KDL::Frame>::Ptr >::const_iterator it =
            frame_references_.find(reference_name);

        return it->second;
      }

      const KDL::ExpressionBase::Ptr find_expression(const std::string& reference_name) const
      {
        if(!has_expression(reference_name))
          throw std::invalid_argument("Could not find expression with name: "+ reference_name);

        if (has_double_expression(reference_name))
          return find_double_expression(reference_name);
        else if (has_vector_expression(reference_name))
          return find_vector_expression(reference_name);
        else if (has_rotation_expression(reference_name))
          return find_rotation_expression(reference_name);
        else if (has_frame_expression(reference_name))
          return find_frame_expression(reference_name);
        else 
          throw std::runtime_error("Could not infer type of expression with name '" + reference_name + "'.");
      }

      bool has_double_expression(const std::string& expression_name) const
      {
        return (double_references_.count(expression_name) == 1);
      }

      bool has_vector_expression(const std::string& expression_name) const
      {
        return (vector_references_.count(expression_name) == 1);
      }

      bool has_rotation_expression(const std::string& expression_name) const
      {
        return (rotation_references_.count(expression_name) == 1);
      }

      bool has_frame_expression(const std::string& expression_name) const
      {
        return (frame_references_.count(expression_name) == 1);
      }

      bool has_expression (const std::string& expression_name) const
      {
        return has_double_expression(expression_name) ||
          has_vector_expression(expression_name) || 
          has_rotation_expression(expression_name) ||
          has_frame_expression(expression_name);
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

    private:
      std::map< std::string, KDL::Expression<double>::Ptr > double_references_;
      std::map< std::string, KDL::Expression<KDL::Vector>::Ptr > vector_references_;
      std::map< std::string, KDL::Expression<KDL::Rotation>::Ptr > rotation_references_;
      std::map< std::string, KDL::Expression<KDL::Frame>::Ptr > frame_references_;
  };
}

#endif // GISKARD_CORE_SCOPE_HPP
