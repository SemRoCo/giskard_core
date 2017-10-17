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

#ifndef GISKARD_CORE_ROBOT_HPP
#define GISKARD_CORE_ROBOT_HPP

#include <urdf_model/model.h>
#include <giskard_core/specifications.hpp>
#include <set>


namespace giskard_core
{
  class Robot
  {
    public:
      Robot(const urdf::Model& robot_model, const std::string& root,
          const std::vector<std::string>& tip_links)
      {
        // TODO: implement me

      }

      FrameSpecPtr get_kinematic_chain(const std::string& root, const std::string& tip) const
      {
        // TODO: implement me
        return frame_constructor_spec();
      }

      std::vector<ControllableConstraintSpecPtr> get_controllable_constraints() const
      {
        // TODO: implement me
        return std::vector<ControllableConstraintSpecPtr>();
      }

      std::vector<HardConstraintSpecPtr> get_hard_constraints() const
      {
        // TODO: implement me
        return std::vector<HardConstraintSpecPtr>();
      }

      std::vector<ScopeEntry> get_scope() const
      {
        // TODO: implement me
        return std::vector<ScopeEntry>();
      }

    protected:
      // TODO: add some useful members

      void init_kinematic_chain(const std::string& root, const std::string& tip)
      {
        // TODO: implement me
      }

  };
}

#endif // GISKARD_CORE_ROBOT_HPP
