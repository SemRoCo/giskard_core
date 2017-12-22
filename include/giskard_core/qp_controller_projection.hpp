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
#ifndef GISKARD_CORE_QP_CONTROLLER_PROJECTION_HPP
#define GISKARD_CORE_QP_CONTROLLER_PROJECTION_HPP

#include <giskard_core/qp_controller.hpp>

namespace giskard_core
{
  class QPControllerProjectionParams
  {
    public:
      QPControllerProjectionParams(double period, const std::vector<std::string>& observable_names,
          const std::map<std::string, double>& convergence_thresholds, size_t min_num_trajectory_points,
          size_t max_num_trajectory_points, int nWSR) :
        period_(period), observable_names_(observable_names), convergence_thresholds_(convergence_thresholds),
        min_num_trajectory_points_(min_num_trajectory_points), max_num_trajectory_points_(max_num_trajectory_points),
        nWSR_(nWSR)
      {}

      void verify_sanity(const std::vector<std::string>& controllable_names) const
      {
        if (period_ <= 0)
          throw std::runtime_error("Requested projection period is not greater 0.");

        if (nWSR_ <= 0)
          throw std::runtime_error("Request nWSR is not greater 0.");

        if (max_num_trajectory_points_ < min_num_trajectory_points_)
          throw std::runtime_error("Requested maximum length of trajectory is smaller than requested minimum length.");

        if (controllable_names.size() > observable_names_.size())
          throw std::runtime_error("Number of controllables exceeds number of observables.");

        for (size_t i=0; i<controllable_names.size(); ++i)
          if (controllable_names[i].compare(observable_names_[i]) != 0)
            throw std::runtime_error("Controllable name at index " + std::to_string(i) + " does not match observable name at the same index.");

        if (convergence_thresholds_.count(default_joint_convergence_threshold_key()) == 0)
          for (auto const & controllable_name: controllable_names)
            if (convergence_thresholds_.count(controllable_name) == 0)
              throw std::runtime_error("Neither '" + default_joint_convergence_threshold_key() + " nor convergence " +
                                       "threshold for controllable '" + controllable_name + "' given.");
      }

      static std::string default_joint_convergence_threshold_key()
      {
        return "default_joint_convergence_threshold"  ;
      }

      double period_;
      std::vector<std::string> observable_names_;
      std::map<std::string, double> convergence_thresholds_;
      size_t min_num_trajectory_points_, max_num_trajectory_points_;
      int nWSR_;
  };

  class QPControllerProjection
  {
    public:
      QPControllerProjection(const QPController& controller, const QPControllerProjectionParams& params) :
          controller_(controller), params_(params)
      {
        params_.verify_sanity(get_controllable_names());
        init_convergence_thresholds(params.convergence_thresholds_);
      }

      void run(const std::map<std::string, double>& start_observables)
      {
        init_controller(start_observables);

        while ((!max_trajectory_length_reached()) && (!controller_converged()))
          sim_controller_once();

        verify_sane_trajectory();
      }

      const QPControllerProjectionParams& get_params() const
      {
        return params_;
      }

      const std::vector<std::string>& get_controllable_names() const
      {
        return controller_.get_controllable_names();
      }

      const std::vector<std::string>& get_observable_names() const
      {
        return params_.observable_names_;
      }

      const std::vector< Eigen::VectorXd >& get_position_trajectories() const
      {
        return position_trajectories_;
      }

      const std::vector< Eigen::VectorXd >& get_velocity_trajectories() const
      {
        return velocity_trajectories_;
      }

    protected:
      QPController controller_;
      QPControllerProjectionParams params_;
      std::vector< Eigen::VectorXd > position_trajectories_;
      std::vector< Eigen::VectorXd > velocity_trajectories_;
      Eigen::VectorXd state_;
      Eigen::VectorXd convergence_thresholds_;

      void init_convergence_thresholds(const std::map<std::string, double>& thresholds)
      {
        convergence_thresholds_.resize(get_controllable_names().size());
        for (size_t i=0; i<get_controllable_names().size(); ++i)
          if (thresholds.count(get_controllable_names()[i]) != 0)
            convergence_thresholds_(i) = thresholds.find(get_controllable_names()[i])->second;
          else if (thresholds.count(QPControllerProjectionParams::default_joint_convergence_threshold_key()) != 0)
            convergence_thresholds_(i) = thresholds.find(QPControllerProjectionParams::default_joint_convergence_threshold_key())->second;
          else
            throw std::runtime_error("No '" + QPControllerProjectionParams::default_joint_convergence_threshold_key() + "' given.");
      }

      void init_controller(const std::map<std::string, double>& start_observables)
      {
        position_trajectories_.clear();
        velocity_trajectories_.clear();

        state_.resize(get_observable_names().size());
        for (size_t i=0; i<get_observable_names().size(); ++i)
          if (start_observables.count(get_observable_names()[i]) == 0)
            throw std::runtime_error("Could not find value for observable '" + get_observable_names()[i] + "'.");
          else
            state_(i) = start_observables.find(get_observable_names()[i])->second;

        if (!controller_.start(state_, params_.nWSR_))
          throw std::runtime_error("Could not start QPController.");

        position_trajectories_.push_back(state_.segment(0, get_controllable_names().size()));
        velocity_trajectories_.push_back(Eigen::VectorXd::Zero(get_controllable_names().size()));
      }

      bool controller_converged() const
      {
        return min_trajectory_length_reached() && all_controllables_converged();
      }

      bool max_trajectory_length_reached() const
      {
        return get_position_trajectories().size() >= get_params().max_num_trajectory_points_;
      }

      bool min_trajectory_length_reached() const
      {
        return get_position_trajectories().size() >= get_params().min_num_trajectory_points_;
      }

      bool all_controllables_converged() const
      {
        using namespace Eigen;
//        VectorXd abs_delta = controller_.get_command().cwiseAbs() - get_thresholds().cwiseAbs();
        return ((controller_.get_command().array().abs() - get_thresholds().array().abs()) < 0).all();
      }

      void sim_controller_once()
      {
        if (!controller_.update(state_, params_.nWSR_))
          throw("Could not update QPController.");

        using Eigen::operator*;
        state_.segment(0, get_controllable_names().size()) += params_.period_ * controller_.get_command();
        position_trajectories_.push_back(state_.segment(0, get_controllable_names().size()));
        velocity_trajectories_.push_back(controller_.get_command());
      }

      void verify_sane_trajectory() const
      {
        if (get_position_trajectories().size() != get_velocity_trajectories().size())
          throw std::runtime_error("Number of trajectory points in position and velocity trajectories do not match.");

        for (const auto & position_point: get_position_trajectories())
          if (position_point.size() != get_controllable_names().size())
            throw std::runtime_error("Number of position values in one trajectory point does not match number of controllables.");

        for (const auto & velocity_point: get_velocity_trajectories())
          if (velocity_point.size() != get_controllable_names().size())
            throw std::runtime_error("Number of velocity values in one trajectory point does not match number of controllables.");
      }

      const Eigen::VectorXd& get_thresholds() const
      {
        return convergence_thresholds_;
      }
  };
}

#endif //GISKARD_CORE_QP_CONTROLLER_PROJECTION_HPP
