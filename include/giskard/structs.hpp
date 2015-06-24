#ifndef GISKARD_STRUCTS_HPP
#define GISKARD_STRUCTS_HPP

#include <string>
#include <iostream>

namespace giskard
{
  class ObservableSpec
  {
    public:
      std::string name_, type_;

      void clear()
      {
        name_ = type_ = "";
      }
  };

  class ControllableSpec
  {
    public:
      std::string name_;
      double lower_vel_limit_, upper_vel_limit_, weight_;

      void clear()
      {
        name_ = "";
        lower_vel_limit_ = upper_vel_limit_ = weight_ = 0.0;
      }
  };

  class ExpressionSpec;

  class ExpressionSpec
  {
    public:
      std::string name_, type_;
      double value_;
      std::vector<ExpressionSpec> inputs_; 

      void clear()
      {
        name_ = type_ = "";
        value_ = 0.0;
        inputs_.clear();
      }
  };

  class ConstraintSpec
  {
    public:
      std::string name_, expression_;
      double lower_, upper_, weight_, p_gain_;
  };

  inline std::ostream& operator<<(std::ostream& os, const ObservableSpec& obs)
  {
    os << "name: " << obs.name_ << "\n";
    os << "type: " << obs.type_ << "\n";

    return os;
  }

  inline std::ostream& operator<<(std::ostream& os, const ControllableSpec& obs)
  {
    os << "name: " << obs.name_ << "\n";
    os << "lower_velocity_limit:: " << obs.lower_vel_limit_ << "\n";
    os << "weight:: " << obs.upper_vel_limit_ << "\n";
    os << "weight:: " << obs.weight_ << "\n";

    return os;
  }

}

#endif // GISKARD_STRUCTS_HPP
