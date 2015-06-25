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
      std::string name_, type_, reference_;

      void clear()
      {
        name_ = type_ = reference_ = "";
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
      std::string name_, expression_, type_;
      double lower_, upper_, weight_, gain_;

    void clear()
    {
      name_ = expression_ = type_ = "";
      lower_ = upper_ = weight_ = gain_ = 0.0;
    }
  };

  class ControllerSpec
  {
    public:
      std::vector<ObservableSpec> observables_;
      std::vector<ExpressionSpec> expressions_;
      std::vector<ControllableSpec> controllables_;
      std::vector<ConstraintSpec> constraints_;
   
    void clear()
    {
      observables_.clear();
      expressions_.clear();
      controllables_.clear();
      constraints_.clear();
    }
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
    os << "type: " << obs.type_ << "\n";
    os << "reference: " << obs.reference_ << "\n";

    return os;
  }
}

#endif // GISKARD_STRUCTS_HPP
