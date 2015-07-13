#ifndef GISKARD_SPECIFICATIONS_HPP
#define GISKARD_SPECIFICATIONS_HPP

#include <string>
#include <iostream>
#include <map>
#include <giskard/expressiontree.hpp>

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

  class ExpressionScope
  {
    public:
      const KDL::Expression<double>::Ptr& find_double_expression(const std::string& reference_name) const
      {
        // TODO: throw exception here
        assert(has_double_expression(reference_name));

        std::map< std::string, KDL::Expression<double>::Ptr >::const_iterator it =
            double_references_.find(reference_name);

        return it->second;
      }

      const KDL::Expression<KDL::Frame>::Ptr& find_frame_expression(const std::string& reference_name) const
      {
        // TODO: throw exception here
        assert(has_frame_expression(reference_name));

        std::map< std::string, KDL::Expression<KDL::Frame>::Ptr >::const_iterator it =
            frame_references_.find(reference_name);

        return it->second;
      }

      bool has_double_expression(const std::string& expression_name) const
      {
        return (double_references_.count(expression_name) == 1);
      }

      bool has_frame_expression(const std::string& expression_name) const
      {
        return (frame_references_.count(expression_name) == 1);
      }

      void add_double_expression(const std::string& reference_name, const KDL::Expression<double>::Ptr& expression)
      {
        // TODO: throw warning here
        assert(!has_double_expression(reference_name));

        double_references_[reference_name] = expression;
      }

      void add_frame_expression(const std::string& reference_name, const KDL::Expression<KDL::Frame>::Ptr& expression)
      {
        // TODO: throw warning here
        assert(!has_frame_expression(reference_name));

        frame_references_[reference_name] = expression;
      }

    private:
      std::map< std::string, KDL::Expression<double>::Ptr > double_references_;
      std::map< std::string, KDL::Expression<KDL::Frame>::Ptr > frame_references_;
  };

  class ExpressionDescription
  { 
    public:

      const std::string& get_name() const
      {
        return name_;
      }

      void set_name(const std::string& name) 
      {
        name_ = name;
      }

      bool get_cached() const
      {
        return cached_;
      }
 
      void set_cached(bool cached)
      {
        cached_ = cached;
      }

    private:
      std::string name_;
      bool cached_;
  };

  typedef typename boost::shared_ptr<ExpressionDescription> ExpressionDescriptionPtr;

  class DoubleExpressionDescription : public ExpressionDescription
  {
    public:
      KDL::Expression<double>::Ptr get_expression(const giskard::ExpressionScope& scope)
      {
        if(get_cached())
          return KDL::cached<double>(generate_expression(scope));
        else
          return generate_expression(scope);
      }

    private:
      virtual KDL::Expression<double>::Ptr generate_expression(const giskard::ExpressionScope& scope) = 0;
  };

  typedef typename boost::shared_ptr<DoubleExpressionDescription> DoubleExpressionDescriptionPtr;

  class ConstDoubleExpressionDescription : public DoubleExpressionDescription
  {
    public:
      double get_value() const
      {
        return value_;
      }

      void set_value(double value)
      {
        value_ = value;
      } 

    private:
      double value_;

      virtual KDL::Expression<double>::Ptr generate_expression(const giskard::ExpressionScope& scope)
      {
        return KDL::Constant(get_value());
      }
  };

  typedef typename boost::shared_ptr<ConstDoubleExpressionDescription> ConstDoubleExpressionDescriptionPtr;

  class InputDoubleExpressionDescription : public DoubleExpressionDescription
  {
    public:
      size_t get_input_num() const
      {
        return input_num_;
      }

      void set_input_num(size_t input_num)
      {
        input_num_ = input_num;
      }

    private:
      size_t input_num_;

      virtual KDL::Expression<double>::Ptr generate_expression(const giskard::ExpressionScope& scope)
      {
        return KDL::input(get_input_num());
      }
  };

  typedef typename boost::shared_ptr<InputDoubleExpressionDescription> InputDoubleExpressionDescriptionPtr;

  class AdditionDoubleExpressionDescription: public DoubleExpressionDescription
  {
    public:
      const std::vector<DoubleExpressionDescriptionPtr>& get_inputs() const
      {
        return inputs_;
      }

      void set_inputs(const std::vector<DoubleExpressionDescriptionPtr>& inputs)
      {
        inputs_ = inputs;
      }

    private:
      std::vector<DoubleExpressionDescriptionPtr> inputs_;

      virtual KDL::Expression<double>::Ptr generate_expression(const giskard::ExpressionScope& scope)
      {
        KDL::Expression<double>::Ptr result = KDL::Constant(0.0);
        using KDL::operator+;
        for(size_t i=0; i<get_inputs().size(); ++i)
          result = result + get_inputs()[i]->get_expression(scope);
    
        return result;
      }
   };

  typedef typename boost::shared_ptr<AdditionDoubleExpressionDescription> AdditionDoubleExpressionDescriptionPtr;
}

#endif // GISKARD_SPECIFICATIONS_HPP
