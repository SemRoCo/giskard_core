#ifndef GISKARD_SPECIFICATIONS_HPP
#define GISKARD_SPECIFICATIONS_HPP

#include <string>
#include <iostream>
#include <map>
#include <giskard/expressiontree.hpp>

namespace giskard
{
//  class ObservableSpec
//  {
//    public:
//      std::string name_, type_;
//
//      void clear()
//      {
//        name_ = type_ = "";
//      }
//  };
//
//  class ControllableSpec
//  {
//    public:
//      std::string name_, type_, reference_;
//
//      void clear()
//      {
//        name_ = type_ = reference_ = "";
//      }
//  };
//
//  class ExpressionSpec;
//
//  class ExpressionSpec
//  {
//    public:
//      std::string name_, type_;
//      double value_;
//      std::vector<ExpressionSpec> inputs_; 
//
//      void clear()
//      {
//        name_ = type_ = "";
//        value_ = 0.0;
//        inputs_.clear();
//      }
//  };
//
//  class ConstraintSpec
//  {
//    public:
//      std::string name_, expression_, type_;
//      double lower_, upper_, weight_, gain_;
//
//    void clear()
//    {
//      name_ = expression_ = type_ = "";
//      lower_ = upper_ = weight_ = gain_ = 0.0;
//    }
//  };
//
//  class ControllerSpec
//  {
//    public:
//      std::vector<ObservableSpec> observables_;
//      std::vector<ExpressionSpec> expressions_;
//      std::vector<ControllableSpec> controllables_;
//      std::vector<ConstraintSpec> constraints_;
//   
//    void clear()
//    {
//      observables_.clear();
//      expressions_.clear();
//      controllables_.clear();
//      constraints_.clear();
//    }
//  };
//
//  inline std::ostream& operator<<(std::ostream& os, const ObservableSpec& obs)
//  {
//    os << "name: " << obs.name_ << "\n";
//    os << "type: " << obs.type_ << "\n";
//
//    return os;
//  }
//
//  inline std::ostream& operator<<(std::ostream& os, const ControllableSpec& obs)
//  {
//    os << "name: " << obs.name_ << "\n";
//    os << "type: " << obs.type_ << "\n";
//    os << "reference: " << obs.reference_ << "\n";
//
//    return os;
//  }

  class Scope
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

  ///
  /// base of all specifications of expressions
  ///

  class Specification
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

  typedef typename boost::shared_ptr<Specification> SpecificationPtr;

  ///
  /// next level of expression specifications
  ///

  class DoubleSpecification : public Specification
  {
    public:
      KDL::Expression<double>::Ptr get_expression(const giskard::Scope& scope)
      {
        if(get_cached())
          return KDL::cached<double>(generate_expression(scope));
        else
          return generate_expression(scope);
      }

    private:
      virtual KDL::Expression<double>::Ptr generate_expression(const giskard::Scope& scope) = 0;
  };

  typedef typename boost::shared_ptr<DoubleSpecification> DoubleSpecificationPtr;

  class VectorSpecification : public Specification
  {
    public:
      KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard::Scope& scope)
      {
        if(get_cached())
          return KDL::cached<KDL::Vector>(generate_expression(scope));
        else
          return generate_expression(scope);
      }

    private:
      virtual KDL::Expression<KDL::Vector>::Ptr generate_expression(const giskard::Scope& scope) = 0;
  };

  typedef typename boost::shared_ptr<VectorSpecification> VectorSpecificationPtr;

  ///
  /// specifications of double expressions
  ///

  class ConstDoubleSpecification : public DoubleSpecification
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

      virtual KDL::Expression<double>::Ptr generate_expression(const giskard::Scope& scope)
      {
        return KDL::Constant(get_value());
      }
  };

  typedef typename boost::shared_ptr<ConstDoubleSpecification> ConstDoubleSpecificationPtr;

  class InputDoubleSpecification : public DoubleSpecification
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

      virtual KDL::Expression<double>::Ptr generate_expression(const giskard::Scope& scope)
      {
        return KDL::input(get_input_num());
      }
  };

  typedef typename boost::shared_ptr<InputDoubleSpecification> InputDoubleSpecificationPtr;

  class ReferenceDoubleSpecification : public DoubleSpecification
  {
    public:
      const std::string& get_reference_name() const
      {
        return reference_name_;
      }

      void set_reference_name(const std::string& reference_name)
      {
        reference_name_ = reference_name;
      }

    private:
      std::string reference_name_;

      virtual KDL::Expression<double>::Ptr generate_expression(const giskard::Scope& scope)
      {
        return scope.find_double_expression(get_reference_name());
      }
  };

  typedef typename boost::shared_ptr<ReferenceDoubleSpecification> ReferenceDoubleSpecificationPtr;

  class AdditionDoubleSpecification: public DoubleSpecification
  {
    public:
      const std::vector<DoubleSpecificationPtr>& get_inputs() const
      {
        return inputs_;
      }

      void set_inputs(const std::vector<DoubleSpecificationPtr>& inputs)
      {
        inputs_ = inputs;
      }

    private:
      std::vector<DoubleSpecificationPtr> inputs_;

      virtual KDL::Expression<double>::Ptr generate_expression(const giskard::Scope& scope)
      {
        KDL::Expression<double>::Ptr result = KDL::Constant(0.0);
        using KDL::operator+;
        for(size_t i=0; i<get_inputs().size(); ++i)
          result = result + get_inputs()[i]->get_expression(scope);
    
        return result;
      }
   };

  typedef typename boost::shared_ptr<AdditionDoubleSpecification> AdditionDoubleSpecificationPtr;

  ///
  /// specifications of vector expressions
  ///

  class ConstructorVectorSpecification: public VectorSpecification
  {
    public:
      const DoubleSpecificationPtr& get_x() const
      {
        return x_;
      }

      void set_x(const DoubleSpecificationPtr& x)
      {
        x_ = x;
      }

      const DoubleSpecificationPtr& get_y() const
      {
        return y_;
      }

      void set_y(const DoubleSpecificationPtr& y)
      {
        y_ = y;
      }

      const DoubleSpecificationPtr& get_z() const
      {
        return z_;
      }

      void set_z(const DoubleSpecificationPtr& z)
      {
        z_ = z;
      }
 
    private:
      DoubleSpecificationPtr x_, y_, z_;

      virtual KDL::Expression<KDL::Vector>::Ptr generate_expression(const giskard::Scope& scope)
      {
        return KDL::vector(get_x()->get_expression(scope), 
            get_y()->get_expression(scope), get_z()->get_expression(scope));
      }
   };

  typedef typename boost::shared_ptr<ConstructorVectorSpecification> ConstructorVectorSpecificationPtr;

}

#endif // GISKARD_SPECIFICATIONS_HPP
