#ifndef GISKARD_SPECIFICATIONS_HPP
#define GISKARD_SPECIFICATIONS_HPP

#include <string>
#include <iostream>
#include <map>
#include <giskard/expressiontree.hpp>
#include <boost/lexical_cast.hpp>

namespace giskard
{
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

      const KDL::Expression<KDL::Vector>::Ptr& find_vector_expression(const std::string& reference_name) const
      {
        // TODO: throw exception here
        assert(has_vector_expression(reference_name));

        std::map< std::string, KDL::Expression<KDL::Vector>::Ptr >::const_iterator it =
            vector_references_.find(reference_name);

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

      bool has_vector_expression(const std::string& expression_name) const
      {
        return (vector_references_.count(expression_name) == 1);
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

      void add_vector_expression(const std::string& reference_name, const KDL::Expression<KDL::Vector>::Ptr& expression)
      {
        // TODO: throw warning here
        assert(!has_vector_expression(reference_name));

        vector_references_[reference_name] = expression;
      }

      void add_frame_expression(const std::string& reference_name, const KDL::Expression<KDL::Frame>::Ptr& expression)
      {
        // TODO: throw warning here
        assert(!has_frame_expression(reference_name));

        frame_references_[reference_name] = expression;
      }

    private:
      std::map< std::string, KDL::Expression<double>::Ptr > double_references_;
      std::map< std::string, KDL::Expression<KDL::Vector>::Ptr > vector_references_;
      std::map< std::string, KDL::Expression<KDL::Frame>::Ptr > frame_references_;
  };

  ///
  /// base of all specifications of expressions
  ///

  class Spec
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

      virtual void clear()
      {
        set_name("");
        set_cached(false);
      }

      virtual bool equals(const Spec& other) const = 0;

      // TODO: extend this with a parameter for indention
      virtual std::string to_string() const = 0;

    private:
      std::string name_;
      bool cached_;
  };

  inline bool operator==(const Spec& lhs, const Spec& rhs)
  {
    return lhs.equals(rhs);
  }

  inline bool operator!=(const Spec& lhs, const Spec& rhs)
  {
    return !operator==(lhs,rhs);
  }

  inline std::ostream& operator<<(std::ostream& os, const Spec& spec)
  {
    os << spec.to_string();
    return os;
  }

  typedef typename boost::shared_ptr<Spec> SpecPtr;

  ///
  /// next level of expression specifications
  ///

  class DoubleSpec : public Spec
  {
    public:
      KDL::Expression<double>::Ptr get_expression(const giskard::Scope& scope)
      {
        if(get_cached())
          return KDL::cached<double>(generate_expression(scope));
        else
          return generate_expression(scope);
      }

      virtual bool equals(const Spec& other) const = 0;

      virtual std::string to_string() const = 0;

    private:
      virtual KDL::Expression<double>::Ptr generate_expression(const giskard::Scope& scope) = 0;
  };

  typedef typename boost::shared_ptr<DoubleSpec> DoubleSpecPtr;

  class VectorSpec : public Spec
  {
    public:
      KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard::Scope& scope)
      {
        if(get_cached())
          return KDL::cached<KDL::Vector>(generate_expression(scope));
        else
          return generate_expression(scope);
      }

      virtual bool equals(const Spec& other) const = 0;

      virtual std::string to_string() const = 0;

    private:
      virtual KDL::Expression<KDL::Vector>::Ptr generate_expression(const giskard::Scope& scope) = 0;
  };

  typedef typename boost::shared_ptr<VectorSpec> VectorSpecPtr;

  class RotationSpec : public Spec
  {
    public:
      KDL::Expression<KDL::Rotation>::Ptr get_expression(const giskard::Scope& scope)
      {
        if(get_cached())
          return KDL::cached<KDL::Rotation>(generate_expression(scope));
        else
          return generate_expression(scope);
      }

      virtual bool equals(const Spec& other) const = 0;

      virtual std::string to_string() const = 0;

    private:
      virtual KDL::Expression<KDL::Rotation>::Ptr generate_expression(const giskard::Scope& scope) = 0;
  };

  typedef typename boost::shared_ptr<RotationSpec> RotationSpecPtr;

  class FrameSpec : public Spec
  {
    public:
      KDL::Expression<KDL::Frame>::Ptr get_expression(const giskard::Scope& scope)
      {
        if(get_cached())
          return KDL::cached<KDL::Frame>(generate_expression(scope));
        else
          return generate_expression(scope);
      }

      virtual bool equals(const Spec& other) const = 0;

      virtual std::string to_string() const = 0;

    private:
      virtual KDL::Expression<KDL::Frame>::Ptr generate_expression(const giskard::Scope& scope) = 0;
  };

  typedef typename boost::shared_ptr<FrameSpec> FrameSpecPtr;

  ///
  /// specifications of double expressions
  ///

  class ConstDoubleSpec : public DoubleSpec
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

      virtual void clear()
      {
        Spec::clear();
        set_value(0.0);
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const ConstDoubleSpec*>(&other))
          return false;

        return KDL::epsilon >
            std::abs(dynamic_cast<const ConstDoubleSpec*>(&other)->get_value() - this->get_value());
      }

      virtual std::string to_string() const
      {
        return boost::lexical_cast<std::string>(get_value());
      }

    private:
      double value_;

      virtual KDL::Expression<double>::Ptr generate_expression(const giskard::Scope& scope)
      {
        return KDL::Constant(get_value());
      }
  };

  typedef typename boost::shared_ptr<ConstDoubleSpec> ConstDoubleSpecPtr;

  class InputDoubleSpec : public DoubleSpec
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

      virtual void clear()
      {
        Spec::clear();
        set_input_num(0);
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const InputDoubleSpec*>(&other))
          return false;

        return dynamic_cast<const InputDoubleSpec*>(&other)->get_input_num() == this->get_input_num();
      }

      virtual std::string to_string() const
      {
        return "todo: implement me";
      }

    private:
      size_t input_num_;

      virtual KDL::Expression<double>::Ptr generate_expression(const giskard::Scope& scope)
      {
        return KDL::input(get_input_num());
      }
  };

  typedef typename boost::shared_ptr<InputDoubleSpec> InputDoubleSpecPtr;

  class ReferenceDoubleSpec : public DoubleSpec
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

      virtual void clear()
      {
        Spec::clear();
        set_reference_name("");
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const ReferenceDoubleSpec*>(&other))
          return false;

        return (dynamic_cast<const ReferenceDoubleSpec*>(&other)->get_reference_name().compare(this->get_reference_name()) == 0);
      }

      virtual std::string to_string() const
      {
        return "todo: implement me";
      }

    private:
      std::string reference_name_;

      virtual KDL::Expression<double>::Ptr generate_expression(const giskard::Scope& scope)
      {
        return scope.find_double_expression(get_reference_name());
      }
  };

  typedef typename boost::shared_ptr<ReferenceDoubleSpec> ReferenceDoubleSpecPtr;

  class AdditionDoubleSpec: public DoubleSpec
  {
    public:
      const std::vector<DoubleSpecPtr>& get_inputs() const
      {
        return inputs_;
      }

      void set_inputs(const std::vector<DoubleSpecPtr>& inputs)
      {
        inputs_ = inputs;
      }

      virtual void clear()
      {
        Spec::clear();
        set_inputs(std::vector<DoubleSpecPtr>());
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const AdditionDoubleSpec*>(&other))
          return false;

        const AdditionDoubleSpec* other_p = dynamic_cast<const AdditionDoubleSpec*>(&other);

        if(get_inputs().size() != other_p->get_inputs().size())
          return false;

        if(!inputs_valid() || !other_p->inputs_valid())
          return false;

        for(size_t i=0; i<get_inputs().size(); ++i)
          if(get_inputs()[i] != other_p->get_inputs()[i])
            return false;
        
        return true;
      }

      bool inputs_valid() const
      {
        for(size_t i=0; i<get_inputs().size(); ++i)
          if(!get_inputs()[i].get())
            return false;

        return true;
      }

      virtual std::string to_string() const
      {
        return "todo: implement me";
      }

    private:
      std::vector<DoubleSpecPtr> inputs_;

      virtual KDL::Expression<double>::Ptr generate_expression(const giskard::Scope& scope)
      {
        KDL::Expression<double>::Ptr result = KDL::Constant(0.0);
        using KDL::operator+;
        for(size_t i=0; i<get_inputs().size(); ++i)
          result = result + get_inputs()[i]->get_expression(scope);
    
        return result;
      }
   };

  typedef typename boost::shared_ptr<AdditionDoubleSpec> AdditionDoubleSpecPtr;

  ///
  /// specifications of vector expressions
  ///

  class ConstructorVectorSpec: public VectorSpec
  {
    public:
      const DoubleSpecPtr& get_x() const
      {
        return x_;
      }

      void set_x(const DoubleSpecPtr& x)
      {
        x_ = x;
      }

      const DoubleSpecPtr& get_y() const
      {
        return y_;
      }

      void set_y(const DoubleSpecPtr& y)
      {
        y_ = y;
      }

      const DoubleSpecPtr& get_z() const
      {
        return z_;
      }

      void set_z(const DoubleSpecPtr& z)
      {
        z_ = z;
      }

      void set(const DoubleSpecPtr& x, const DoubleSpecPtr& y, 
          const DoubleSpecPtr& z)
      {
        set_x(x);
        set_y(y);
        set_z(z);
      }

      virtual void clear()
      {
        Spec::clear();
        set(DoubleSpecPtr(), DoubleSpecPtr(), DoubleSpecPtr());
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const ConstructorVectorSpec*>(&other))
          return false;

        const ConstructorVectorSpec* other_p = dynamic_cast<const ConstructorVectorSpec*>(&other);
        
        if(!members_valid() || !other_p->members_valid())
          return false;

        return (get_x()->equals(*(other_p->get_x()))) && (get_y()->equals(*(other_p->get_y()))) &&
            (get_z()->equals(*(other_p->get_z())));
      }

      bool members_valid() const
      {
        return get_x().get() && get_y().get() && get_z().get();
      }

      virtual std::string to_string() const
      {
        std::string result = "type: VECTOR3(";
        result += get_x()->to_string() + ", ";
        result += get_y()->to_string() + ", ";
        result += get_z()->to_string() + ")";
        return result;
      }

    private:
      DoubleSpecPtr x_, y_, z_;

      virtual KDL::Expression<KDL::Vector>::Ptr generate_expression(const giskard::Scope& scope)
      {
        return KDL::vector(get_x()->get_expression(scope), 
            get_y()->get_expression(scope), get_z()->get_expression(scope));
      }
  };

  typedef typename boost::shared_ptr<ConstructorVectorSpec> ConstructorVectorSpecPtr;
 
  ///
  /// specifications for rotation expresssions
  ///

  class AxisAngleSpec: public RotationSpec
  {
    public:
      const VectorSpecPtr& get_axis() const
      {
        return axis_;
      }

      void set_axis(const VectorSpecPtr& axis)
      {
        axis_ = axis;
      }

      const DoubleSpecPtr& get_angle() const
      {
        return angle_;
      }

      void set_angle(const DoubleSpecPtr& angle)
      {
        angle_ = angle;
      }

      virtual void clear()
      {
        Spec::clear();
        set_angle(DoubleSpecPtr());
        set_axis(VectorSpecPtr());
      }

      bool members_valid() const
      {
        return get_axis().get() && get_angle().get();
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const AxisAngleSpec*>(&other))
          return false;

        const AxisAngleSpec* other_p = dynamic_cast<const AxisAngleSpec*>(&other);

        if(!members_valid() || !other_p->members_valid())
          return false;

        return (get_angle()->equals(*(other_p->get_angle()))) && 
               (get_axis()->equals(*( other_p->get_axis())));
      }

      virtual std::string to_string() const
      {
        std::string result = "type: ROTATION\naxis:\n";
        result += get_axis()->to_string() + "\nangle:\n";
        result += get_angle()->to_string();
        return result;
      }

    private:
      VectorSpecPtr axis_;
      DoubleSpecPtr angle_;

      virtual KDL::Expression<KDL::Rotation>::Ptr generate_expression(const giskard::Scope& scope)
      {
        // note: this type of rotation expressions only expect expressions for their angle, not
        //       the angle. while this my make sense, it does break code symmetry.
        KDL::Expression<double>::Ptr angle = get_angle()->get_expression(scope);
        KDL::Vector axis = get_axis()->get_expression(scope)->value();

        return KDL::rot(axis, angle);
      }
  };

  typedef typename boost::shared_ptr<AxisAngleSpec> AxisAngleSpecPtr;

  ///
  /// specifications for frame expresssions
  ///

  class ConstructorFrameSpec: public FrameSpec
  {
    public:
      const giskard::VectorSpecPtr& get_translation() const
      {
        return translation_;
      }

      void set_translation(const giskard::VectorSpecPtr& translation)
      {
        translation_ = translation;
      }

      const giskard::RotationSpecPtr& get_rotation() const
      {
        return rotation_;
      }

      void set_rotation(const giskard::RotationSpecPtr& rotation)
      {
        rotation_ = rotation;
      }

      virtual void clear()
      {
        Spec::clear();
        set_rotation(RotationSpecPtr());
        set_translation(VectorSpecPtr());
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const ConstructorFrameSpec*>(&other))
          return false;

        const ConstructorFrameSpec* other_p = dynamic_cast<const ConstructorFrameSpec*>(&other);

        if(!members_valid() || !other_p->members_valid())
          return false;
        
        return (get_translation()->equals(*(other_p->get_translation()))) && 
            (get_rotation()->equals(*(other_p->get_rotation())));
      }

      bool members_valid() const
      {
        return get_translation().get() && get_rotation().get();
      }

      virtual std::string to_string() const
      {
        std::string result = "type: FRAME\ntranslation:\n";
        result += get_translation()->to_string() + "\nrotation:\n";
        result += get_rotation()->to_string();
        return result;
      }

    private:
      VectorSpecPtr translation_;
      RotationSpecPtr rotation_;

      virtual KDL::Expression<KDL::Frame>::Ptr generate_expression(const giskard::Scope& scope)
      {
        KDL::Expression<KDL::Rotation>::Ptr rot = get_rotation()->get_expression(scope);

        KDL::Expression<KDL::Vector>::Ptr trans = get_translation()->get_expression(scope);
        return KDL::frame(rot, trans);
      }
  };

  typedef typename boost::shared_ptr<ConstructorFrameSpec> ConstructorFrameSpecPtr;

  class MultiplicationFrameSpec: public FrameSpec
  {
    public:
      const std::vector<FrameSpecPtr>& get_inputs() const
      {
        return inputs_;
      }

      void set_inputs(const std::vector<FrameSpecPtr>& inputs)
      {
        inputs_ = inputs;
      }

      virtual void clear()
      {
        Spec::clear();
        set_inputs(std::vector<FrameSpecPtr>());
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const MultiplicationFrameSpec*>(&other))
          return false;

        const MultiplicationFrameSpec* other_p = dynamic_cast<const MultiplicationFrameSpec*>(&other);

        if(get_inputs().size() != other_p->get_inputs().size())
          return false;

        if(!inputs_valid() || !other_p->inputs_valid())
          return false;

        for(size_t i=0; i<get_inputs().size(); ++i)
          if(get_inputs()[i] != other_p->get_inputs()[i])
            return false;
        
        return true;
      }

      bool inputs_valid() const
      {
        for(size_t i=0; i<get_inputs().size(); ++i)
          if(!get_inputs()[i].get())
            return false;

        return true;
      }

      virtual std::string to_string() const
      {
        std::string result = "type: FRAME-MULTIPLICATION\ninputs:[";
        for(size_t i=0; i<get_inputs().size(); ++i)
          result += "\n" + get_inputs()[i]->to_string();
        result += "]";
        return result;
      }

    private:
      std::vector<giskard::FrameSpecPtr> inputs_;

      virtual KDL::Expression<KDL::Frame>::Ptr generate_expression(const giskard::Scope& scope)
      {
        KDL::Expression<KDL::Frame>::Ptr result = KDL::Constant(KDL::Frame::Identity());

        using KDL::operator*;
        for(size_t i=0; i<get_inputs().size(); ++i)
          result = result * get_inputs()[i]->get_expression(scope);

        return result;
      }
  };

  typedef typename boost::shared_ptr<MultiplicationFrameSpec> MultiplicationFrameSpecPtr;

  class FrameReferenceSpec : public FrameSpec
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

      virtual void clear()
      {
        Spec::clear();
        set_reference_name("");
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const FrameReferenceSpec*>(&other))
          return false;

        return (dynamic_cast<const FrameReferenceSpec*>(&other)->get_reference_name().compare(this->get_reference_name()) == 0);
      }

      virtual std::string to_string() const
      {
        return "todo: implement me";
      }

    private:
      std::string reference_name_;

      virtual KDL::Expression<KDL::Frame>::Ptr generate_expression(const giskard::Scope& scope)
      {
        return scope.find_frame_expression(get_reference_name());
      }
  };

  typedef typename boost::shared_ptr<FrameReferenceSpec> FrameReferenceSpecPtr;

  ///
  /// Specification of a Scope
  ///

  class ScopeEntry 
  {
    public:
      std::string name;
      giskard::SpecPtr spec;
  };

  typedef std::vector<ScopeEntry> ScopeSpec;

}

#endif // GISKARD_SPECIFICATIONS_HPP
