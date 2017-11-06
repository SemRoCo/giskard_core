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

#ifndef GISKARD_CORE_SPECIFICATIONS_HPP
#define GISKARD_CORE_SPECIFICATIONS_HPP

#include <string>
#include <iostream>
#include <map>
#include <boost/lexical_cast.hpp>
#include <giskard_core/expressiontree.hpp>
#include <giskard_core/scope.hpp>

namespace giskard_core
{
  ///
  /// base of all specifications of expressions
  ///

  class Spec
  { 
    public:
      virtual bool equals(const Spec& other) const = 0;
  };

  inline bool operator==(const Spec& lhs, const Spec& rhs)
  {
    return lhs.equals(rhs);
  }

  inline bool operator!=(const Spec& lhs, const Spec& rhs)
  {
    return !operator==(lhs,rhs);
  }

  typedef typename boost::shared_ptr<Spec> SpecPtr;

  ///
  /// next level of expression specifications
  ///

  class AliasReferenceSpec: public Spec
  {
    public:
      AliasReferenceSpec() : reference_name_( "" ) {}
      AliasReferenceSpec(const std::string& reference_name) : 
        reference_name_( reference_name ) {}
      AliasReferenceSpec(const AliasReferenceSpec& other) : 
        reference_name_ ( other.get_reference_name() ) {}

      const std::string& get_reference_name() const
      {
        return reference_name_;
      }

      void set_reference_name(const std::string& reference_name)
      {
        reference_name_ = reference_name;
      }

      bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const AliasReferenceSpec*>(&other))
          return false;
  
        return (dynamic_cast<const AliasReferenceSpec*>(&other)->get_reference_name().compare(this->get_reference_name()) == 0);
      }
  
      KDL::ExpressionBase::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return scope.find_expression(get_reference_name());
      }
    private:
      std::string reference_name_;
  };

  typedef typename boost::shared_ptr<AliasReferenceSpec> AliasReferenceSpecPtr;

  inline AliasReferenceSpecPtr alias_reference_spec(const std::string& reference_name = "")
  {
    return AliasReferenceSpecPtr(new AliasReferenceSpec(reference_name));
  }

  class DoubleSpec : public Spec
  {
    public:
      virtual bool equals(const Spec& other) const = 0;

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope) = 0;
  };

  typedef typename boost::shared_ptr<DoubleSpec> DoubleSpecPtr;

  class VectorSpec : public Spec
  {
    public:
      virtual bool equals(const Spec& other) const = 0;

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard_core::Scope& scope) = 0;
  };

  typedef typename boost::shared_ptr<VectorSpec> VectorSpecPtr;

  class RotationSpec : public Spec
  {
    public:
      virtual bool equals(const Spec& other) const = 0;

      virtual KDL::Expression<KDL::Rotation>::Ptr get_expression(const giskard_core::Scope& scope) = 0;
  };

  typedef typename boost::shared_ptr<RotationSpec> RotationSpecPtr;

  class FrameSpec : public Spec
  {
    public:
      virtual bool equals(const Spec& other) const = 0;

      virtual KDL::Expression<KDL::Frame>::Ptr get_expression(const giskard_core::Scope& scope) = 0;
  };

  typedef typename boost::shared_ptr<FrameSpec> FrameSpecPtr;

  ///
  /// forward declarations of useful convenience functions to build default specifications
  ///

  inline DoubleSpecPtr double_const_spec(double value = 0.0);

  inline VectorSpecPtr vector_constructor_spec(const DoubleSpecPtr& x = double_const_spec(),
      const DoubleSpecPtr& y = double_const_spec(), const DoubleSpecPtr& z = double_const_spec());

  inline RotationSpecPtr quaternion_spec(double x=0, double y=0, double z=0, double w=1);

  inline FrameSpecPtr frame_constructor_spec(const VectorSpecPtr& translation =
      vector_constructor_spec(), const RotationSpecPtr& rotation = quaternion_spec());

  ///
  /// specifications of double expressions
  ///

  class DoubleConstSpec : public DoubleSpec
  {
    public:
      DoubleConstSpec() : value_( 0.0 ) {}
      DoubleConstSpec(double value) : value_( value ) {}
      DoubleConstSpec(const DoubleConstSpec& other) : value_ ( other.get_value() ) {}
      ~DoubleConstSpec() {}

      double get_value() const
      {
        return value_;
      }

      void set_value(double value)
      {
        value_ = value;
      } 

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleConstSpec*>(&other))
          return false;

        return KDL::epsilon >
            std::abs(dynamic_cast<const DoubleConstSpec*>(&other)->get_value() - this->get_value());
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::Constant(get_value());
      }

    private:
      double value_;
  };

  typedef typename boost::shared_ptr<DoubleConstSpec> DoubleConstSpecPtr;

  inline DoubleSpecPtr double_const_spec(double value)
  {
    return DoubleConstSpecPtr(new DoubleConstSpec(value));
  }

  class DoubleInputSpec : public DoubleSpec
  {
    public:
      DoubleInputSpec() :
        input_num_( 0 ) {}
      DoubleInputSpec(const DoubleInputSpec& other) :
        input_num_( other.get_input_num() ) {}
      DoubleInputSpec(size_t input_num) :
        input_num_( input_num ) {}
      ~DoubleInputSpec() {}

      size_t get_input_num() const
      {
        return input_num_;
      }

      void set_input_num(size_t input_num)
      {
        input_num_ = input_num;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleInputSpec*>(&other))
          return false;

        return dynamic_cast<const DoubleInputSpec*>(&other)->get_input_num() == this->get_input_num();
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::input(get_input_num());
      }

    private:
      size_t input_num_;
  };

  typedef typename boost::shared_ptr<DoubleInputSpec> DoubleInputSpecPtr;

  inline DoubleInputSpecPtr input(size_t input_num = 0)
  {
    return DoubleInputSpecPtr(new DoubleInputSpec(input_num));
  }

  class DoubleReferenceSpec : public DoubleSpec
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

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleReferenceSpec*>(&other))
          return false;

        return (dynamic_cast<const DoubleReferenceSpec*>(&other)->get_reference_name().compare(this->get_reference_name()) == 0);
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return scope.find_double_expression(get_reference_name());
      }

    private:
      std::string reference_name_;
  };

  typedef typename boost::shared_ptr<DoubleReferenceSpec> DoubleReferenceSpecPtr;

  class DoubleAdditionSpec: public DoubleSpec
  {
    public:
      DoubleAdditionSpec() :
        inputs_( {double_const_spec()} ) {}
      DoubleAdditionSpec(const DoubleAdditionSpec& other) :
        inputs_( other.get_inputs() ) {}
      DoubleAdditionSpec(const std::vector<DoubleSpecPtr>& inputs) :
        inputs_( inputs ) {}
      ~DoubleAdditionSpec() {}

      const std::vector<DoubleSpecPtr>& get_inputs() const
      {
        return inputs_;
      }

      void set_inputs(const std::vector<DoubleSpecPtr>& inputs)
      {
        inputs_ = inputs;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleAdditionSpec*>(&other))
          return false;

        const DoubleAdditionSpec* other_p = dynamic_cast<const DoubleAdditionSpec*>(&other);

        if(get_inputs().size() != other_p->get_inputs().size())
          return false;

        if(!inputs_valid() || !other_p->inputs_valid())
          return false;

        for(size_t i=0; i<get_inputs().size(); ++i)
          if(!get_inputs()[i]->equals(*(other_p->get_inputs()[i])))
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

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        KDL::Expression<double>::Ptr result = KDL::Constant(0.0);
        using KDL::operator+;
        for(size_t i=0; i<get_inputs().size(); ++i)
          result = result + get_inputs()[i]->get_expression(scope);
    
        return result;
      }

    private:
      std::vector<DoubleSpecPtr> inputs_;
   };

  typedef typename boost::shared_ptr<DoubleAdditionSpec> DoubleAdditionSpecPtr;

  inline DoubleAdditionSpecPtr double_add_spec(const std::vector<DoubleSpecPtr>& inputs = {double_const_spec()})
  {
    return DoubleAdditionSpecPtr(new DoubleAdditionSpec(inputs));
  }

  class DoubleSubtractionSpec: public DoubleSpec
  {
    public:
      DoubleSubtractionSpec() :
        inputs_( {double_const_spec()} ) {}
      DoubleSubtractionSpec(const DoubleSubtractionSpec& other) :
        inputs_( other.get_inputs() ) {}
      DoubleSubtractionSpec(const std::vector<DoubleSpecPtr>& inputs) :
        inputs_( inputs ) {}
      ~DoubleSubtractionSpec() {}

      const std::vector<DoubleSpecPtr>& get_inputs() const
      {
        return inputs_;
      }

      void set_inputs(const std::vector<DoubleSpecPtr>& inputs)
      {
        inputs_ = inputs;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleSubtractionSpec*>(&other))
          return false;

        const DoubleSubtractionSpec* other_p = dynamic_cast<const DoubleSubtractionSpec*>(&other);

        if(get_inputs().size() != other_p->get_inputs().size())
          return false;

        if(!inputs_valid() || !other_p->inputs_valid())
          return false;

        for(size_t i=0; i<get_inputs().size(); ++i)
          if(!get_inputs()[i]->equals(*(other_p->get_inputs()[i])))
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

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        if(get_inputs().size() == 0)
          throw std::length_error("Found DoubleSubtractionSpec with zero inputs.");

        using KDL::operator+;
        using KDL::operator-;

        KDL::Expression<double>::Ptr minuend = get_inputs()[0]->get_expression(scope);

        if(get_inputs().size() == 1)
          return -minuend;
        else
        {
          KDL::Expression<double>::Ptr subtrahend = get_inputs()[1]->get_expression(scope);

          for(size_t i=2; i<get_inputs().size(); ++i)
            subtrahend = subtrahend + get_inputs()[i]->get_expression(scope);

          return minuend - subtrahend;
        }
      }

    private:
      std::vector<giskard_core::DoubleSpecPtr> inputs_;
  };

  typedef typename boost::shared_ptr<DoubleSubtractionSpec> DoubleSubtractionSpecPtr;

  inline DoubleSubtractionSpecPtr double_sub_spec(const std::vector<DoubleSpecPtr>& inputs = {double_const_spec()})
  {
    return DoubleSubtractionSpecPtr(new DoubleSubtractionSpec(inputs));
  }

  class DoubleNormOfSpec : public DoubleSpec
  {
    public:
      DoubleNormOfSpec() :
        vector_( vector_constructor_spec() ) {}
      DoubleNormOfSpec(const DoubleNormOfSpec& other) :
        vector_( other.get_vector() ) {}
      DoubleNormOfSpec(const VectorSpecPtr& new_vector) :
        vector_( new_vector ) {}
      ~DoubleNormOfSpec() {}

      const giskard_core::VectorSpecPtr& get_vector() const
      {
        return vector_;
      }

      void set_vector(const giskard_core::VectorSpecPtr& vector)
      {
        vector_ = vector;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleNormOfSpec*>(&other))
          return false;

        return dynamic_cast<const DoubleNormOfSpec*>(&other)->get_vector()->equals(*(this->get_vector()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::norm(get_vector()->get_expression(scope));
      }

    private:
      giskard_core::VectorSpecPtr vector_;
  };

  typedef typename boost::shared_ptr<DoubleNormOfSpec> DoubleNormOfSpecPtr;

  inline DoubleNormOfSpecPtr vector_norm(const VectorSpecPtr& new_vector = vector_constructor_spec())
  {
    return DoubleNormOfSpecPtr(new DoubleNormOfSpec(new_vector));
  }

  class DoubleMultiplicationSpec: public DoubleSpec
  {
    public:
      DoubleMultiplicationSpec() :
        inputs_( {double_const_spec()} ) {}
      DoubleMultiplicationSpec(const DoubleMultiplicationSpec& other) :
        inputs_( other.get_inputs() ) {}
      DoubleMultiplicationSpec(const std::vector<DoubleSpecPtr>& inputs) :
        inputs_( inputs ) {}
      ~DoubleMultiplicationSpec() {}

      const std::vector<DoubleSpecPtr>& get_inputs() const
      {
        return inputs_;
      }

      void set_inputs(const std::vector<DoubleSpecPtr>& inputs)
      {
        inputs_ = inputs;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleMultiplicationSpec*>(&other))
          return false;

        const DoubleMultiplicationSpec* other_p = dynamic_cast<const DoubleMultiplicationSpec*>(&other);

        if(get_inputs().size() != other_p->get_inputs().size())
          return false;

        if(!inputs_valid() || !other_p->inputs_valid())
          return false;

        for(size_t i=0; i<get_inputs().size(); ++i)
          if(!get_inputs()[i]->equals(*(other_p->get_inputs()[i])))
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

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        KDL::Expression<double>::Ptr result = KDL::Constant(1.0);

        using KDL::operator*;
        for(size_t i=0; i<get_inputs().size(); ++i)
          result = result * get_inputs()[i]->get_expression(scope);

        return result; 
      }

    private:
      std::vector<giskard_core::DoubleSpecPtr> inputs_;
  };

  typedef typename boost::shared_ptr<DoubleMultiplicationSpec> DoubleMultiplicationSpecPtr;

  inline DoubleMultiplicationSpecPtr double_mul_spec(const std::vector<DoubleSpecPtr>& inputs = {double_const_spec()})
  {
    return DoubleMultiplicationSpecPtr(new DoubleMultiplicationSpec(inputs));
  }

  class DoubleDivisionSpec: public DoubleSpec
  {
    public:
      DoubleDivisionSpec() :
        inputs_( {double_const_spec()} ) {}
      DoubleDivisionSpec(const DoubleDivisionSpec& other) :
        inputs_( other.get_inputs() ) {}
      DoubleDivisionSpec(const std::vector<DoubleSpecPtr>& inputs) :
        inputs_( inputs ) {}
      ~DoubleDivisionSpec() {}

      const std::vector<DoubleSpecPtr>& get_inputs() const
      {
        return inputs_;
      }

      void set_inputs(const std::vector<DoubleSpecPtr>& inputs)
      {
        inputs_ = inputs;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleDivisionSpec*>(&other))
          return false;

        const DoubleDivisionSpec* other_p = dynamic_cast<const DoubleDivisionSpec*>(&other);

        if(get_inputs().size() != other_p->get_inputs().size())
          return false;

        if(!inputs_valid() || !other_p->inputs_valid())
          return false;

        for(size_t i=0; i<get_inputs().size(); ++i)
          if(!get_inputs()[i]->equals(*(other_p->get_inputs()[i])))
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

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        if(get_inputs().size() == 0)
          throw std::length_error("Found DoubleDivisionSpec with zero inputs.");

        using KDL::operator*;
        using KDL::operator/;

        KDL::Expression<double>::Ptr dividend = get_inputs()[0]->get_expression(scope);

        if(get_inputs().size() == 1)
          return  KDL::Constant(1.0)/dividend;
        else
        {
          KDL::Expression<double>::Ptr divisor = get_inputs()[1]->get_expression(scope);

          for(size_t i=2; i<get_inputs().size(); ++i)
            divisor = divisor * get_inputs()[i]->get_expression(scope);

          return dividend / divisor;
        }
      }

    private:
      std::vector<giskard_core::DoubleSpecPtr> inputs_;
  };

  typedef typename boost::shared_ptr<DoubleDivisionSpec> DoubleDivisionSpecPtr;

  inline DoubleDivisionSpecPtr double_div(const std::vector<DoubleSpecPtr>& inputs = {double_const_spec()})
  {
    return DoubleDivisionSpecPtr(new DoubleDivisionSpec(inputs));
  }

  class DoubleXCoordOfSpec : public DoubleSpec
  {
    public:
      DoubleXCoordOfSpec() :
        vector_( vector_constructor_spec() ) {}
      DoubleXCoordOfSpec(const DoubleXCoordOfSpec& other) :
        vector_( other.get_vector() ) {}
      DoubleXCoordOfSpec(const VectorSpecPtr& new_vector) :
        vector_( new_vector ) {}
      ~DoubleXCoordOfSpec() {}

      const giskard_core::VectorSpecPtr& get_vector() const
      {
        return vector_;
      }

      void set_vector(const giskard_core::VectorSpecPtr& vector)
      {
        vector_ = vector;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleXCoordOfSpec*>(&other))
          return false;

        return dynamic_cast<const DoubleXCoordOfSpec*>(&other)->get_vector()->equals(*(this->get_vector()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::coord_x(get_vector()->get_expression(scope));
      }

    private:
      giskard_core::VectorSpecPtr vector_;
  };

  typedef typename boost::shared_ptr<DoubleXCoordOfSpec> DoubleXCoordOfSpecPtr;

  inline DoubleXCoordOfSpecPtr x_coord(const VectorSpecPtr vector_ = vector_constructor_spec())
  {
    return DoubleXCoordOfSpecPtr(new DoubleXCoordOfSpec(vector_));
  }

  class DoubleYCoordOfSpec : public DoubleSpec
  {
    public:
      DoubleYCoordOfSpec() :
        vector_( vector_constructor_spec() ) {}
      DoubleYCoordOfSpec(const DoubleYCoordOfSpec& other) :
        vector_( other.get_vector() ) {}
      DoubleYCoordOfSpec(const VectorSpecPtr& new_vector) :
        vector_( new_vector ) {}
      ~DoubleYCoordOfSpec() {}

      const giskard_core::VectorSpecPtr& get_vector() const
      {
        return vector_;
      }

      void set_vector(const giskard_core::VectorSpecPtr& vector)
      {
        vector_ = vector;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleYCoordOfSpec*>(&other))
          return false;

        return dynamic_cast<const DoubleYCoordOfSpec*>(&other)->get_vector()->equals(*(this->get_vector()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::coord_y(get_vector()->get_expression(scope));
      }

    private:
      giskard_core::VectorSpecPtr vector_;
  };

  typedef typename boost::shared_ptr<DoubleYCoordOfSpec> DoubleYCoordOfSpecPtr;

  inline DoubleYCoordOfSpecPtr y_coord(const VectorSpecPtr vector_ = vector_constructor_spec())
  {
    return DoubleYCoordOfSpecPtr(new DoubleYCoordOfSpec(vector_));
  }

  class DoubleZCoordOfSpec : public DoubleSpec
  {
    public:
      DoubleZCoordOfSpec() :
        vector_( vector_constructor_spec() ) {}
      DoubleZCoordOfSpec(const DoubleZCoordOfSpec& other) :
        vector_( other.get_vector() ) {}
      DoubleZCoordOfSpec(const VectorSpecPtr& new_vector) :
        vector_( new_vector ) {}
      ~DoubleZCoordOfSpec() {}

      const giskard_core::VectorSpecPtr& get_vector() const
      {
        return vector_;
      }

      void set_vector(const giskard_core::VectorSpecPtr& vector)
      {
        vector_ = vector;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleZCoordOfSpec*>(&other))
          return false;

        return dynamic_cast<const DoubleZCoordOfSpec*>(&other)->get_vector()->equals(*(this->get_vector()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::coord_z(get_vector()->get_expression(scope));
      }

    private:
      giskard_core::VectorSpecPtr vector_;
  };

  typedef typename boost::shared_ptr<DoubleZCoordOfSpec> DoubleZCoordOfSpecPtr;

  inline DoubleZCoordOfSpecPtr z_coord(const VectorSpecPtr vector_ = vector_constructor_spec())
  {
    return DoubleZCoordOfSpecPtr(new DoubleZCoordOfSpec(vector_));
  }

  class VectorDotSpec: public DoubleSpec
  {
    public:
      const VectorSpecPtr& get_lhs() const
      {
        return lhs_;
      }

      const VectorSpecPtr& get_rhs() const
      {
        return rhs_;
      }

      void set_lhs(const VectorSpecPtr& lhs)
      {
        lhs_ = lhs;
      }

      void set_rhs(const VectorSpecPtr& rhs)
      {
        rhs_ = rhs;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const VectorDotSpec*>(&other))
          return false;

        const VectorDotSpec* other_p = dynamic_cast<const VectorDotSpec*>(&other);

        return get_lhs().get() && get_rhs().get() &&
            other_p->get_lhs().get() && other_p->get_rhs().get() &&
            get_lhs()->equals(*(other_p->get_lhs())) &&
            get_rhs()->equals(*(other_p->get_rhs()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::dot(get_lhs()->get_expression(scope), get_rhs()->get_expression(scope));
      }

    private:
      giskard_core::VectorSpecPtr lhs_, rhs_;
  };

  typedef typename boost::shared_ptr<VectorDotSpec> VectorDotSpecPtr;

  class MinSpec: public DoubleSpec
  {
    public:
      const DoubleSpecPtr& get_lhs() const
      {
        return lhs_;
      }

      const DoubleSpecPtr& get_rhs() const
      {
        return rhs_;
      }

      void set_lhs(const DoubleSpecPtr& lhs)
      {
        lhs_ = lhs;
      }

      void set_rhs(const DoubleSpecPtr& rhs)
      {
        rhs_ = rhs;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const MinSpec*>(&other))
          return false;

        const MinSpec* other_p = dynamic_cast<const MinSpec*>(&other);

        return get_lhs().get() && get_rhs().get() &&
            other_p->get_lhs().get() && other_p->get_rhs().get() &&
            get_lhs()->equals(*(other_p->get_lhs())) &&
            get_rhs()->equals(*(other_p->get_rhs()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::minimum(get_lhs()->get_expression(scope), get_rhs()->get_expression(scope));
      }

    private:
      giskard_core::DoubleSpecPtr lhs_, rhs_;
  };

  typedef typename boost::shared_ptr<MinSpec> MinSpecPtr;

  class AbsSpec: public DoubleSpec
  {
    public:
      const DoubleSpecPtr& get_value() const
      {
        return value_;
      }

      void set_value(const DoubleSpecPtr& value)
      {
        value_ = value;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const AbsSpec*>(&other))
          return false;

        const AbsSpec* other_p = dynamic_cast<const AbsSpec*>(&other);

        return get_value().get() && other_p->get_value().get() &&
            get_value()->equals(*(other_p->get_value()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::abs(get_value()->get_expression(scope));
      }

    private:
      giskard_core::DoubleSpecPtr value_;
  };

  typedef typename boost::shared_ptr<AbsSpec> AbsSpecPtr;

  class DoubleIfSpec: public DoubleSpec
  {
    public:
      DoubleIfSpec() :
        condition_( double_const_spec() ), if_( double_const_spec() ), else_( double_const_spec() ) {}
      DoubleIfSpec(const DoubleIfSpec& other) :
        condition_( other.get_condition()), if_( other.get_if() ), else_( other.get_else() ) {}
      DoubleIfSpec(const DoubleSpecPtr& condition, const DoubleSpecPtr& new_if, const DoubleSpecPtr& new_else ) :
        condition_( condition ), if_( new_if ), else_( new_else ) {}
      ~DoubleIfSpec() {}

      const DoubleSpecPtr& get_condition() const
      {
        return condition_;
      }

      const DoubleSpecPtr& get_if() const
      {
        return if_;
      }

      const DoubleSpecPtr& get_else() const
      {
        return else_;
      }

      void set_condition(const DoubleSpecPtr& condition)
      {
        condition_ = condition;
      }

      void set_if(const DoubleSpecPtr& new_if)
      {
        if_ = new_if;
      }

      void set_else(const DoubleSpecPtr& new_else)
      {
        else_ = new_else;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleIfSpec*>(&other))
          return false;

        const DoubleIfSpec* other_p = dynamic_cast<const DoubleIfSpec*>(&other);

        return get_condition().get() && get_if().get() && get_else().get() &&
            other_p->get_condition().get() && other_p->get_if().get() && other_p->get_else().get() &&
            get_condition()->equals(*(other_p->get_condition())) &&
            get_if()->equals(*(other_p->get_if())) &&
            get_else()->equals(*(other_p->get_else()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::conditional<double>(get_condition()->get_expression(scope), get_if()->get_expression(scope), get_else()->get_expression(scope));
      }

    private:
      giskard_core::DoubleSpecPtr condition_, if_, else_;
  };

  typedef typename boost::shared_ptr<DoubleIfSpec> DoubleIfSpecPtr;

  inline DoubleSpecPtr double_if(const DoubleSpecPtr& condition_ =
      double_const_spec(), const DoubleSpecPtr& if_ = double_const_spec(), const DoubleSpecPtr& else_ = double_const_spec())
  {
    return DoubleIfSpecPtr(new DoubleIfSpec(condition_, if_, else_));
  }

  class FmodSpec: public DoubleSpec
  {
    public:
      FmodSpec() :
        nominator_( double_const_spec() ), denominator_( double_const_spec() ) {}
      FmodSpec(const FmodSpec& other) :
        nominator_( other.get_nominator()), denominator_( other.get_denominator() ) {}
      FmodSpec(const DoubleSpecPtr& nominator, const DoubleSpecPtr& denominator) :
        nominator_( nominator ), denominator_( denominator ) {}
      ~FmodSpec() {}

      const DoubleSpecPtr& get_nominator() const
      {
        return nominator_;
      }

      void set_nominator(const DoubleSpecPtr& nominator)
      {
        nominator_ = nominator;
      }

      const DoubleSpecPtr& get_denominator() const
      {
        return denominator_;
      }

      void set_denominator(const DoubleSpecPtr& denominator)
      {
        denominator_ = denominator;
      }

      bool members_valid() const
      {
        return get_nominator().get() && get_denominator().get();
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const FmodSpec*>(&other))
          return false;

        const FmodSpec* other_p = dynamic_cast<const FmodSpec*>(&other);

        if(!members_valid() || !other_p->members_valid())
          return false;

        return (get_nominator()->equals(*(other_p->get_nominator()))) && 
               (get_denominator()->equals(*( other_p->get_denominator())));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        // note: This expression only expects a TRUE expressions for the nominator.
        //       While this makes sense, it does break code symmetry.
        KDL::Expression<double>::Ptr nominator = get_nominator()->get_expression(scope);
        double denominator = get_denominator()->get_expression(scope)->value();

        return KDL::fmod(nominator, denominator);
      }

    private:
      DoubleSpecPtr nominator_, denominator_;
  };

  typedef typename boost::shared_ptr<FmodSpec> FmodSpecPtr;

  inline DoubleSpecPtr fmod(const DoubleSpecPtr& nominator =
      double_const_spec(), const DoubleSpecPtr& demoninator = double_const_spec())
  {
    return FmodSpecPtr(new FmodSpec(nominator, demoninator));
  }

  class SinSpec: public DoubleSpec
  {
    public:
      const DoubleSpecPtr& get_value() const
      {
        return value_;
      }

      void set_value(const DoubleSpecPtr& value)
      {
        value_ = value;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const SinSpec*>(&other))
          return false;

        const SinSpec* other_p = dynamic_cast<const SinSpec*>(&other);

        return get_value().get() && other_p->get_value().get() &&
            get_value()->equals(*(other_p->get_value()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::sin(get_value()->get_expression(scope));
      }

    private:
      giskard_core::DoubleSpecPtr value_;
  };

  typedef typename boost::shared_ptr<SinSpec> SinSpecPtr;

  class CosSpec: public DoubleSpec
  {
    public:
      const DoubleSpecPtr& get_value() const
      {
        return value_;
      }

      void set_value(const DoubleSpecPtr& value)
      {
        value_ = value;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const CosSpec*>(&other))
          return false;

        const CosSpec* other_p = dynamic_cast<const CosSpec*>(&other);

        return get_value().get() && other_p->get_value().get() &&
            get_value()->equals(*(other_p->get_value()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::cos(get_value()->get_expression(scope));
      }

    private:
      giskard_core::DoubleSpecPtr value_;
  };

  typedef typename boost::shared_ptr<CosSpec> CosSpecPtr;

  class TanSpec: public DoubleSpec
  {
    public:
      const DoubleSpecPtr& get_value() const
      {
        return value_;
      }

      void set_value(const DoubleSpecPtr& value)
      {
        value_ = value;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const TanSpec*>(&other))
          return false;

        const TanSpec* other_p = dynamic_cast<const TanSpec*>(&other);

        return get_value().get() && other_p->get_value().get() &&
            get_value()->equals(*(other_p->get_value()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::tan(get_value()->get_expression(scope));
      }

    private:
      giskard_core::DoubleSpecPtr value_;
  };

  typedef typename boost::shared_ptr<TanSpec> TanSpecPtr;


  class ASinSpec: public DoubleSpec
  {
    public:
      const DoubleSpecPtr& get_value() const
      {
        return value_;
      }

      void set_value(const DoubleSpecPtr& value)
      {
        value_ = value;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const ASinSpec*>(&other))
          return false;

        const ASinSpec* other_p = dynamic_cast<const ASinSpec*>(&other);

        return get_value().get() && other_p->get_value().get() &&
            get_value()->equals(*(other_p->get_value()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::asin(get_value()->get_expression(scope));
      }

    private:
      giskard_core::DoubleSpecPtr value_;
  };

  typedef typename boost::shared_ptr<ASinSpec> ASinSpecPtr;

  class ACosSpec: public DoubleSpec
  {
    public:
      const DoubleSpecPtr& get_value() const
      {
        return value_;
      }

      void set_value(const DoubleSpecPtr& value)
      {
        value_ = value;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const ACosSpec*>(&other))
          return false;

        const ACosSpec* other_p = dynamic_cast<const ACosSpec*>(&other);

        return get_value().get() && other_p->get_value().get() &&
            get_value()->equals(*(other_p->get_value()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::acos(get_value()->get_expression(scope));
      }

    private:
      giskard_core::DoubleSpecPtr value_;
  };

  typedef typename boost::shared_ptr<ACosSpec> ACosSpecPtr;

  class ATanSpec: public DoubleSpec
  {
    public:
      const DoubleSpecPtr& get_value() const
      {
        return value_;
      }

      void set_value(const DoubleSpecPtr& value)
      {
        value_ = value;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const ATanSpec*>(&other))
          return false;

        const ATanSpec* other_p = dynamic_cast<const ATanSpec*>(&other);

        return get_value().get() && other_p->get_value().get() &&
            get_value()->equals(*(other_p->get_value()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::atan(get_value()->get_expression(scope));
      }

    private:
      giskard_core::DoubleSpecPtr value_;
  };

  typedef typename boost::shared_ptr<ATanSpec> ATanSpecPtr;

  class SqrtSpec: public DoubleSpec
  {
    public:
      const DoubleSpecPtr& get_value() const
      {
        return value_;
      }

      void set_value(const DoubleSpecPtr& value)
      {
        value_ = value;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const SqrtSpec*>(&other))
          return false;

        const SqrtSpec* other_p = dynamic_cast<const SqrtSpec*>(&other);

        return get_value().get() && other_p->get_value().get() &&
            get_value()->equals(*(other_p->get_value()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::sqrt(get_value()->get_expression(scope));
      }

    private:
      giskard_core::DoubleSpecPtr value_;
  };

  typedef typename boost::shared_ptr<SqrtSpec> SqrtSpecPtr;

  class MaxSpec: public DoubleSpec
  {
    public:
      const DoubleSpecPtr& get_lhs() const
      {
        return lhs_;
      }

      const DoubleSpecPtr& get_rhs() const
      {
        return rhs_;
      }

      void set_lhs(const DoubleSpecPtr& lhs)
      {
        lhs_ = lhs;
      }

      void set_rhs(const DoubleSpecPtr& rhs)
      {
        rhs_ = rhs;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const MaxSpec*>(&other))
          return false;

        const MaxSpec* other_p = dynamic_cast<const MaxSpec*>(&other);

        return get_lhs().get() && get_rhs().get() &&
            other_p->get_lhs().get() && other_p->get_rhs().get() &&
            get_lhs()->equals(*(other_p->get_lhs())) &&
            get_rhs()->equals(*(other_p->get_rhs()));
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::maximum(get_lhs()->get_expression(scope), get_rhs()->get_expression(scope));
      }

    private:
      giskard_core::DoubleSpecPtr lhs_, rhs_;
  };

  typedef typename boost::shared_ptr<MaxSpec> MaxSpecPtr;

  ///
  /// specifications of vector expressions
  ///

  class VectorCachedSpec: public VectorSpec
  {
    public:
      const giskard_core::VectorSpecPtr& get_vector() const
      {
        return vector_;
      }

      void set_vector(const giskard_core::VectorSpecPtr& vector)
      {
        vector_ = vector;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const VectorCachedSpec*>(&other))
          return false;

        const VectorCachedSpec* other_p = dynamic_cast<const VectorCachedSpec*>(&other);

        return get_vector().get() && other_p->get_vector().get() &&
            get_vector()->equals(*(other_p->get_vector()));
      }

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::cached<KDL::Vector>(get_vector()->get_expression(scope));
      }

    private:
      VectorSpecPtr vector_;
  };

  typedef typename boost::shared_ptr<VectorCachedSpec> VectorCachedSpecPtr;

  class VectorConstructorSpec: public VectorSpec
  {
    public:
      VectorConstructorSpec() :
        x_( double_const_spec() ), y_( double_const_spec() ), z_( double_const_spec() ) {}
      VectorConstructorSpec(const DoubleSpecPtr& x, const DoubleSpecPtr& y, const DoubleSpecPtr& z) :
        x_( x ), y_( y ), z_( z ) {}
      VectorConstructorSpec(const VectorConstructorSpec& other) :
        x_( other.get_x() ), y_( other.get_x() ), z_( other.get_x() ) {}
      ~VectorConstructorSpec() {}

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

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const VectorConstructorSpec*>(&other))
          return false;

        const VectorConstructorSpec* other_p = dynamic_cast<const VectorConstructorSpec*>(&other);
        
        if(!members_valid() || !other_p->members_valid())
          return false;

        return (get_x()->equals(*(other_p->get_x()))) && (get_y()->equals(*(other_p->get_y()))) &&
            (get_z()->equals(*(other_p->get_z())));
      }

      bool members_valid() const
      {
        return get_x().get() && get_y().get() && get_z().get();
      }

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::vector(get_x()->get_expression(scope), 
            get_y()->get_expression(scope), get_z()->get_expression(scope));
      }

    private:
      DoubleSpecPtr x_, y_, z_;
  };

  typedef typename boost::shared_ptr<VectorConstructorSpec> VectorConstructorSpecPtr;

  inline VectorSpecPtr vector_constructor_spec(const DoubleSpecPtr& x, const DoubleSpecPtr& y, const DoubleSpecPtr& z)
  {
    return VectorConstructorSpecPtr(new VectorConstructorSpec(x, y, z));
  }

  class VectorAdditionSpec: public VectorSpec
  {
    public:
      const std::vector<VectorSpecPtr>& get_inputs() const
      {
        return inputs_;
      }

      void set_inputs(const std::vector<VectorSpecPtr>& inputs)
      {
        inputs_ = inputs;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const VectorAdditionSpec*>(&other))
          return false;

        const VectorAdditionSpec* other_p = dynamic_cast<const VectorAdditionSpec*>(&other);

        if(get_inputs().size() != other_p->get_inputs().size())
          return false;

        if(!inputs_valid() || !other_p->inputs_valid())
          return false;

        for(size_t i=0; i<get_inputs().size(); ++i)
          if(!get_inputs()[i]->equals(*(other_p->get_inputs()[i])))
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

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        using KDL::operator+;

        KDL::Expression<KDL::Vector>::Ptr result = KDL::vector(KDL::Constant(0.0), KDL::Constant(0.0), KDL::Constant(0.0));

        for(size_t i=0; i<get_inputs().size(); ++i)
          result = result + get_inputs()[i]->get_expression(scope);

        return result;
      }

    private:
      std::vector<giskard_core::VectorSpecPtr> inputs_;
  };

  typedef typename boost::shared_ptr<VectorAdditionSpec> VectorAdditionSpecPtr;

  class VectorSubtractionSpec: public VectorSpec
  {
    public:
      VectorSubtractionSpec() :
        inputs_( {vector_constructor_spec()} ) {}
      VectorSubtractionSpec(const VectorSubtractionSpec& other) :
        inputs_( other.get_inputs() ) {}
      VectorSubtractionSpec(const std::vector<VectorSpecPtr>& inputs) :
        inputs_( inputs ) {}
      ~VectorSubtractionSpec() {}

      const std::vector<VectorSpecPtr>& get_inputs() const
      {
        return inputs_;
      }

      void set_inputs(const std::vector<VectorSpecPtr>& inputs)
      {
        inputs_ = inputs;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const VectorSubtractionSpec*>(&other))
          return false;

        const VectorSubtractionSpec* other_p = dynamic_cast<const VectorSubtractionSpec*>(&other);

        if(get_inputs().size() != other_p->get_inputs().size())
          return false;

        if(!inputs_valid() || !other_p->inputs_valid())
          return false;

        for(size_t i=0; i<get_inputs().size(); ++i)
          if(!get_inputs()[i]->equals(*(other_p->get_inputs()[i])))
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

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        if(get_inputs().size() == 0)
          throw std::length_error("Found VectorSubtractionSpec with zero inputs.");

        using KDL::operator+;
        using KDL::operator-;

        KDL::Expression<KDL::Vector>::Ptr minuend = get_inputs()[0]->get_expression(scope);

        if(get_inputs().size() == 1)
          return -minuend;
        else
        {
          KDL::Expression<KDL::Vector>::Ptr subtrahend = get_inputs()[1]->get_expression(scope);

          for(size_t i=2; i<get_inputs().size(); ++i)
            subtrahend = subtrahend + get_inputs()[i]->get_expression(scope);

          return minuend - subtrahend;
        }
      }

    private:
      std::vector<giskard_core::VectorSpecPtr> inputs_;
  };

  typedef typename boost::shared_ptr<VectorSubtractionSpec> VectorSubtractionSpecPtr;

  inline VectorSubtractionSpecPtr vector_sub_spec(const std::vector<VectorSpecPtr>& inputs = {vector_constructor_spec()})
  {
    return VectorSubtractionSpecPtr(new VectorSubtractionSpec(inputs));
  }

  class VectorReferenceSpec : public VectorSpec
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

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const VectorReferenceSpec*>(&other))
          return false;

        return (dynamic_cast<const VectorReferenceSpec*>(&other)->get_reference_name().compare(this->get_reference_name()) == 0);
      }

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return scope.find_vector_expression(get_reference_name());
      }

    private:
      std::string reference_name_;
  };

  typedef typename boost::shared_ptr<VectorReferenceSpec> VectorReferenceSpecPtr;

  class VectorOriginOfSpec : public VectorSpec
  {
    public:
      VectorOriginOfSpec() :
        frame_( frame_constructor_spec()) {}
      VectorOriginOfSpec(const VectorOriginOfSpec& other) :
        frame_( other.get_frame() ) {}
      VectorOriginOfSpec(const FrameSpecPtr& frame) :
        frame_( frame ) {}
      ~VectorOriginOfSpec() {}

      const giskard_core::FrameSpecPtr& get_frame() const
      {
        return frame_;
      }

      void set_frame(const giskard_core::FrameSpecPtr& frame)
      {
        frame_ = frame;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const VectorOriginOfSpec*>(&other))
          return false;

        return dynamic_cast<const VectorOriginOfSpec*>(&other)->get_frame()->equals(*(this->get_frame()));
      }

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::origin(get_frame()->get_expression(scope));
      }

    private:
      giskard_core::FrameSpecPtr frame_;
  };

  typedef typename boost::shared_ptr<VectorOriginOfSpec> VectorOriginOfSpecPtr;

  inline VectorOriginOfSpecPtr origin(const FrameSpecPtr& frame)
  {
    return VectorOriginOfSpecPtr(new VectorOriginOfSpec(frame));
  }

  class VectorFrameMultiplicationSpec: public VectorSpec
  {
    public:
      const VectorSpecPtr& get_vector() const
      {
        return vector_;
      }

      const FrameSpecPtr& get_frame() const
      {
        return frame_;
      }

      void set_vector(const VectorSpecPtr& vector)
      {
        vector_ = vector;
      }

      void set_frame(const FrameSpecPtr& frame)
      {
        frame_ = frame;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const VectorFrameMultiplicationSpec*>(&other))
          return false;

        const VectorFrameMultiplicationSpec* other_p = dynamic_cast<const VectorFrameMultiplicationSpec*>(&other);

        return get_frame().get() && get_vector().get() && 
            get_frame()->equals(*(other_p->get_frame())) &&
            get_vector()->equals(*(other_p->get_vector()));
      }

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        using KDL::operator*;

        return get_frame()->get_expression(scope) * get_vector()->get_expression(scope);
      }

    private:
      VectorSpecPtr vector_;
      FrameSpecPtr frame_;
  };

  typedef typename boost::shared_ptr<VectorFrameMultiplicationSpec> VectorFrameMultiplicationSpecPtr;

  class VectorRotationMultiplicationSpec: public VectorSpec
  {
    public:
      VectorRotationMultiplicationSpec() :
        vector_( vector_constructor_spec() ), rotation_( quaternion_spec() ) {}
      VectorRotationMultiplicationSpec(const VectorRotationMultiplicationSpec& other) :
        vector_( other.get_vector()), rotation_( other.get_rotation() ) {}
      VectorRotationMultiplicationSpec(const RotationSpecPtr& new_rotation, const VectorSpecPtr& new_vector) :
        vector_( new_vector ), rotation_( new_rotation ) {}
      ~VectorRotationMultiplicationSpec() {}

      const VectorSpecPtr& get_vector() const
      {
        return vector_;
      }

      const RotationSpecPtr& get_rotation() const
      {
        return rotation_;
      }

      void set_vector(const VectorSpecPtr& vector)
      {
        vector_ = vector;
      }

      void set_rotation(const RotationSpecPtr& rotation)
      {
        rotation_ = rotation;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const VectorRotationMultiplicationSpec*>(&other))
          return false;

        const VectorRotationMultiplicationSpec* other_p = dynamic_cast<const VectorRotationMultiplicationSpec*>(&other);

        return get_rotation().get() && get_vector().get() && 
            get_rotation()->equals(*(other_p->get_rotation())) &&
            get_vector()->equals(*(other_p->get_vector()));
      }

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        using KDL::operator*;

        return get_rotation()->get_expression(scope) * get_vector()->get_expression(scope);
      }

    private:
      VectorSpecPtr vector_;
      RotationSpecPtr rotation_;
  };

  typedef typename boost::shared_ptr<VectorRotationMultiplicationSpec> VectorRotationMultiplicationSpecPtr;

  inline VectorRotationMultiplicationSpecPtr rotate_vector(const RotationSpecPtr& new_rotation =
      quaternion_spec(), const VectorSpecPtr& new_vector = vector_constructor_spec())
  {
    return VectorRotationMultiplicationSpecPtr(new VectorRotationMultiplicationSpec(new_rotation, new_vector));
  }

  class VectorDoubleMultiplicationSpec: public VectorSpec
  {
    public:
      VectorDoubleMultiplicationSpec() :
        vector_( vector_constructor_spec() ), double_( double_const_spec() ) {}
      VectorDoubleMultiplicationSpec(const VectorDoubleMultiplicationSpec& other) :
        vector_( other.get_vector()), double_( other.get_double() ) {}
      VectorDoubleMultiplicationSpec(const VectorSpecPtr& new_vector, const DoubleSpecPtr& new_double) :
        vector_( new_vector ), double_( new_double ) {}
      ~VectorDoubleMultiplicationSpec() {}

      const VectorSpecPtr& get_vector() const
      {
        return vector_;
      }

      const DoubleSpecPtr& get_double() const
      {
        return double_;
      }

      void set_vector(const VectorSpecPtr& vector)
      {
        vector_ = vector;
      }

      void set_double(const DoubleSpecPtr& new_double)
      {
        double_ = new_double;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const VectorDoubleMultiplicationSpec*>(&other))
          return false;

        const VectorDoubleMultiplicationSpec* other_p = 
            dynamic_cast<const VectorDoubleMultiplicationSpec*>(&other);

        return get_double().get() && get_vector().get() && 
            get_double()->equals(*(other_p->get_double())) &&
            get_vector()->equals(*(other_p->get_vector()));
      }

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        using KDL::operator*;

        return get_double()->get_expression(scope) * get_vector()->get_expression(scope);
      }

    private:
      VectorSpecPtr vector_;
      DoubleSpecPtr double_;
  };

  typedef typename boost::shared_ptr<VectorDoubleMultiplicationSpec> VectorDoubleMultiplicationSpecPtr;

  inline VectorDoubleMultiplicationSpecPtr vector_double_mul(const VectorSpecPtr& new_vector =
      vector_constructor_spec(), const DoubleSpecPtr& new_double = double_const_spec())
  {
    return VectorDoubleMultiplicationSpecPtr(new VectorDoubleMultiplicationSpec(new_vector, new_double));
  }

  class VectorRotationVectorSpec : public VectorSpec
  {
    public:
      VectorRotationVectorSpec() :
        rotation_( quaternion_spec() ) {}
      VectorRotationVectorSpec(const VectorRotationVectorSpec& other) :
        rotation_( other.get_rotation()) {}
      VectorRotationVectorSpec(const RotationSpecPtr& rotation) :
        rotation_ ( rotation ) {}
      ~VectorRotationVectorSpec() {}

      const giskard_core::RotationSpecPtr& get_rotation() const
      {
        return rotation_;
      }

      void set_rotation(const giskard_core::RotationSpecPtr& rotation)
      {
        rotation_ = rotation;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const VectorRotationVectorSpec*>(&other))
          return false;

        return dynamic_cast<const VectorRotationVectorSpec*>(&other)->get_rotation()->equals(*(this->get_rotation()));
      }

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::getRotVec(get_rotation()->get_expression(scope));
      }

    private:
      giskard_core::RotationSpecPtr rotation_;
  };

  typedef typename boost::shared_ptr<VectorRotationVectorSpec> VectorRotationVectorSpecPtr;

  inline VectorSpecPtr rot_vector(const RotationSpecPtr& rotation = quaternion_spec())
  {
    return VectorRotationVectorSpecPtr(new VectorRotationVectorSpec(rotation));
  }

  class VectorCrossSpec: public VectorSpec
  {
    public:
      const VectorSpecPtr& get_lhs() const
      {
        return lhs_;
      }

      const VectorSpecPtr& get_rhs() const
      {
        return rhs_;
      }

      void set_lhs(const VectorSpecPtr& lhs)
      {
        lhs_ = lhs;
      }

      void set_rhs(const VectorSpecPtr& rhs)
      {
        rhs_ = rhs;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const VectorCrossSpec*>(&other))
          return false;

        const VectorCrossSpec* other_p = dynamic_cast<const VectorCrossSpec*>(&other);

        return get_lhs().get() && get_rhs().get() &&
            other_p->get_lhs().get() && other_p->get_rhs().get() &&
            get_lhs()->equals(*(other_p->get_lhs())) &&
            get_rhs()->equals(*(other_p->get_rhs()));
      }

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::cross(get_lhs()->get_expression(scope), get_rhs()->get_expression(scope));
      }

    private:
      giskard_core::VectorSpecPtr lhs_, rhs_;
  };

  typedef typename boost::shared_ptr<VectorCrossSpec> VectorCrossSpecPtr;

  ///
  /// specifications for rotation expresssions
  ///

  class RotationQuaternionConstructorSpec : public RotationSpec
  {
    /*
     * TODO: turns this into a proper spec by
     * - adding a Quaternion expression to expressiongraph (does not have a derivative, yet)
     * - using it here
     * - turning all of this inputs into DoubleSpecPtr
     */

    public:
      RotationQuaternionConstructorSpec() : 
        x_( 0.0 ), y_( 0.0 ), z_( 0.0 ), w_( 1.0 ) {}
      RotationQuaternionConstructorSpec(double x, double y, double z, double w) :
        x_( x ), y_( y ), z_( z ), w_( w ) {}
      RotationQuaternionConstructorSpec(const RotationQuaternionConstructorSpec& other) :
        x_( other.get_x() ), y_( other.get_y() ), z_( other.get_z() ), w_( other.get_w() ) {}
      ~RotationQuaternionConstructorSpec() {}

      double get_x() const
      {
        return x_;
      }

      double get_y() const
      {
        return y_;
      }

      double get_z() const
      {
        return z_;
      }

      double get_w() const
      {
        return w_;
      }

      void set_x(double x)
      {
        x_ = x;
      }

      void set_y(double y)
      {
        y_ = y;
      }
      
      void set_z(double z)
      {
        z_ = z;
      }

      void set_w(double w)
      {
        w_ = w;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const RotationQuaternionConstructorSpec*>(&other))
          return false;

        return (KDL::epsilon > std::abs(dynamic_cast<const RotationQuaternionConstructorSpec*>(&other)->get_x() - this->get_x())) &&
            (KDL::epsilon > std::abs(dynamic_cast<const RotationQuaternionConstructorSpec*>(&other)->get_y() - this->get_y())) && (KDL::epsilon > std::abs(dynamic_cast<const RotationQuaternionConstructorSpec*>(&other)->get_z() - this->get_z())) && (KDL::epsilon > std::abs(dynamic_cast<const RotationQuaternionConstructorSpec*>(&other)->get_w() - this->get_w()));
      }

      virtual KDL::Expression<KDL::Rotation>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::Constant(KDL::Rotation::Quaternion(get_x(), get_y(), get_z(), get_w()));
      }

    private:
      double x_, y_, z_, w_;
  };

  typedef typename boost::shared_ptr<RotationQuaternionConstructorSpec> RotationQuaternionConstructorSpecPtr;

  inline RotationSpecPtr quaternion_spec(double x, double y, double z, double w)
  {
    return RotationQuaternionConstructorSpecPtr(new RotationQuaternionConstructorSpec(x, y, z, w));
  }

  class AxisAngleSpec: public RotationSpec
  {
    public:
      AxisAngleSpec() :
        axis_( vector_constructor_spec() ), angle_( double_const_spec() ) {}
      AxisAngleSpec(const AxisAngleSpec& other) :
        axis_( other.get_axis()), angle_( other.get_angle() ) {}
      AxisAngleSpec(const VectorSpecPtr& new_axis, const DoubleSpecPtr& new_angle) :
        axis_( new_axis ), angle_( new_angle ) {}
      ~AxisAngleSpec() {}

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

      virtual KDL::Expression<KDL::Rotation>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        KDL::Expression<KDL::Vector>::Ptr axis = get_axis()->get_expression(scope);
        KDL::Expression<double>::Ptr angle = get_angle()->get_expression(scope);
        KDL::Expression<double>::Ptr axis_norm = KDL::norm(axis);

        using KDL::operator/;
        using KDL::operator*;
        // TODO: expose normalization of vectors as operation with specification
        KDL::Expression<KDL::Vector>::Ptr near_axis =
            KDL::near_zero<KDL::Vector>(axis_norm, KDL::epsilon, KDL::Constant(KDL::Vector(1.0, 0.0, 0.0)), axis * (KDL::Constant(1.0)/axis_norm));
        KDL::Expression<double>::Ptr near_angle =
            KDL::near_zero<double>(axis_norm, KDL::epsilon, KDL::Constant(0.0), angle);
        return KDL::rotVec(near_axis, near_angle);
      }

    private:
      VectorSpecPtr axis_;
      DoubleSpecPtr angle_;
  };

  typedef typename boost::shared_ptr<AxisAngleSpec> AxisAngleSpecPtr;

  inline AxisAngleSpecPtr axis_angle_spec(const VectorSpecPtr& axis =
      vector_constructor_spec(), const DoubleSpecPtr& angle = double_const_spec())
  {
    return AxisAngleSpecPtr(new AxisAngleSpec(axis, angle));
  }

  class SlerpSpec: public RotationSpec
  {
    public:
      SlerpSpec() :
        from_( quaternion_spec() ), to_( quaternion_spec() ), param_( double_const_spec() ) {}
      SlerpSpec(const SlerpSpec& other) :
        from_( other.get_from()), to_( other.get_to() ), param_( other.get_param() ) {}
      SlerpSpec(const RotationSpecPtr& from, const RotationSpecPtr& to, const DoubleSpecPtr& param) :
        from_( from ), to_( to ), param_( param ) {}
      ~SlerpSpec() {}

      const RotationSpecPtr& get_from() const
      {
        return from_;
      }

      void set_from(const RotationSpecPtr& from)
      {
        from_ = from;
      }

      const RotationSpecPtr& get_to() const
      {
        return to_;
      }

      void set_to(const RotationSpecPtr& to)
      {
        to_ = to;
      }

      const DoubleSpecPtr& get_param() const
      {
        return param_;
      }

      void set_param(const DoubleSpecPtr& param)
      {
        param_ = param;
      }

      bool members_valid() const
      {
        return get_from().get() && get_to().get() && get_param().get();
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const SlerpSpec*>(&other))
          return false;

        const SlerpSpec* other_p = dynamic_cast<const SlerpSpec*>(&other);

        if(!members_valid() || !other_p->members_valid())
          return false;

        return (get_from()->equals(*(other_p->get_from()))) && 
               (get_to()->equals(*( other_p->get_to()))) &&
               (get_param()->equals(*( other_p->get_param())));
      }

      virtual KDL::Expression<KDL::Rotation>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        // NOTE: This type of expression not part of the original KDL::expressiongraph
        //       library. It is actually part of giskard_core.
        return KDL::slerp(get_from()->get_expression(scope),
            get_to()->get_expression(scope), 
            get_param()->get_expression(scope));
      }

    private:
      RotationSpecPtr from_, to_;
      DoubleSpecPtr param_;
  };

  typedef typename boost::shared_ptr<SlerpSpec> SlerpSpecPtr;

  inline RotationSpecPtr slerp_spec(const RotationSpecPtr& from, const RotationSpecPtr& to, const DoubleSpecPtr& param)
  {
    return SlerpSpecPtr(new SlerpSpec(from, to, param));
  }

  class RotationReferenceSpec : public RotationSpec
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

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const RotationReferenceSpec*>(&other))
          return false;

        return (dynamic_cast<const RotationReferenceSpec*>(&other)->get_reference_name().compare(this->get_reference_name()) == 0);
      }

      virtual KDL::Expression<KDL::Rotation>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return scope.find_rotation_expression(get_reference_name());
      }

    private:
      std::string reference_name_;
  };

  typedef typename boost::shared_ptr<RotationReferenceSpec> RotationReferenceSpecPtr;

  class InverseRotationSpec : public RotationSpec
  {
    public:
      InverseRotationSpec() :
        rotation_( quaternion_spec(0, 0, 0, 1) ) {}
      InverseRotationSpec(const InverseRotationSpec& other) :
        rotation_( other.get_rotation() ) {}
      InverseRotationSpec(const RotationSpecPtr& rotation) :
        rotation_( rotation ) {}
      ~InverseRotationSpec() {}

      const RotationSpecPtr& get_rotation() const
      {
        return rotation_;
      }

      void set_rotation(const RotationSpecPtr& rotation)
      {
        rotation_ = rotation;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const InverseRotationSpec*>(&other))
          return false;

        return dynamic_cast<const InverseRotationSpec*>(&other)->get_rotation()->equals(*(this->get_rotation()));
      }

      virtual KDL::Expression<KDL::Rotation>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::inv(get_rotation()->get_expression(scope));
      }

    private:
      RotationSpecPtr rotation_;
  };

  typedef typename boost::shared_ptr<InverseRotationSpec> InverseRotationSpecPtr;
  
  inline InverseRotationSpecPtr inverse_rotation_spec(const RotationSpecPtr& rotation)
  {
    return InverseRotationSpecPtr(new InverseRotationSpec(rotation));
  }

  class RotationMultiplicationSpec: public RotationSpec
  {
    public:
      RotationMultiplicationSpec() :
        inputs_( std::vector<RotationSpecPtr>() ) {}
      RotationMultiplicationSpec(const RotationMultiplicationSpec& other) :
        inputs_ ( other.get_inputs() ) {}
      RotationMultiplicationSpec(const std::vector<RotationSpecPtr>& inputs) :
        inputs_( inputs ) {}
      ~RotationMultiplicationSpec() {}

      const std::vector<RotationSpecPtr>& get_inputs() const
      {
        return inputs_;
      }

      void set_inputs(const std::vector<RotationSpecPtr>& inputs)
      {
        inputs_ = inputs;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const RotationMultiplicationSpec*>(&other))
          return false;

        const RotationMultiplicationSpec* other_p = 
          dynamic_cast<const RotationMultiplicationSpec*>(&other);

        if(get_inputs().size() != other_p->get_inputs().size())
          return false;

        if(!inputs_valid() || !other_p->inputs_valid())
          return false;

        for(size_t i=0; i<get_inputs().size(); ++i)
          if(!get_inputs()[i]->equals(*(other_p->get_inputs()[i])))
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

      virtual KDL::Expression<KDL::Rotation>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        KDL::Expression<KDL::Rotation>::Ptr result = KDL::Constant(KDL::Rotation::Identity());

        using KDL::operator*;
        for(size_t i=0; i<get_inputs().size(); ++i)
          result = result * get_inputs()[i]->get_expression(scope);

        return result; 
      }

    private:
      std::vector<giskard_core::RotationSpecPtr> inputs_;
  };

  typedef typename boost::shared_ptr<RotationMultiplicationSpec> RotationMultiplicationSpecPtr;

  inline RotationMultiplicationSpecPtr rotation_multiplication_spec(const std::vector<RotationSpecPtr>& inputs)
  {
    return RotationMultiplicationSpecPtr(new RotationMultiplicationSpec(inputs));
  }

  ///
  /// specifications for frame expresssions
  ///

  class FrameCachedSpec: public FrameSpec
  {
    public:
      const giskard_core::FrameSpecPtr& get_frame() const
      {
        return frame_;
      }

      void set_frame(const giskard_core::FrameSpecPtr& frame)
      {
        frame_ = frame;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const FrameCachedSpec*>(&other))
          return false;

        const FrameCachedSpec* other_p = dynamic_cast<const FrameCachedSpec*>(&other);

        return get_frame().get() && other_p->get_frame().get() &&
            get_frame()->equals(*(other_p->get_frame()));
      }

      virtual KDL::Expression<KDL::Frame>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::cached<KDL::Frame>(get_frame()->get_expression(scope));
      }

    private:
      FrameSpecPtr frame_;
  };

  typedef typename boost::shared_ptr<FrameCachedSpec> FrameCachedSpecPtr;

  class FrameConstructorSpec: public FrameSpec
  {
    public:
      FrameConstructorSpec() :
        translation_( vector_constructor_spec() ), rotation_( quaternion_spec(0.0, 0.0, 0.0, 1.0) ) {}
      FrameConstructorSpec(const FrameConstructorSpec& other) :
        translation_( other.get_translation()), rotation_( other.get_rotation() ) {}
      FrameConstructorSpec(const VectorSpecPtr& translation, const RotationSpecPtr& rotation) :
        translation_( translation ), rotation_( rotation ) {}
      ~FrameConstructorSpec() {}

      const giskard_core::VectorSpecPtr& get_translation() const
      {
        return translation_;
      }

      void set_translation(const giskard_core::VectorSpecPtr& translation)
      {
        translation_ = translation;
      }

      const giskard_core::RotationSpecPtr& get_rotation() const
      {
        return rotation_;
      }

      void set_rotation(const giskard_core::RotationSpecPtr& rotation)
      {
        rotation_ = rotation;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const FrameConstructorSpec*>(&other))
          return false;

        const FrameConstructorSpec* other_p = dynamic_cast<const FrameConstructorSpec*>(&other);

        if(!members_valid() || !other_p->members_valid())
          return false;
        
        return (get_translation()->equals(*(other_p->get_translation()))) && 
            (get_rotation()->equals(*(other_p->get_rotation())));
      }

      bool members_valid() const
      {
        return get_translation().get() && get_rotation().get();
      }

      virtual KDL::Expression<KDL::Frame>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        KDL::Expression<KDL::Rotation>::Ptr rot = get_rotation()->get_expression(scope);

        KDL::Expression<KDL::Vector>::Ptr trans = get_translation()->get_expression(scope);
        return KDL::frame(rot, trans);
      }

    private:
      VectorSpecPtr translation_;
      RotationSpecPtr rotation_;
  };

  typedef typename boost::shared_ptr<FrameConstructorSpec> FrameConstructorSpecPtr;

  inline FrameSpecPtr frame_constructor_spec(const VectorSpecPtr& translation, const RotationSpecPtr& rotation)
  {
    return FrameConstructorSpecPtr(new FrameConstructorSpec(translation, rotation));
  }

  class OrientationOfSpec : public RotationSpec
  {
    public:
      OrientationOfSpec() :
        frame_( FrameConstructorSpecPtr(new FrameConstructorSpec())) {}
      OrientationOfSpec(const OrientationOfSpec& other) :
        frame_( other.get_frame() ) {}
      OrientationOfSpec(const FrameSpecPtr& frame) :
        frame_( frame ) {}
      ~OrientationOfSpec() {}

      const giskard_core::FrameSpecPtr& get_frame() const
      {
        return frame_;
      }

      void set_frame(const giskard_core::FrameSpecPtr& frame)
      {
        frame_ = frame;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const OrientationOfSpec*>(&other))
          return false;

        return dynamic_cast<const OrientationOfSpec*>(&other)->get_frame()->equals(*(this->get_frame()));
      }

      virtual KDL::Expression<KDL::Rotation>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::rotation(get_frame()->get_expression(scope));
      }

    private:
      giskard_core::FrameSpecPtr frame_;
  };

  typedef typename boost::shared_ptr<OrientationOfSpec> OrientationOfSpecPtr;

  inline OrientationOfSpecPtr orientation_of_spec(const FrameSpecPtr& frame)
  {
    return OrientationOfSpecPtr(new OrientationOfSpec(frame));
  }

  class FrameMultiplicationSpec: public FrameSpec
  {
    public:
      FrameMultiplicationSpec() {}
      FrameMultiplicationSpec(const FrameMultiplicationSpec& other) :
        inputs_( other.get_inputs()) {}
      FrameMultiplicationSpec(const std::vector<FrameSpecPtr>& inputs) :
        inputs_( inputs ) {}
      ~FrameMultiplicationSpec() {}

      const std::vector<FrameSpecPtr>& get_inputs() const
      {
        return inputs_;
      }

      void set_inputs(const std::vector<FrameSpecPtr>& inputs)
      {
        inputs_ = inputs;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const FrameMultiplicationSpec*>(&other))
          return false;

        const FrameMultiplicationSpec* other_p = dynamic_cast<const FrameMultiplicationSpec*>(&other);

        if(get_inputs().size() != other_p->get_inputs().size())
          return false;

        if(!inputs_valid() || !other_p->inputs_valid())
          return false;

        for(size_t i=0; i<get_inputs().size(); ++i)
          if(!get_inputs()[i]->equals(*(other_p->get_inputs()[i])))
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

      virtual KDL::Expression<KDL::Frame>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        KDL::Expression<KDL::Frame>::Ptr result = KDL::Constant(KDL::Frame::Identity());

        using KDL::operator*;
        for(size_t i=0; i<get_inputs().size(); ++i)
          result = result * get_inputs()[i]->get_expression(scope);

        return result;
      }

    private:
      std::vector<giskard_core::FrameSpecPtr> inputs_;
  };

  typedef typename boost::shared_ptr<FrameMultiplicationSpec> FrameMultiplicationSpecPtr;

  inline FrameMultiplicationSpecPtr frame_multiplication_spec(const std::vector<FrameSpecPtr>& inputs = {})
  {
    return FrameMultiplicationSpecPtr(new FrameMultiplicationSpec(inputs));
  }

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

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const FrameReferenceSpec*>(&other))
          return false;

        return (dynamic_cast<const FrameReferenceSpec*>(&other)->get_reference_name().compare(this->get_reference_name()) == 0);
      }

      virtual KDL::Expression<KDL::Frame>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return scope.find_frame_expression(get_reference_name());
      }

    private:
      std::string reference_name_;
  };

  typedef typename boost::shared_ptr<FrameReferenceSpec> FrameReferenceSpecPtr;

  class InverseFrameSpec : public FrameSpec
  {
    public:
      InverseFrameSpec() :
        frame_( frame_constructor_spec() ) {}
      InverseFrameSpec(const InverseFrameSpec& other) :
        frame_( other.get_frame() ) {}
      InverseFrameSpec(const FrameSpecPtr& frame) :
        frame_( frame ) {}
      ~InverseFrameSpec() {}

      const FrameSpecPtr& get_frame() const
      {
        return frame_;
      }

      void set_frame(const FrameSpecPtr& frame)
      {
        frame_ = frame;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const InverseFrameSpec*>(&other))
          return false;

        return dynamic_cast<const InverseFrameSpec*>(&other)->get_frame()->equals(*(this->get_frame()));
      }

      virtual KDL::Expression<KDL::Frame>::Ptr get_expression(const giskard_core::Scope& scope)
      {
        return KDL::inv(get_frame()->get_expression(scope));
      }

    private:
      FrameSpecPtr frame_;
  };

  typedef typename boost::shared_ptr<InverseFrameSpec> InverseFrameSpecPtr;
  
  inline InverseFrameSpecPtr inverse_frame_spec(const FrameSpecPtr& frame =
      frame_constructor_spec())
  {
    return InverseFrameSpecPtr(new InverseFrameSpec(frame));
  }

  ///
  /// Specification of a Scope
  ///

  class ScopeEntry 
  {
    public:
      ScopeEntry() {}
      ScopeEntry(const ScopeEntry& other):
        name(other.name), spec(other.spec) {}
      ScopeEntry(const std::string& new_name, const SpecPtr& new_spec) :
        name(new_name), spec(new_spec) {}
      ~ScopeEntry() {}

      std::string name;
      SpecPtr spec;
  };

  typedef std::vector<ScopeEntry> ScopeSpec;

  class ControllableConstraintSpec
  {
    public:
      giskard_core::DoubleSpecPtr lower_, upper_, weight_;
      size_t input_number_;
      std::string name_;
  };

  typedef typename boost::shared_ptr<ControllableConstraintSpec> ControllableConstraintSpecPtr;
  
  class SoftConstraintSpec
  {
    public:
      giskard_core::DoubleSpecPtr expression_, lower_, upper_, weight_;
      std::string name_;
  };

  typedef typename boost::shared_ptr<SoftConstraintSpec> SoftConstraintSpecPtr;

  class HardConstraintSpec
  {
    public:
      giskard_core::DoubleSpecPtr expression_, lower_, upper_;
  };

  typedef typename boost::shared_ptr<HardConstraintSpec> HardConstraintSpecPtr;

  class QPControllerSpec
  {
    public:
      /* NOTE: The scope specification cannot be turned into a map even though it
       *       suspicously looks like it could be turned into a map. The reason is
       *       that the order in which the individual specification are evaluated matters.
       *
       *       Example: Our scope contains two simple expressions:
       *         - b = 3
       *         - a = 2 * b
       *
       *       If we read them into a map<string, SpecPtr> (that usually sorts by
       *       string), then we will actually try to generate a before we have
       *       evaluated b. And that will not work.
       */
      std::vector< giskard_core::ScopeEntry > scope_;
      std::vector< giskard_core::ControllableConstraintSpec > controllable_constraints_;
      std::vector< giskard_core::SoftConstraintSpec > soft_constraints_;
      std::vector< giskard_core::HardConstraintSpec > hard_constraints_;
  };
}

#endif // GISKARD_CORE_SPECIFICATIONS_HPP
