/*
 * Copyright (C) 2015 Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

#ifndef GISKARD_SPECIFICATIONS_HPP
#define GISKARD_SPECIFICATIONS_HPP

#include <string>
#include <iostream>
#include <map>
#include <boost/lexical_cast.hpp>
#include <giskard/expressiontree.hpp>
#include <giskard/scope.hpp>

namespace giskard
{
  ///
  /// base of all specifications of expressions
  ///

  class Spec
  { 
    public:
      virtual bool equals(const Spec& other) const = 0;

      // TODO: extend this with a parameter for indention
      virtual std::string to_string() const = 0;
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
      virtual bool equals(const Spec& other) const = 0;

      virtual std::string to_string() const = 0;

      virtual KDL::Expression<double>::Ptr get_expression(const giskard::Scope& scope) = 0;
  };

  typedef typename boost::shared_ptr<DoubleSpec> DoubleSpecPtr;

  class VectorSpec : public Spec
  {
    public:
      virtual bool equals(const Spec& other) const = 0;

      virtual std::string to_string() const = 0;

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard::Scope& scope) = 0;
  };

  typedef typename boost::shared_ptr<VectorSpec> VectorSpecPtr;

  class RotationSpec : public Spec
  {
    public:
      virtual bool equals(const Spec& other) const = 0;

      virtual std::string to_string() const = 0;

      virtual KDL::Expression<KDL::Rotation>::Ptr get_expression(const giskard::Scope& scope) = 0;
  };

  typedef typename boost::shared_ptr<RotationSpec> RotationSpecPtr;

  class FrameSpec : public Spec
  {
    public:
      virtual bool equals(const Spec& other) const = 0;

      virtual std::string to_string() const = 0;

      virtual KDL::Expression<KDL::Frame>::Ptr get_expression(const giskard::Scope& scope) = 0;
  };

  typedef typename boost::shared_ptr<FrameSpec> FrameSpecPtr;

  ///
  /// specifications of double expressions
  ///

  class DoubleConstSpec : public DoubleSpec
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

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleConstSpec*>(&other))
          return false;

        return KDL::epsilon >
            std::abs(dynamic_cast<const DoubleConstSpec*>(&other)->get_value() - this->get_value());
      }

      virtual std::string to_string() const
      {
        return boost::lexical_cast<std::string>(get_value());
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard::Scope& scope)
      {
        return KDL::Constant(get_value());
      }

    private:
      double value_;
  };

  typedef typename boost::shared_ptr<DoubleConstSpec> DoubleConstSpecPtr;

  class DoubleInputSpec : public DoubleSpec
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

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleInputSpec*>(&other))
          return false;

        return dynamic_cast<const DoubleInputSpec*>(&other)->get_input_num() == this->get_input_num();
      }

      virtual std::string to_string() const
      {
        return "todo: implement me";
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard::Scope& scope)
      {
        return KDL::input(get_input_num());
      }

    private:
      size_t input_num_;
  };

  typedef typename boost::shared_ptr<DoubleInputSpec> DoubleInputSpecPtr;

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

      virtual std::string to_string() const
      {
        return "todo: implement me";
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard::Scope& scope)
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

      virtual std::string to_string() const
      {
        return "todo: implement me";
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard::Scope& scope)
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

  class DoubleSubtractionSpec: public DoubleSpec
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

      virtual std::string to_string() const
      {
        // todo: implement me
        return "";
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard::Scope& scope)
      {
        // todo: throw exception here
        assert(get_inputs().size() > 0);

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
      std::vector<giskard::DoubleSpecPtr> inputs_;
  };

  typedef typename boost::shared_ptr<DoubleSubtractionSpec> DoubleSubtractionSpecPtr;

  class DoubleNormOfSpec : public DoubleSpec
  {
    public:
      const giskard::VectorSpecPtr& get_vector() const
      {
        return vector_;
      }

      void set_vector(const giskard::VectorSpecPtr& vector)
      {
        vector_ = vector;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleNormOfSpec*>(&other))
          return false;

        return dynamic_cast<const DoubleNormOfSpec*>(&other)->get_vector()->equals(*(this->get_vector()));
      }

      virtual std::string to_string() const
      {
        return "todo: implement me";
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard::Scope& scope)
      {
        return KDL::norm(get_vector()->get_expression(scope));
      }

    private:
      giskard::VectorSpecPtr vector_;
  };

  typedef typename boost::shared_ptr<DoubleNormOfSpec> DoubleNormOfSpecPtr;

  class DoubleMultiplicationSpec: public DoubleSpec
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

      virtual std::string to_string() const
      {
        // todo: implement me
        return "";
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard::Scope& scope)
      {
        KDL::Expression<double>::Ptr result = KDL::Constant(1.0);

        using KDL::operator*;
        for(size_t i=0; i<get_inputs().size(); ++i)
          result = result * get_inputs()[i]->get_expression(scope);

        return result; 
      }

    private:
      std::vector<giskard::DoubleSpecPtr> inputs_;
  };

  typedef typename boost::shared_ptr<DoubleMultiplicationSpec> DoubleMultiplicationSpecPtr;

  class DoubleDivisionSpec: public DoubleSpec
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

      virtual std::string to_string() const
      {
        // todo: implement me
        return "";
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard::Scope& scope)
      {
        // todo: throw exception here
        assert(get_inputs().size() > 0);

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
      std::vector<giskard::DoubleSpecPtr> inputs_;
  };

  typedef typename boost::shared_ptr<DoubleDivisionSpec> DoubleDivisionSpecPtr;

  class DoubleXCoordOfSpec : public DoubleSpec
  {
    public:
      const giskard::VectorSpecPtr& get_vector() const
      {
        return vector_;
      }

      void set_vector(const giskard::VectorSpecPtr& vector)
      {
        vector_ = vector;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleXCoordOfSpec*>(&other))
          return false;

        return dynamic_cast<const DoubleXCoordOfSpec*>(&other)->get_vector()->equals(*(this->get_vector()));
      }

      virtual std::string to_string() const
      {
        return "todo: implement me";
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard::Scope& scope)
      {
        return KDL::coord_x(get_vector()->get_expression(scope));
      }

    private:
      giskard::VectorSpecPtr vector_;
  };

  typedef typename boost::shared_ptr<DoubleXCoordOfSpec> DoubleXCoordOfSpecPtr;

  class DoubleYCoordOfSpec : public DoubleSpec
  {
    public:
      const giskard::VectorSpecPtr& get_vector() const
      {
        return vector_;
      }

      void set_vector(const giskard::VectorSpecPtr& vector)
      {
        vector_ = vector;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleYCoordOfSpec*>(&other))
          return false;

        return dynamic_cast<const DoubleYCoordOfSpec*>(&other)->get_vector()->equals(*(this->get_vector()));
      }

      virtual std::string to_string() const
      {
        return "todo: implement me";
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard::Scope& scope)
      {
        return KDL::coord_y(get_vector()->get_expression(scope));
      }

    private:
      giskard::VectorSpecPtr vector_;
  };

  typedef typename boost::shared_ptr<DoubleYCoordOfSpec> DoubleYCoordOfSpecPtr;

  class DoubleZCoordOfSpec : public DoubleSpec
  {
    public:
      const giskard::VectorSpecPtr& get_vector() const
      {
        return vector_;
      }

      void set_vector(const giskard::VectorSpecPtr& vector)
      {
        vector_ = vector;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const DoubleZCoordOfSpec*>(&other))
          return false;

        return dynamic_cast<const DoubleZCoordOfSpec*>(&other)->get_vector()->equals(*(this->get_vector()));
      }

      virtual std::string to_string() const
      {
        return "todo: implement me";
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard::Scope& scope)
      {
        return KDL::coord_z(get_vector()->get_expression(scope));
      }

    private:
      giskard::VectorSpecPtr vector_;
  };

  typedef typename boost::shared_ptr<DoubleZCoordOfSpec> DoubleZCoordOfSpecPtr;

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

      virtual std::string to_string() const
      {
        // todo: implement me
        return "";
      }

      virtual KDL::Expression<double>::Ptr get_expression(const giskard::Scope& scope)
      {
        return KDL::dot(get_lhs()->get_expression(scope), get_rhs()->get_expression(scope));
      }

    private:
      giskard::VectorSpecPtr lhs_, rhs_;
  };

  typedef typename boost::shared_ptr<VectorDotSpec> VectorDotSpecPtr;

  ///
  /// specifications of vector expressions
  ///

  class VectorConstructorSpec: public VectorSpec
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

      virtual std::string to_string() const
      {
        std::string result = "type: VECTOR3(";
        result += get_x()->to_string() + ", ";
        result += get_y()->to_string() + ", ";
        result += get_z()->to_string() + ")";
        return result;
      }

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard::Scope& scope)
      {
        return KDL::vector(get_x()->get_expression(scope), 
            get_y()->get_expression(scope), get_z()->get_expression(scope));
      }

    private:
      DoubleSpecPtr x_, y_, z_;
  };

  typedef typename boost::shared_ptr<VectorConstructorSpec> VectorConstructorSpecPtr;

  class VectorSubtractionSpec: public VectorSpec
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

      virtual std::string to_string() const
      {
        // todo: implement me
        return "";
      }

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard::Scope& scope)
      {
        // todo: throw exception here
        assert(get_inputs().size() > 0);

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
      std::vector<giskard::VectorSpecPtr> inputs_;
  };

  typedef typename boost::shared_ptr<VectorSubtractionSpec> VectorSubtractionSpecPtr;

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

      virtual std::string to_string() const
      {
        return "todo: implement me";
      }

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard::Scope& scope)
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
      const giskard::FrameSpecPtr& get_frame() const
      {
        return frame_;
      }

      void set_frame(const giskard::FrameSpecPtr& frame)
      {
        frame_ = frame;
      }

      virtual bool equals(const Spec& other) const
      {
        if(!dynamic_cast<const VectorOriginOfSpec*>(&other))
          return false;

        return dynamic_cast<const VectorOriginOfSpec*>(&other)->get_frame()->equals(*(this->get_frame()));
      }

      virtual std::string to_string() const
      {
        return "todo: implement me";
      }

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard::Scope& scope)
      {
        return KDL::origin(get_frame()->get_expression(scope));
      }

    private:
      giskard::FrameSpecPtr frame_;
  };

  typedef typename boost::shared_ptr<VectorOriginOfSpec> VectorOriginOfSpecPtr;

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

      virtual std::string to_string() const
      {
        // todo: implement me
        return "";
      }

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard::Scope& scope)
      {
        using KDL::operator*;

        return get_frame()->get_expression(scope) * get_vector()->get_expression(scope);
      }

    private:
      VectorSpecPtr vector_;
      FrameSpecPtr frame_;
  };

  typedef typename boost::shared_ptr<VectorFrameMultiplicationSpec> VectorFrameMultiplicationSpecPtr;

  class VectorDoubleMultiplicationSpec: public VectorSpec
  {
    public:
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

      virtual std::string to_string() const
      {
        // todo: implement me
        return "";
      }

      virtual KDL::Expression<KDL::Vector>::Ptr get_expression(const giskard::Scope& scope)
      {
        using KDL::operator*;

        return get_double()->get_expression(scope) * get_vector()->get_expression(scope);
      }

    private:
      VectorSpecPtr vector_;
      DoubleSpecPtr double_;
  };

  typedef typename boost::shared_ptr<VectorDoubleMultiplicationSpec> VectorDoubleMultiplicationSpecPtr;

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

      virtual KDL::Expression<KDL::Rotation>::Ptr get_expression(const giskard::Scope& scope)
      {
        // note: this type of rotation expressions only expect expressions for their axis, not
        //       the angle. while this my make sense, it does break code symmetry.
        KDL::Expression<double>::Ptr angle = get_angle()->get_expression(scope);
        KDL::Vector axis = get_axis()->get_expression(scope)->value();

        return KDL::rot(axis, angle);
      }

    private:
      VectorSpecPtr axis_;
      DoubleSpecPtr angle_;
  };

  typedef typename boost::shared_ptr<AxisAngleSpec> AxisAngleSpecPtr;

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

      virtual std::string to_string() const
      {
        return "todo: implement me";
      }

      virtual KDL::Expression<KDL::Rotation>::Ptr get_expression(const giskard::Scope& scope)
      {
        return scope.find_rotation_expression(get_reference_name());
      }

    private:
      std::string reference_name_;
  };

  typedef typename boost::shared_ptr<RotationReferenceSpec> RotationReferenceSpecPtr;

  ///
  /// specifications for frame expresssions
  ///

  class FrameConstructorSpec: public FrameSpec
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

      virtual std::string to_string() const
      {
        std::string result = "type: FRAME\ntranslation:\n";
        result += get_translation()->to_string() + "\nrotation:\n";
        result += get_rotation()->to_string();
        return result;
      }

      virtual KDL::Expression<KDL::Frame>::Ptr get_expression(const giskard::Scope& scope)
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

  class FrameMultiplicationSpec: public FrameSpec
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

      virtual std::string to_string() const
      {
        std::string result = "type: FRAME-MULTIPLICATION\ninputs:[";
        for(size_t i=0; i<get_inputs().size(); ++i)
          result += "\n" + get_inputs()[i]->to_string();
        result += "]";
        return result;
      }

      virtual KDL::Expression<KDL::Frame>::Ptr get_expression(const giskard::Scope& scope)
      {
        KDL::Expression<KDL::Frame>::Ptr result = KDL::Constant(KDL::Frame::Identity());

        using KDL::operator*;
        for(size_t i=0; i<get_inputs().size(); ++i)
          result = result * get_inputs()[i]->get_expression(scope);

        return result;
      }

    private:
      std::vector<giskard::FrameSpecPtr> inputs_;
  };

  typedef typename boost::shared_ptr<FrameMultiplicationSpec> FrameMultiplicationSpecPtr;

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

      virtual std::string to_string() const
      {
        return "todo: implement me";
      }

      virtual KDL::Expression<KDL::Frame>::Ptr get_expression(const giskard::Scope& scope)
      {
        return scope.find_frame_expression(get_reference_name());
      }

    private:
      std::string reference_name_;
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

  class ControllableConstraintSpec
  {
    public:
      giskard::DoubleSpecPtr lower_, upper_, weight_;
      size_t input_number_;
  };

  typedef typename boost::shared_ptr<ControllableConstraintSpec> ControllableConstraintSpecPtr;
  
  class SoftConstraintSpec
  {
    public:
      giskard::DoubleSpecPtr expression_, lower_, upper_, weight_;
  };

  typedef typename boost::shared_ptr<SoftConstraintSpec> SoftConstraintSpecPtr;

  class HardConstraintSpec
  {
    public:
      giskard::DoubleSpecPtr expression_, lower_, upper_;
  };

  typedef typename boost::shared_ptr<HardConstraintSpec> HardConstraintSpecPtr;

  class QPControllerSpec
  {
    public:
      std::vector< giskard::ScopeEntry > scope_;
      std::vector< giskard::ControllableConstraintSpec > controllable_constraints_;
      std::vector< giskard::SoftConstraintSpec > soft_constraints_;
      std::vector< giskard::HardConstraintSpec > hard_constraints_;
  };
}

#endif // GISKARD_SPECIFICATIONS_HPP
