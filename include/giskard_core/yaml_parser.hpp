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

#ifndef GISKARD_CORE_YAML_PARSER_HPP
#define GISKARD_CORE_YAML_PARSER_HPP

#include <yaml-cpp/yaml.h>
#include <vector>
#include <giskard_core/specifications.hpp>

namespace YAML {

  // 
  // parsing of double specs
  //

  inline bool is_const_double(const Node& node)
  {
    if(!node.IsScalar())
      return false;

    try
    {
      node.as<double>();
      return true;
    }
    catch (const YAML::Exception& e)
    {
      return false;
    }
  }

  template<>
  struct convert<giskard_core::DoubleConstSpecPtr> 
  {
    
    static Node encode(const giskard_core::DoubleConstSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_value();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::DoubleConstSpecPtr& rhs) 
    {
      if(!is_const_double(node))
        return false;
  
      rhs = giskard_core::DoubleConstSpecPtr(new giskard_core::DoubleConstSpec());
      rhs->set_value(node.as<double>());

      return true;
    }
  };

  inline bool is_input_scalar(const Node& node)
  {
    if (node.IsMap() && (node.size() == 1) && node["input-scalar"]) {
      try
      {
        node["input-scalar"].as<std::string>();
        return true;
      }
      catch (const YAML::Exception& e)
      { }
    }
    return false;
  }

  template<>
  struct convert<giskard_core::DoubleInputSpecPtr> 
  {
    
    static Node encode(const giskard_core::DoubleInputSpecPtr& rhs) 
    {
      Node node;
      node["input-scalar"] = rhs->get_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::DoubleInputSpecPtr& rhs) 
    {
      if(!is_input_scalar(node))
        return false;
  
      rhs = giskard_core::DoubleInputSpecPtr(new giskard_core::DoubleInputSpec(node["input-scalar"].as<std::string>()));

      return true;
    }
  };

  inline bool is_input_joint(const Node& node)
  {
    if (node.IsMap() && (node.size() == 1) && node["input-joint"]) {
      try
      {
        node["input-joint"].as<std::string>();
        return true;
      }
      catch (const YAML::Exception& e)
      { }
    }
    return false;
  }

  template<>
  struct convert<giskard_core::JointInputSpecPtr> 
  {
    
    static Node encode(const giskard_core::JointInputSpecPtr& rhs) 
    {
      Node node;
      node["input-joint"] = rhs->get_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::JointInputSpecPtr& rhs) 
    {
      if(!is_input_joint(node))
        return false;
  
      rhs = giskard_core::JointInputSpecPtr(new giskard_core::JointInputSpec(node["input-joint"].as<std::string>()));

      return true;
    }
  };

  inline bool is_double_reference(const Node& node)
  {
    if(!node.IsScalar())
      return false;

    try
    {
      node.as<std::string>();
      return true;
    }
    catch (const YAML::Exception& e)
    {
      return false;
    }
  }

  template<>
  struct convert<giskard_core::DoubleReferenceSpecPtr> 
  {
    
    static Node encode(const giskard_core::DoubleReferenceSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_reference_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::DoubleReferenceSpecPtr& rhs) 
    {
      if(!is_double_reference(node))
        return false;
 
      rhs = giskard_core::DoubleReferenceSpecPtr(new giskard_core::DoubleReferenceSpec());
      rhs->set_reference_name(node.as<std::string>());

      return true;
    }
  };

  inline bool is_double_addition(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["double-add"] &&
        node["double-add"].IsSequence();
  }

  template<>
  struct convert<giskard_core::DoubleAdditionSpecPtr> 
  {
    static Node encode(const giskard_core::DoubleAdditionSpecPtr& rhs) 
    {
      Node node;
      node["double-add"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::DoubleAdditionSpecPtr& rhs) 
    {
      if(!is_double_addition(node))
        return false;

      rhs = giskard_core::DoubleAdditionSpecPtr(new giskard_core::DoubleAdditionSpec()); 
      rhs->set_inputs(node["double-add"].as< std::vector<giskard_core::DoubleSpecPtr> >());

      return true;
    }
  };

  inline bool is_double_subtraction(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["double-sub"] &&
        node["double-sub"].IsSequence();
  }

  template<>
  struct convert<giskard_core::DoubleSubtractionSpecPtr> 
  {
    static Node encode(const giskard_core::DoubleSubtractionSpecPtr& rhs) 
    {
      Node node;
      node["double-sub"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::DoubleSubtractionSpecPtr& rhs) 
    {
      if(!is_double_subtraction(node))
        return false;

      rhs = giskard_core::DoubleSubtractionSpecPtr(new giskard_core::DoubleSubtractionSpec()); 
      rhs->set_inputs(node["double-sub"].as< std::vector<giskard_core::DoubleSpecPtr> >());

      return true;
    }
  };

  inline bool is_double_norm_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["vector-norm"];
  }

  template<>
  struct convert<giskard_core::DoubleNormOfSpecPtr> 
  {
    
    static Node encode(const giskard_core::DoubleNormOfSpecPtr& rhs) 
    {
      Node node;
      node["vector-norm"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::DoubleNormOfSpecPtr& rhs) 
    {
      if(!is_double_norm_of(node))
        return false;
  
      rhs = giskard_core::DoubleNormOfSpecPtr(new giskard_core::DoubleNormOfSpec());
      rhs->set_vector(node["vector-norm"].as<giskard_core::VectorSpecPtr>());

      return true;
    }
  };

  inline bool is_double_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["double-mul"] &&
        node["double-mul"].IsSequence();
  }

  template<>
  struct convert<giskard_core::DoubleMultiplicationSpecPtr> 
  {
    static Node encode(const giskard_core::DoubleMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["double-mul"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::DoubleMultiplicationSpecPtr& rhs) 
    {
      if(!is_double_multiplication(node))
        return false;

      rhs = giskard_core::DoubleMultiplicationSpecPtr(new giskard_core::DoubleMultiplicationSpec()); 
      rhs->set_inputs(node["double-mul"].as< std::vector<giskard_core::DoubleSpecPtr> >());

      return true;
    }
  };

  inline bool is_double_division(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["double-div"] &&
        node["double-div"].IsSequence();
  }

  template<>
  struct convert<giskard_core::DoubleDivisionSpecPtr> 
  {
    static Node encode(const giskard_core::DoubleDivisionSpecPtr& rhs) 
    {
      Node node;
      node["double-div"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::DoubleDivisionSpecPtr& rhs) 
    {
      if(!is_double_division(node))
        return false;

      rhs = giskard_core::DoubleDivisionSpecPtr(new giskard_core::DoubleDivisionSpec()); 
      rhs->set_inputs(node["double-div"].as< std::vector<giskard_core::DoubleSpecPtr> >());

      return true;
    }
  };

  inline bool is_x_coord_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["x-coord"];
  }

  template<>
  struct convert<giskard_core::DoubleXCoordOfSpecPtr> 
  {
    static Node encode(const giskard_core::DoubleXCoordOfSpecPtr& rhs) 
    {
      Node node;
      node["x-coord"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::DoubleXCoordOfSpecPtr& rhs) 
    {
      if(!is_x_coord_of(node))
        return false;

      rhs = giskard_core::DoubleXCoordOfSpecPtr(new giskard_core::DoubleXCoordOfSpec()); 
      rhs->set_vector(node["x-coord"].as< giskard_core::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_y_coord_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["y-coord"];
  }

  template<>
  struct convert<giskard_core::DoubleYCoordOfSpecPtr> 
  {
    static Node encode(const giskard_core::DoubleYCoordOfSpecPtr& rhs) 
    {
      Node node;
      node["y-coord"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::DoubleYCoordOfSpecPtr& rhs) 
    {
      if(!is_y_coord_of(node))
        return false;

      rhs = giskard_core::DoubleYCoordOfSpecPtr(new giskard_core::DoubleYCoordOfSpec()); 
      rhs->set_vector(node["y-coord"].as< giskard_core::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_z_coord_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["z-coord"];
  }

  template<>
  struct convert<giskard_core::DoubleZCoordOfSpecPtr> 
  {
    static Node encode(const giskard_core::DoubleZCoordOfSpecPtr& rhs) 
    {
      Node node;
      node["z-coord"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::DoubleZCoordOfSpecPtr& rhs) 
    {
      if(!is_z_coord_of(node))
        return false;

      rhs = giskard_core::DoubleZCoordOfSpecPtr(new giskard_core::DoubleZCoordOfSpec()); 
      rhs->set_vector(node["z-coord"].as< giskard_core::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_vector_dot(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["vector-dot"] &&
        node["vector-dot"].IsSequence() && (node["vector-dot"].size() == 2);
  }

  template<>
  struct convert<giskard_core::VectorDotSpecPtr>
  {
    static Node encode(const giskard_core::VectorDotSpecPtr& rhs) 
    {
      Node node;
      node["vector-dot"][0] = rhs->get_lhs();
      node["vector-dot"][1] = rhs->get_rhs();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::VectorDotSpecPtr& rhs) 
    {
      if(!is_vector_dot(node))
        return false;

      rhs = giskard_core::VectorDotSpecPtr(new giskard_core::VectorDotSpec()); 
      rhs->set_lhs(node["vector-dot"][0].as< giskard_core::VectorSpecPtr >());
      rhs->set_rhs(node["vector-dot"][1].as< giskard_core::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_abs(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["abs"];
  }

  template<>
  struct convert<giskard_core::AbsSpecPtr>
  {
    static Node encode(const giskard_core::AbsSpecPtr& rhs) 
    {
      Node node;
      node["abs"] = rhs->get_value();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::AbsSpecPtr& rhs) 
    {
      if(!is_abs(node))
        return false;

      rhs = giskard_core::AbsSpecPtr(new giskard_core::AbsSpec()); 
      rhs->set_value(node["abs"].as< giskard_core::DoubleSpecPtr >());

      return true;
    }
  };

  inline bool is_fmod(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["fmod"] &&
      node["fmod"].IsSequence() && (node["fmod"].size() == 2);
  }

  template<>
  struct convert<giskard_core::FmodSpecPtr>
  {
    static Node encode(const giskard_core::FmodSpecPtr& rhs) 
    {
      Node node;
      node["fmod"][0] = rhs->get_nominator();
      node["fmod"][1] = rhs->get_denominator();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::FmodSpecPtr& rhs) 
    {
      if(!is_fmod(node))
        return false;

      rhs = giskard_core::FmodSpecPtr(new giskard_core::FmodSpec()); 
      rhs->set_nominator(node["fmod"][0].as< giskard_core::DoubleSpecPtr >());
      rhs->set_denominator(node["fmod"][1].as< giskard_core::DoubleSpecPtr >());

      return true;
    }
  };

  inline bool is_min(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["min"] &&
        node["min"].IsSequence() && (node["min"].size() == 2);
  }

  template<>
  struct convert<giskard_core::MinSpecPtr>
  {
    static Node encode(const giskard_core::MinSpecPtr& rhs) 
    {
      Node node;
      node["min"][0] = rhs->get_lhs();
      node["min"][1] = rhs->get_rhs();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::MinSpecPtr& rhs) 
    {
      if(!is_min(node))
        return false;

      rhs = giskard_core::MinSpecPtr(new giskard_core::MinSpec()); 
      rhs->set_lhs(node["min"][0].as< giskard_core::DoubleSpecPtr >());
      rhs->set_rhs(node["min"][1].as< giskard_core::DoubleSpecPtr >());

      return true;
    }
  };

  inline bool is_double_if(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["double-if"] &&
        node["double-if"].IsSequence() && (node["double-if"].size() == 3);
  }

  template<>
  struct convert<giskard_core::DoubleIfSpecPtr>
  {
    static Node encode(const giskard_core::DoubleIfSpecPtr& rhs) 
    {
      Node node;
      node["double-if"][0] = rhs->get_condition();
      node["double-if"][1] = rhs->get_if();
      node["double-if"][2] = rhs->get_else();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::DoubleIfSpecPtr& rhs) 
    {
      if(!is_double_if(node))
        return false;

      rhs = giskard_core::DoubleIfSpecPtr(new giskard_core::DoubleIfSpec()); 
      rhs->set_condition(node["double-if"][0].as< giskard_core::DoubleSpecPtr >());
      rhs->set_if(node["double-if"][1].as< giskard_core::DoubleSpecPtr >());
      rhs->set_else(node["double-if"][2].as< giskard_core::DoubleSpecPtr >());

      return true;
    }
  };

  template<>
  struct convert<giskard_core::DoubleSpecPtr> 
  {
    
    static Node encode(const giskard_core::DoubleSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rhs).get())
      {
        giskard_core::DoubleConstSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::DoubleConstSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::DoubleInputSpec>(rhs).get())
      {
        giskard_core::DoubleInputSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::DoubleInputSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::JointInputSpec>(rhs).get()) 
      {
        giskard_core::JointInputSpecPtr p = boost::dynamic_pointer_cast<giskard_core::JointInputSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::DoubleReferenceSpec>(rhs).get())
      {
        giskard_core::DoubleReferenceSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::DoubleReferenceSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::DoubleAdditionSpec>(rhs).get())
      {
        giskard_core::DoubleAdditionSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::DoubleAdditionSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::DoubleSubtractionSpec>(rhs).get())
      {
        giskard_core::DoubleSubtractionSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::DoubleSubtractionSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::DoubleNormOfSpec>(rhs).get())
      {
        giskard_core::DoubleNormOfSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::DoubleNormOfSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::DoubleMultiplicationSpec>(rhs).get())
      {
        giskard_core::DoubleMultiplicationSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::DoubleMultiplicationSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::DoubleDivisionSpec>(rhs).get())
      {
        giskard_core::DoubleDivisionSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::DoubleDivisionSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::DoubleXCoordOfSpec>(rhs).get())
      {
        giskard_core::DoubleXCoordOfSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::DoubleXCoordOfSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::DoubleYCoordOfSpec>(rhs).get())
      {
        giskard_core::DoubleYCoordOfSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::DoubleYCoordOfSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::DoubleZCoordOfSpec>(rhs).get())
      {
        giskard_core::DoubleZCoordOfSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::DoubleZCoordOfSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::VectorDotSpec>(rhs).get())
      {
        giskard_core::VectorDotSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::VectorDotSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::MinSpec>(rhs).get())
      {
        giskard_core::MinSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::MinSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::AbsSpec>(rhs).get())
      {
        giskard_core::AbsSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::AbsSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::FmodSpec>(rhs).get())
      {
        giskard_core::FmodSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::FmodSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard_core::DoubleIfSpec>(rhs).get())
      {
        giskard_core::DoubleIfSpecPtr p = 
            boost::dynamic_pointer_cast<giskard_core::DoubleIfSpec>(rhs);
        node = p;
      }

      return node;
    }
  
    static bool decode(const Node& node, giskard_core::DoubleSpecPtr& rhs) 
    {

      if(is_const_double(node))
      {
        rhs = node.as<giskard_core::DoubleConstSpecPtr>();
        return true;
      }
      else if(is_input_scalar(node))
      {
        rhs = node.as<giskard_core::DoubleInputSpecPtr>();
        return true;
      }
      else if(is_input_joint(node))
      {
        rhs = node.as<giskard_core::JointInputSpecPtr>();
        return true;
      }
      else if(is_double_addition(node))
      {
        rhs = node.as<giskard_core::DoubleAdditionSpecPtr>();
        return true;
      }
      else if(is_double_subtraction(node))
      {
        rhs = node.as<giskard_core::DoubleSubtractionSpecPtr>();
        return true;
      }
      else if(is_double_multiplication(node))
      {
        rhs = node.as<giskard_core::DoubleMultiplicationSpecPtr>();
        return true;
      }
      else if(is_double_division(node))
      {
        rhs = node.as<giskard_core::DoubleDivisionSpecPtr>();
        return true;
      }
      else if(is_double_norm_of(node))
      {
        rhs = node.as<giskard_core::DoubleNormOfSpecPtr>();
        return true;
      }
      else if(is_x_coord_of(node))
      {
        rhs = node.as<giskard_core::DoubleXCoordOfSpecPtr>();
        return true;
      }
      else if(is_y_coord_of(node))
      {
        rhs = node.as<giskard_core::DoubleYCoordOfSpecPtr>();
        return true;
      }
      else if(is_z_coord_of(node))
      {
        rhs = node.as<giskard_core::DoubleZCoordOfSpecPtr>();
        return true;
      }
      else if(is_double_reference(node))
      {
        rhs = node.as<giskard_core::DoubleReferenceSpecPtr>();
        return true;
      }
      else if(is_vector_dot(node))
      {
        rhs = node.as<giskard_core::VectorDotSpecPtr>();
        return true;
      }
      else if(is_min(node))
      {
        rhs = node.as<giskard_core::MinSpecPtr>();
        return true;
      }
      else if(is_abs(node))
      {
        rhs = node.as<giskard_core::AbsSpecPtr>();
        return true;
      }
      else if(is_fmod(node))
      {
        rhs = node.as<giskard_core::FmodSpecPtr>();
        return true;
      }
      else if(is_double_if(node))
      {
        rhs = node.as<giskard_core::DoubleIfSpecPtr>();
        return true;
      }
      else
        return false;
    }
  };
 
  //
  // parsing of vector specs
  //

    inline bool is_input_vec3(const Node& node)
  {
    if (node.IsMap() && (node.size() == 1) && node["input-vec3"]) {
      try
      {
        node["input-vec3"].as<std::string>();
        return true;
      }
      catch (const YAML::Exception& e)
      { }
    }
    return false;
  }

  template<>
  struct convert<giskard_core::VectorInputSpecPtr> 
  {
    
    static Node encode(const giskard_core::VectorInputSpecPtr& rhs) 
    {
      Node node;
      node["input-vec3"] = rhs->get_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::VectorInputSpecPtr& rhs) 
    {
      if(!is_input_vec3(node))
        return false;
  
      rhs = giskard_core::VectorInputSpecPtr(new giskard_core::VectorInputSpec(node["input-vec3"].as<std::string>()));

      return true;
    }
  };

  inline bool is_cached_vector(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["cached-vector"];
  }

  template<>
  struct convert<giskard_core::VectorCachedSpecPtr> 
  {
    static Node encode(const giskard_core::VectorCachedSpecPtr& rhs) 
    {
      Node node;
      node["cached-vector"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::VectorCachedSpecPtr& rhs) 
    {
      if(!is_cached_vector(node))
        return false;

      rhs = giskard_core::VectorCachedSpecPtr(new giskard_core::VectorCachedSpec()); 
      rhs->set_vector(node["cached-vector"].as<giskard_core::VectorSpecPtr>());

      return true;
    }
  };

  inline bool is_constructor_vector(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["vector3"] &&
        node["vector3"].IsSequence() && (node["vector3"].size() == 3);
  }

  template<>
  struct convert<giskard_core::VectorConstructorSpecPtr> 
  {
    static Node encode(const giskard_core::VectorConstructorSpecPtr& rhs) 
    {
      Node node;
      node["vector3"].push_back(rhs->get_x());
      node["vector3"].push_back(rhs->get_y());
      node["vector3"].push_back(rhs->get_z());
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::VectorConstructorSpecPtr& rhs) 
    {
      if(!is_constructor_vector(node))
        return false;

      rhs = giskard_core::VectorConstructorSpecPtr(new giskard_core::VectorConstructorSpec()); 
      rhs->set_x(node["vector3"][0].as<giskard_core::DoubleSpecPtr>());
      rhs->set_y(node["vector3"][1].as<giskard_core::DoubleSpecPtr>());
      rhs->set_z(node["vector3"][2].as<giskard_core::DoubleSpecPtr>());

      return true;
    }
  };

  inline bool is_vector_reference(const Node& node)
  {
    if(!node.IsScalar())
      return false;

    try
    {
      node.as<std::string>();
      return true;
    }
    catch (const YAML::Exception& e)
    {
      return false;
    }
  }

  template<>
  struct convert<giskard_core::VectorReferenceSpecPtr> 
  {
    
    static Node encode(const giskard_core::VectorReferenceSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_reference_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::VectorReferenceSpecPtr& rhs) 
    {
      if(!is_vector_reference(node))
        return false;
  
      rhs = giskard_core::VectorReferenceSpecPtr(new giskard_core::VectorReferenceSpec());
      rhs->set_reference_name(node.as<std::string>());

      return true;
    }
  };

  inline bool is_vector_origin_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["origin-of"];
  }

  template<>
  struct convert<giskard_core::VectorOriginOfSpecPtr> 
  {
    
    static Node encode(const giskard_core::VectorOriginOfSpecPtr& rhs) 
    {
      Node node;
      node["origin-of"] = rhs->get_frame();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::VectorOriginOfSpecPtr& rhs) 
    {
      if(!is_vector_origin_of(node))
        return false;
  
      rhs = giskard_core::VectorOriginOfSpecPtr(new giskard_core::VectorOriginOfSpec());
      rhs->set_frame(node["origin-of"].as<giskard_core::FrameSpecPtr>());

      return true;
    }
  };

  inline bool is_vector_addition(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["vector-add"] &&
        node["vector-add"].IsSequence();
  }

  template<>
  struct convert<giskard_core::VectorAdditionSpecPtr> 
  {
    static Node encode(const giskard_core::VectorAdditionSpecPtr& rhs) 
    {
      Node node;
      node["vector-add"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::VectorAdditionSpecPtr& rhs) 
    {
      if(!is_vector_addition(node))
        return false;

      rhs = giskard_core::VectorAdditionSpecPtr(new giskard_core::VectorAdditionSpec()); 
      rhs->set_inputs(node["vector-add"].as< std::vector<giskard_core::VectorSpecPtr> >());

      return true;
    }
  };

  inline bool is_vector_subtraction(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["vector-sub"] &&
        node["vector-sub"].IsSequence();
  }

  template<>
  struct convert<giskard_core::VectorSubtractionSpecPtr> 
  {
    static Node encode(const giskard_core::VectorSubtractionSpecPtr& rhs) 
    {
      Node node;
      node["vector-sub"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::VectorSubtractionSpecPtr& rhs) 
    {
      if(!is_vector_subtraction(node))
        return false;

      rhs = giskard_core::VectorSubtractionSpecPtr(new giskard_core::VectorSubtractionSpec()); 
      rhs->set_inputs(node["vector-sub"].as< std::vector<giskard_core::VectorSpecPtr> >());

      return true;
    }
  };

  inline bool is_vector_rotation_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["rotate-vector"] &&
        node["rotate-vector"].IsSequence() && (node["rotate-vector"].size() == 2);
  }

  template<>
  struct convert<giskard_core::VectorRotationMultiplicationSpecPtr> 
  {
    static Node encode(const giskard_core::VectorRotationMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["rotate-vector"][0] = rhs->get_rotation();
      node["rotate-vector"][1] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::VectorRotationMultiplicationSpecPtr& rhs) 
    {
      if(!is_vector_rotation_multiplication(node))
        return false;

      rhs = giskard_core::VectorRotationMultiplicationSpecPtr(new giskard_core::VectorRotationMultiplicationSpec()); 
      rhs->set_rotation(node["rotate-vector"][0].as< giskard_core::RotationSpecPtr >());
      rhs->set_vector(node["rotate-vector"][1].as< giskard_core::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_vector_frame_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["transform-vector"] &&
        node["transform-vector"].IsSequence() && (node["transform-vector"].size() == 2);
  }

  template<>
  struct convert<giskard_core::VectorFrameMultiplicationSpecPtr> 
  {
    static Node encode(const giskard_core::VectorFrameMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["transform-vector"][0] = rhs->get_frame();
      node["transform-vector"][1] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::VectorFrameMultiplicationSpecPtr& rhs) 
    {
      if(!is_vector_frame_multiplication(node))
        return false;

      rhs = giskard_core::VectorFrameMultiplicationSpecPtr(new giskard_core::VectorFrameMultiplicationSpec()); 
      rhs->set_frame(node["transform-vector"][0].as< giskard_core::FrameSpecPtr >());
      rhs->set_vector(node["transform-vector"][1].as< giskard_core::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_vector_double_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["scale-vector"] &&
        node["scale-vector"].IsSequence() && (node["scale-vector"].size() == 2);
  }

  template<>
  struct convert<giskard_core::VectorDoubleMultiplicationSpecPtr> 
  {
    static Node encode(const giskard_core::VectorDoubleMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["scale-vector"][0] = rhs->get_double();
      node["scale-vector"][1] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::VectorDoubleMultiplicationSpecPtr& rhs) 
    {
      if(!is_vector_double_multiplication(node))
        return false;

      rhs = giskard_core::VectorDoubleMultiplicationSpecPtr(new giskard_core::VectorDoubleMultiplicationSpec()); 
      rhs->set_double(node["scale-vector"][0].as< giskard_core::DoubleSpecPtr >());
      rhs->set_vector(node["scale-vector"][1].as< giskard_core::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_vector_rotation_vector(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["rot-vector"];
  }

  template<>
  struct convert<giskard_core::VectorRotationVectorSpecPtr> 
  {
    
    static Node encode(const giskard_core::VectorRotationVectorSpecPtr& rhs) 
    {
      Node node;
      node["rot-vector"] = rhs->get_rotation();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::VectorRotationVectorSpecPtr& rhs) 
    {
      if(!is_vector_rotation_vector(node))
        return false;
  
      rhs = giskard_core::VectorRotationVectorSpecPtr(new giskard_core::VectorRotationVectorSpec());
      rhs->set_rotation(node["rot-vector"].as<giskard_core::RotationSpecPtr>());

      return true;
    }
  };

  template<>
  struct convert<giskard_core::VectorSpecPtr> 
  {
    static Node encode(const giskard_core::VectorSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard_core::VectorInputSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::VectorInputSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::VectorCachedSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::VectorCachedSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::VectorConstructorSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::VectorReferenceSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::VectorReferenceSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::VectorOriginOfSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::VectorOriginOfSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::VectorAdditionSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::VectorAdditionSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::VectorSubtractionSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::VectorSubtractionSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::VectorFrameMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::VectorFrameMultiplicationSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::VectorRotationMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::VectorRotationMultiplicationSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::VectorDoubleMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::VectorDoubleMultiplicationSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::VectorRotationVectorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::VectorRotationVectorSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard_core::VectorSpecPtr& rhs) 
    {
      if(is_input_vec3(node))
      {
        rhs = node.as<giskard_core::VectorInputSpecPtr>();
        return true;
      }
      else if(is_cached_vector(node))
      {
        rhs = node.as<giskard_core::VectorCachedSpecPtr>();
        return true;
      }
      else if(is_constructor_vector(node))
      {
        rhs = node.as<giskard_core::VectorConstructorSpecPtr>();
        return true;
      }
      else if(is_vector_reference(node))
      {
        rhs = node.as<giskard_core::VectorReferenceSpecPtr>();
        return true;
      }
      else if(is_vector_origin_of(node))
      {
        rhs = node.as<giskard_core::VectorOriginOfSpecPtr>();
        return true;
      }
      else if(is_vector_addition(node))
      {
        rhs = node.as<giskard_core::VectorAdditionSpecPtr>();
        return true;
      }
      else if(is_vector_subtraction(node))
      {
        rhs = node.as<giskard_core::VectorSubtractionSpecPtr>();
        return true;
      }
      else if(is_vector_frame_multiplication(node))
      {
        rhs = node.as<giskard_core::VectorFrameMultiplicationSpecPtr>();
        return true;
      }
      else if(is_vector_rotation_multiplication(node))
      {
        rhs = node.as<giskard_core::VectorRotationMultiplicationSpecPtr>();
        return true;
      }
      else if(is_vector_double_multiplication(node))
      {
        rhs = node.as<giskard_core::VectorDoubleMultiplicationSpecPtr>();
        return true;
      }
      else if(is_vector_rotation_vector(node))
      {
        rhs = node.as<giskard_core::VectorRotationVectorSpecPtr>();
        return true;
      }
      else
        return false;
    }
  };

  ///
  /// parsing rotation specs
  ///

  inline bool is_input_rotation(const Node& node)
  {
    if (node.IsMap() && (node.size() == 1) && node["input-rotation"]) {
      try
      {
        node["input-rotation"].as<std::string>();
        return true;
      }
      catch (const YAML::Exception& e)
      { }
    }
    return false;
  }

  template<>
  struct convert<giskard_core::RotationInputSpecPtr> 
  {
    
    static Node encode(const giskard_core::RotationInputSpecPtr& rhs) 
    {
      Node node;
      node["input-rotation"] = rhs->get_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::RotationInputSpecPtr& rhs) 
    {
      if(!is_input_rotation(node))
        return false;
  
      rhs = giskard_core::RotationInputSpecPtr(new giskard_core::RotationInputSpec(node["input-rotation"].as<std::string>()));

      return true;
    }
  };

  inline bool is_quaternion_constructor(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["quaternion"] &&
        node["quaternion"].IsSequence() && (node["quaternion"].size() == 4);
  }

  template<>
  struct convert<giskard_core::RotationQuaternionConstructorSpecPtr> 
  {
    static Node encode(const giskard_core::RotationQuaternionConstructorSpecPtr& rhs) 
    {
      Node node;
      node["quaternion"][0] = rhs->get_x();
      node["quaternion"][1] = rhs->get_y();
      node["quaternion"][2] = rhs->get_z();
      node["quaternion"][3] = rhs->get_w();

      return node;
    }
  
    static bool decode(const Node& node, giskard_core::RotationQuaternionConstructorSpecPtr& rhs) 
    {
      if(!is_quaternion_constructor(node))
        return false;

      rhs = giskard_core::RotationQuaternionConstructorSpecPtr(new giskard_core::RotationQuaternionConstructorSpec());
      rhs->set_x(node["quaternion"][0].as<double>());
      rhs->set_y(node["quaternion"][1].as<double>());
      rhs->set_z(node["quaternion"][2].as<double>());
      rhs->set_w(node["quaternion"][3].as<double>());

      return true;
    }
  };

  inline bool is_slerp(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["slerp"] &&
        node["slerp"].IsSequence() && (node["slerp"].size() == 3);
  }

  template<>
  struct convert<giskard_core::SlerpSpecPtr> 
  {
    static Node encode(const giskard_core::SlerpSpecPtr& rhs) 
    {
      Node node;
      node["slerp"][0] = rhs->get_from();
      node["slerp"][1] = rhs->get_to();
      node["slerp"][2] = rhs->get_param();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::SlerpSpecPtr& rhs) 
    {
      if(!is_slerp(node))
        return false;

      rhs = giskard_core::SlerpSpecPtr(new giskard_core::SlerpSpec()); 
      rhs->set_from(node["slerp"][0].as<giskard_core::RotationSpecPtr>());
      rhs->set_to(node["slerp"][1].as<giskard_core::RotationSpecPtr>());
      rhs->set_param(node["slerp"][2].as<giskard_core::DoubleSpecPtr>());

      return true;
    }
  };

  inline bool is_axis_angle(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["axis-angle"] &&
        node["axis-angle"].IsSequence() && (node["axis-angle"].size() == 2);
  }

  template<>
  struct convert<giskard_core::AxisAngleSpecPtr> 
  {
    static Node encode(const giskard_core::AxisAngleSpecPtr& rhs) 
    {
      Node node;
      node["axis-angle"][0] = rhs->get_axis();
      node["axis-angle"][1] = rhs->get_angle();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::AxisAngleSpecPtr& rhs) 
    {
      if(!is_axis_angle(node))
        return false;

      rhs = giskard_core::AxisAngleSpecPtr(new giskard_core::AxisAngleSpec()); 
      rhs->set_axis(node["axis-angle"][0].as<giskard_core::VectorSpecPtr>());
      rhs->set_angle(node["axis-angle"][1].as<giskard_core::DoubleSpecPtr>());

      return true;
    }
  };

  inline bool is_orientation_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["orientation-of"];
  }

  template<>
  struct convert<giskard_core::OrientationOfSpecPtr> 
  {
    
    static Node encode(const giskard_core::OrientationOfSpecPtr& rhs) 
    {
      Node node;
      node["orientation-of"] = rhs->get_frame();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::OrientationOfSpecPtr& rhs) 
    {
      if(!is_orientation_of(node))
        return false;
  
      rhs = giskard_core::OrientationOfSpecPtr(new giskard_core::OrientationOfSpec());
      rhs->set_frame(node["orientation-of"].as<giskard_core::FrameSpecPtr>());

      return true;
    }
  };

  inline bool is_rotation_reference(const Node& node)
  {
    if(!node.IsScalar())
      return false;

    try
    {
      node.as<std::string>();
      return true;
    }
    catch (const YAML::Exception& e)
    {
      return false;
    }
  }

  template<>
  struct convert<giskard_core::RotationReferenceSpecPtr> 
  {
    
    static Node encode(const giskard_core::RotationReferenceSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_reference_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::RotationReferenceSpecPtr& rhs) 
    {
      if(!is_rotation_reference(node))
        return false;
 
      rhs = giskard_core::RotationReferenceSpecPtr(new giskard_core::RotationReferenceSpec());
      rhs->set_reference_name(node.as<std::string>());

      return true;
    }
  };

  inline bool is_inverse_rotation(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["inverse-rotation"];
  }

  template<>
  struct convert<giskard_core::InverseRotationSpecPtr> 
  {
    
    static Node encode(const giskard_core::InverseRotationSpecPtr& rhs) 
    {
      Node node;
      node["inverse-rotation"] = rhs->get_rotation();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::InverseRotationSpecPtr& rhs) 
    {
      if(!is_inverse_rotation(node))
        return false;

      rhs = giskard_core::InverseRotationSpecPtr(new giskard_core::InverseRotationSpec());
      rhs->set_rotation(node["inverse-rotation"].as<giskard_core::RotationSpecPtr>());

      return true;
    }
  };

  inline bool is_rotation_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["rotation-mul"] &&
        node["rotation-mul"].IsSequence();
  }

  template<>
  struct convert<giskard_core::RotationMultiplicationSpecPtr> 
  {
    static Node encode(const giskard_core::RotationMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["rotation-mul"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::RotationMultiplicationSpecPtr& rhs) 
    {
      if(!is_rotation_multiplication(node))
        return false;

      rhs = giskard_core::RotationMultiplicationSpecPtr(new giskard_core::RotationMultiplicationSpec()); 
      rhs->set_inputs(node["rotation-mul"].as< std::vector<giskard_core::RotationSpecPtr> >());

      return true;
    }
  };

  template<>
  struct convert<giskard_core::RotationSpecPtr> 
  {
    static Node encode(const giskard_core::RotationSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard_core::RotationInputSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::RotationInputSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::AxisAngleSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::AxisAngleSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::RotationQuaternionConstructorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::RotationQuaternionConstructorSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::OrientationOfSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::OrientationOfSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::RotationReferenceSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::RotationReferenceSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::InverseRotationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::InverseRotationSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::RotationMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::RotationMultiplicationSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::SlerpSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::SlerpSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard_core::RotationSpecPtr& rhs) 
    {
      if(is_input_rotation(node)) {
        rhs = node.as<giskard_core::RotationInputSpecPtr>();
        return true;
      }
      else if(is_axis_angle(node))
      {
        rhs = node.as<giskard_core::AxisAngleSpecPtr>();
        return true;
      } 
      else if(is_slerp(node))
      {
        rhs = node.as<giskard_core::SlerpSpecPtr>();
        return true;
      } 
      else if(is_quaternion_constructor(node))
      {
        rhs = node.as<giskard_core::RotationQuaternionConstructorSpecPtr>();
        return true;
      }
      else if(is_rotation_reference(node))
      {
        rhs = node.as<giskard_core::RotationReferenceSpecPtr>();
        return true;
      }
      else if(is_orientation_of(node))
      {
        rhs = node.as<giskard_core::OrientationOfSpecPtr>();
        return true;
      }
      else if(is_inverse_rotation(node))
      {
        rhs = node.as<giskard_core::InverseRotationSpecPtr>();
        return true;
      }
      else if(is_rotation_multiplication(node))
      {
        rhs = node.as<giskard_core::RotationMultiplicationSpecPtr>();
        return true;
      }
      else
        return false;
    }
  }; 

  ///
  /// parsing frame specifications
  ///


  inline bool is_input_frame(const Node& node)
  {
    if (node.IsMap() && (node.size() == 1) && node["input-frame"]) {
      try
      {
        node["input-frame"].as<std::string>();
        return true;
      }
      catch (const YAML::Exception& e)
      { }
    }
    return false;
  }

  template<>
  struct convert<giskard_core::FrameInputSpecPtr> 
  {
    
    static Node encode(const giskard_core::FrameInputSpecPtr& rhs) 
    {
      Node node;
      node["input-frame"] = rhs->get_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::FrameInputSpecPtr& rhs) 
    {
      if(!is_input_frame(node))
        return false;
  
      rhs = giskard_core::FrameInputSpecPtr(new giskard_core::FrameInputSpec(node["input-frame"].as<std::string>()));

      return true;
    }
  };

  inline bool is_cached_frame(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["cached-frame"];
  }

  template<>
  struct convert<giskard_core::FrameCachedSpecPtr> 
  {
    static Node encode(const giskard_core::FrameCachedSpecPtr& rhs) 
    {
      Node node;
      node["cached-frame"] = rhs->get_frame();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::FrameCachedSpecPtr& rhs) 
    {
      if(!is_cached_frame(node))
        return false;

      rhs = giskard_core::FrameCachedSpecPtr(new giskard_core::FrameCachedSpec()); 
      rhs->set_frame(node["cached-frame"].as<giskard_core::FrameSpecPtr>());

      return true;
    }
  };

  inline bool is_constructor_frame(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["frame"] &&
        node["frame"].IsSequence() && (node["frame"].size() == 2);
  }

  template<>
  struct convert<giskard_core::FrameConstructorSpecPtr> 
  {
    static Node encode(const giskard_core::FrameConstructorSpecPtr& rhs) 
    {
      Node node;
      node["frame"][0] = rhs->get_rotation();
      node["frame"][1] = rhs->get_translation();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::FrameConstructorSpecPtr& rhs) 
    {
      if(!is_constructor_frame(node))
        return false;

      rhs = giskard_core::FrameConstructorSpecPtr(new giskard_core::FrameConstructorSpec()); 
      rhs->set_rotation(node["frame"][0].as<giskard_core::RotationSpecPtr>());
      rhs->set_translation(node["frame"][1].as<giskard_core::VectorSpecPtr>());

      return true;
    }
  };

  inline bool is_frame_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["frame-mul"] &&
        node["frame-mul"].IsSequence();
  }

  template<>
  struct convert<giskard_core::FrameMultiplicationSpecPtr> 
  {
    static Node encode(const giskard_core::FrameMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["frame-mul"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::FrameMultiplicationSpecPtr& rhs) 
    {
      if(!is_frame_multiplication(node))
        return false;

      rhs = giskard_core::FrameMultiplicationSpecPtr(new giskard_core::FrameMultiplicationSpec()); 
      rhs->set_inputs(node["frame-mul"].as< std::vector<giskard_core::FrameSpecPtr> >());

      return true;
    }
  };

  inline bool is_frame_reference(const Node& node)
  {
    if(!node.IsScalar())
      return false;

    try
    {
      node.as<std::string>();
      return true;
    }
    catch (const YAML::Exception& e)
    {
      return false;
    }
  }

  template<>
  struct convert<giskard_core::FrameReferenceSpecPtr> 
  {
    
    static Node encode(const giskard_core::FrameReferenceSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_reference_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::FrameReferenceSpecPtr& rhs) 
    {
      if(!is_frame_reference(node))
        return false;
  
      rhs = giskard_core::FrameReferenceSpecPtr(new giskard_core::FrameReferenceSpec());
      rhs->set_reference_name(node.as<std::string>());

      return true;
    }
  };

  inline bool is_inverse_frame(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["inverse-frame"];
  }

  template<>
  struct convert<giskard_core::InverseFrameSpecPtr> 
  {
    
    static Node encode(const giskard_core::InverseFrameSpecPtr& rhs) 
    {
      Node node;
      node["inverse-frame"] = rhs->get_frame();
      return node;
    }
  
    static bool decode(const Node& node, giskard_core::InverseFrameSpecPtr& rhs) 
    {
      if(!is_inverse_frame(node))
        return false;

      rhs = giskard_core::InverseFrameSpecPtr(new giskard_core::InverseFrameSpec());
      rhs->set_frame(node["inverse-frame"].as<giskard_core::FrameSpecPtr>());

      return true;
    }
  };

  template<>
  struct convert<giskard_core::FrameSpecPtr> 
  {
    static Node encode(const giskard_core::FrameSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard_core::FrameInputSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::FrameInputSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::FrameCachedSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::FrameCachedSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard_core::FrameConstructorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::FrameConstructorSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard_core::FrameMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::FrameMultiplicationSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard_core::FrameReferenceSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::FrameReferenceSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard_core::InverseFrameSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::InverseFrameSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard_core::FrameSpecPtr& rhs) 
    {
      if(is_input_frame(node))
      {
        rhs = node.as<giskard_core::FrameInputSpecPtr>();
        return true;
      }
      else if(is_cached_frame(node))
      {
        rhs = node.as<giskard_core::FrameCachedSpecPtr>();
        return true;
      }
      else if(is_constructor_frame(node))
      {
        rhs = node.as<giskard_core::FrameConstructorSpecPtr>();
        return true;
      }
      else if(is_frame_multiplication(node))
      {
        rhs = node.as<giskard_core::FrameMultiplicationSpecPtr>();
        return true;
      }
      else if(is_frame_reference(node))
      {
        rhs = node.as<giskard_core::FrameReferenceSpecPtr>();
        return true;
      }
      else if(is_inverse_frame(node))
      {
        rhs = node.as<giskard_core::InverseFrameSpecPtr>();
        return true;
      }
      else
        return false;
    }
  };

  ///
  /// parsing of general specifications
  ///

  inline bool is_double_spec(const Node& node)
  {
    return is_const_double(node) || is_input_scalar(node) || is_input_joint(node) || is_double_reference(node) ||
        is_double_norm_of(node) || is_double_multiplication(node) || is_double_division(node) ||
        is_double_addition(node) || is_double_subtraction(node) ||
        is_x_coord_of(node) || is_y_coord_of(node) || is_z_coord_of(node) ||
        is_vector_dot(node) || is_min(node) || is_double_if(node) || is_abs(node) ||
        is_fmod(node);
  }

  inline bool is_vector_spec(const Node& node)
  {
    return is_cached_vector(node) || is_input_vec3(node) || is_constructor_vector(node) || is_vector_reference(node) ||
        is_vector_origin_of(node) || is_vector_addition(node) ||
        is_vector_subtraction(node) ||
        is_vector_frame_multiplication(node) || is_vector_double_multiplication(node) ||
        is_vector_rotation_vector(node) || is_vector_rotation_multiplication(node);
  }

  inline bool is_rotation_spec(const Node& node)
  {
    return is_quaternion_constructor(node) || is_input_rotation(node) || is_axis_angle(node) || 
      is_rotation_reference(node) || is_orientation_of(node) ||
      is_inverse_rotation(node) || is_rotation_multiplication(node) ||
      is_slerp(node);
  }

  inline bool is_frame_spec(const Node& node)
  {
    return is_cached_frame(node) || is_input_frame(node) || is_constructor_frame(node) || 
        is_frame_multiplication(node) || is_frame_reference(node) ||
        is_inverse_frame(node);
  }

  template<>
  struct convert<giskard_core::SpecPtr> 
  {
    static Node encode(const giskard_core::SpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard_core::FrameSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::FrameSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard_core::VectorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::VectorSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard_core::RotationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::RotationSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard_core::DoubleSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard_core::DoubleSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard_core::SpecPtr& rhs) 
    {
      if(is_double_spec(node))
      {
        rhs = node.as<giskard_core::DoubleSpecPtr>();
        return true;
      }
      else if(is_vector_spec(node))
      {
        rhs = node.as<giskard_core::VectorSpecPtr>();
        return true;
      }
      else if(is_rotation_spec(node))
      {
        rhs = node.as<giskard_core::RotationSpecPtr>();
        return true;
      }
      else if(is_frame_spec(node))
      {
        rhs = node.as<giskard_core::FrameSpecPtr>();
        return true;
      }
      else
        return false;
    }
  };

  ///
  /// Parsing of more composed structures
  ///

  inline bool is_scope_entry(const Node& node)
  {
    return node.IsMap() && (node.size() == 1);
  }

  template<>
  struct convert<giskard_core::ScopeEntry> 
  {
    static Node encode(const giskard_core::ScopeEntry& rhs) 
    {
      YAML::Node node;

      node[rhs.name] = rhs.spec;

      return node;
    }
  
    static bool decode(const Node& node, giskard_core::ScopeEntry& rhs) 
    {
      if(!is_scope_entry(node))
        return false;

      rhs.name = node.begin()->first.as<std::string>();
      rhs.spec = node.begin()->second.as<giskard_core::SpecPtr>(); 

      return true;
    }
  };

  inline bool is_controllable_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["controllable-constraint"] &&
        node["controllable-constraint"].IsSequence() && (node["controllable-constraint"].size() == 4);
  }

  template<>
  struct convert<giskard_core::ControllableConstraintSpec> 
  {
    static Node encode(const giskard_core::ControllableConstraintSpec& rhs) 
    {
      YAML::Node node;

      node["controllable-constraint"][0] = rhs.lower_;
      node["controllable-constraint"][1] = rhs.upper_;
      node["controllable-constraint"][2] = rhs.weight_;
      node["controllable-constraint"][3] = rhs.input_;

      return node;
    }
  
    static bool decode(const Node& node, giskard_core::ControllableConstraintSpec& rhs) 
    {
      if(!is_controllable_spec(node))
        return false;

      rhs.lower_ = node["controllable-constraint"][0].as<giskard_core::DoubleSpecPtr>();
      rhs.upper_ = node["controllable-constraint"][1].as<giskard_core::DoubleSpecPtr>();
      rhs.weight_ = node["controllable-constraint"][2].as<giskard_core::DoubleSpecPtr>();
      rhs.input_ = node["controllable-constraint"][3].as<std::string>();

      return true;
    }
  };

  inline bool is_soft_constraint_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["soft-constraint"] &&
        node["soft-constraint"].IsSequence() && (node["soft-constraint"].size() == 5);
  }

  template<>
  struct convert<giskard_core::SoftConstraintSpec> 
  {
    static Node encode(const giskard_core::SoftConstraintSpec& rhs) 
    {
      YAML::Node node;

      node["soft-constraint"][0] = rhs.lower_;
      node["soft-constraint"][1] = rhs.upper_;
      node["soft-constraint"][2] = rhs.weight_;
      node["soft-constraint"][3] = rhs.expression_;
      node["soft-constraint"][4] = rhs.name_;

      return node;
    }
  
    static bool decode(const Node& node, giskard_core::SoftConstraintSpec& rhs) 
    {
      if(!is_soft_constraint_spec(node))
        return false;

      rhs.lower_ = node["soft-constraint"][0].as<giskard_core::DoubleSpecPtr>();
      rhs.upper_ = node["soft-constraint"][1].as<giskard_core::DoubleSpecPtr>();
      rhs.weight_ = node["soft-constraint"][2].as<giskard_core::DoubleSpecPtr>();
      rhs.expression_ = node["soft-constraint"][3].as<giskard_core::DoubleSpecPtr>();
      rhs.name_ = node["soft-constraint"][4].as<std::string>();

      return true;
    }
  };

  inline bool is_hard_constraint_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["hard-constraint"] &&
        node["hard-constraint"].IsSequence() && (node["hard-constraint"].size() == 3);
  }

  template<>
  struct convert<giskard_core::HardConstraintSpec> 
  {
    static Node encode(const giskard_core::HardConstraintSpec& rhs) 
    {
      YAML::Node node;

      node["hard-constraint"][0] = rhs.lower_;
      node["hard-constraint"][1] = rhs.upper_;
      node["hard-constraint"][2] = rhs.expression_;

      return node;
    }
  
    static bool decode(const Node& node, giskard_core::HardConstraintSpec& rhs) 
    {
      if(!is_hard_constraint_spec(node))
        return false;

      rhs.lower_ = node["hard-constraint"][0].as<giskard_core::DoubleSpecPtr>();
      rhs.upper_ = node["hard-constraint"][1].as<giskard_core::DoubleSpecPtr>();
      rhs.expression_ = node["hard-constraint"][2].as<giskard_core::DoubleSpecPtr>();

      return true;
    }
  };

  inline bool is_qp_controller_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 4) && node["scope"] &&
        node["scope"].IsSequence() && node["soft-constraints"] &&
        node["soft-constraints"].IsSequence() && node["hard-constraints"] &&
        node["hard-constraints"].IsSequence() && node["controllable-constraints"] &&
        node["controllable-constraints"].IsSequence();
  }

  template<>
  struct convert<giskard_core::QPControllerSpec> 
  {
    static Node encode(const giskard_core::QPControllerSpec& rhs) 
    {
      YAML::Node node;

      node["scope"] = rhs.scope_;
      node["controllable-constraints"] = rhs.controllable_constraints_;
      node["soft-constraints"] = rhs.soft_constraints_;
      node["hard-constraints"] = rhs.hard_constraints_;

      return node;
    }
  
    static bool decode(const Node& node, giskard_core::QPControllerSpec& rhs) 
    {
      if(!is_qp_controller_spec(node))
        return false;

      rhs.scope_ = node["scope"].as< std::vector<giskard_core::ScopeEntry> >();
      rhs.controllable_constraints_ = 
          node["controllable-constraints"].as< std::vector<giskard_core::ControllableConstraintSpec> >();
      rhs.soft_constraints_ = 
          node["soft-constraints"].as< std::vector<giskard_core::SoftConstraintSpec> >();
      rhs.hard_constraints_ = 
          node["hard-constraints"].as< std::vector<giskard_core::HardConstraintSpec> >();

      return true;
    }
  };

}

#endif // GISKARD_CORE_YAML_PARSER_HPP
