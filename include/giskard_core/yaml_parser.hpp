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
  struct convert<giskard::core::DoubleConstSpecPtr> 
  {
    
    static Node encode(const giskard::core::DoubleConstSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_value();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::DoubleConstSpecPtr& rhs) 
    {
      if(!is_const_double(node))
        return false;
  
      rhs = giskard::core::DoubleConstSpecPtr(new giskard::core::DoubleConstSpec());
      rhs->set_value(node.as<double>());

      return true;
    }
  };

  inline bool is_input(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["input-var"] && 
        node["input-var"].IsScalar();
  }

  template<>
  struct convert<giskard::core::DoubleInputSpecPtr> 
  {
    
    static Node encode(const giskard::core::DoubleInputSpecPtr& rhs) 
    {
      Node node;
      node["input-var"] = rhs->get_input_num();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::DoubleInputSpecPtr& rhs) 
    {
      if(!is_input(node))
        return false;
  
      rhs = giskard::core::DoubleInputSpecPtr(new giskard::core::DoubleInputSpec());
      rhs->set_input_num(node["input-var"].as<size_t>());

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
  struct convert<giskard::core::DoubleReferenceSpecPtr> 
  {
    
    static Node encode(const giskard::core::DoubleReferenceSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_reference_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::DoubleReferenceSpecPtr& rhs) 
    {
      if(!is_double_reference(node))
        return false;
 
      rhs = giskard::core::DoubleReferenceSpecPtr(new giskard::core::DoubleReferenceSpec());
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
  struct convert<giskard::core::DoubleAdditionSpecPtr> 
  {
    static Node encode(const giskard::core::DoubleAdditionSpecPtr& rhs) 
    {
      Node node;
      node["double-add"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::DoubleAdditionSpecPtr& rhs) 
    {
      if(!is_double_addition(node))
        return false;

      rhs = giskard::core::DoubleAdditionSpecPtr(new giskard::core::DoubleAdditionSpec()); 
      rhs->set_inputs(node["double-add"].as< std::vector<giskard::core::DoubleSpecPtr> >());

      return true;
    }
  };

  inline bool is_double_subtraction(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["double-sub"] &&
        node["double-sub"].IsSequence();
  }

  template<>
  struct convert<giskard::core::DoubleSubtractionSpecPtr> 
  {
    static Node encode(const giskard::core::DoubleSubtractionSpecPtr& rhs) 
    {
      Node node;
      node["double-sub"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::DoubleSubtractionSpecPtr& rhs) 
    {
      if(!is_double_subtraction(node))
        return false;

      rhs = giskard::core::DoubleSubtractionSpecPtr(new giskard::core::DoubleSubtractionSpec()); 
      rhs->set_inputs(node["double-sub"].as< std::vector<giskard::core::DoubleSpecPtr> >());

      return true;
    }
  };

  inline bool is_double_norm_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["vector-norm"];
  }

  template<>
  struct convert<giskard::core::DoubleNormOfSpecPtr> 
  {
    
    static Node encode(const giskard::core::DoubleNormOfSpecPtr& rhs) 
    {
      Node node;
      node["vector-norm"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::DoubleNormOfSpecPtr& rhs) 
    {
      if(!is_double_norm_of(node))
        return false;
  
      rhs = giskard::core::DoubleNormOfSpecPtr(new giskard::core::DoubleNormOfSpec());
      rhs->set_vector(node["vector-norm"].as<giskard::core::VectorSpecPtr>());

      return true;
    }
  };

  inline bool is_double_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["double-mul"] &&
        node["double-mul"].IsSequence();
  }

  template<>
  struct convert<giskard::core::DoubleMultiplicationSpecPtr> 
  {
    static Node encode(const giskard::core::DoubleMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["double-mul"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::DoubleMultiplicationSpecPtr& rhs) 
    {
      if(!is_double_multiplication(node))
        return false;

      rhs = giskard::core::DoubleMultiplicationSpecPtr(new giskard::core::DoubleMultiplicationSpec()); 
      rhs->set_inputs(node["double-mul"].as< std::vector<giskard::core::DoubleSpecPtr> >());

      return true;
    }
  };

  inline bool is_double_division(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["double-div"] &&
        node["double-div"].IsSequence();
  }

  template<>
  struct convert<giskard::core::DoubleDivisionSpecPtr> 
  {
    static Node encode(const giskard::core::DoubleDivisionSpecPtr& rhs) 
    {
      Node node;
      node["double-div"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::DoubleDivisionSpecPtr& rhs) 
    {
      if(!is_double_division(node))
        return false;

      rhs = giskard::core::DoubleDivisionSpecPtr(new giskard::core::DoubleDivisionSpec()); 
      rhs->set_inputs(node["double-div"].as< std::vector<giskard::core::DoubleSpecPtr> >());

      return true;
    }
  };

  inline bool is_x_coord_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["x-coord"];
  }

  template<>
  struct convert<giskard::core::DoubleXCoordOfSpecPtr> 
  {
    static Node encode(const giskard::core::DoubleXCoordOfSpecPtr& rhs) 
    {
      Node node;
      node["x-coord"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::DoubleXCoordOfSpecPtr& rhs) 
    {
      if(!is_x_coord_of(node))
        return false;

      rhs = giskard::core::DoubleXCoordOfSpecPtr(new giskard::core::DoubleXCoordOfSpec()); 
      rhs->set_vector(node["x-coord"].as< giskard::core::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_y_coord_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["y-coord"];
  }

  template<>
  struct convert<giskard::core::DoubleYCoordOfSpecPtr> 
  {
    static Node encode(const giskard::core::DoubleYCoordOfSpecPtr& rhs) 
    {
      Node node;
      node["y-coord"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::DoubleYCoordOfSpecPtr& rhs) 
    {
      if(!is_y_coord_of(node))
        return false;

      rhs = giskard::core::DoubleYCoordOfSpecPtr(new giskard::core::DoubleYCoordOfSpec()); 
      rhs->set_vector(node["y-coord"].as< giskard::core::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_z_coord_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["z-coord"];
  }

  template<>
  struct convert<giskard::core::DoubleZCoordOfSpecPtr> 
  {
    static Node encode(const giskard::core::DoubleZCoordOfSpecPtr& rhs) 
    {
      Node node;
      node["z-coord"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::DoubleZCoordOfSpecPtr& rhs) 
    {
      if(!is_z_coord_of(node))
        return false;

      rhs = giskard::core::DoubleZCoordOfSpecPtr(new giskard::core::DoubleZCoordOfSpec()); 
      rhs->set_vector(node["z-coord"].as< giskard::core::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_vector_dot(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["vector-dot"] &&
        node["vector-dot"].IsSequence() && (node["vector-dot"].size() == 2);
  }

  template<>
  struct convert<giskard::core::VectorDotSpecPtr>
  {
    static Node encode(const giskard::core::VectorDotSpecPtr& rhs) 
    {
      Node node;
      node["vector-dot"][0] = rhs->get_lhs();
      node["vector-dot"][1] = rhs->get_rhs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::VectorDotSpecPtr& rhs) 
    {
      if(!is_vector_dot(node))
        return false;

      rhs = giskard::core::VectorDotSpecPtr(new giskard::core::VectorDotSpec()); 
      rhs->set_lhs(node["vector-dot"][0].as< giskard::core::VectorSpecPtr >());
      rhs->set_rhs(node["vector-dot"][1].as< giskard::core::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_abs(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["abs"];
  }

  template<>
  struct convert<giskard::core::AbsSpecPtr>
  {
    static Node encode(const giskard::core::AbsSpecPtr& rhs) 
    {
      Node node;
      node["abs"] = rhs->get_value();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::AbsSpecPtr& rhs) 
    {
      if(!is_abs(node))
        return false;

      rhs = giskard::core::AbsSpecPtr(new giskard::core::AbsSpec()); 
      rhs->set_value(node["abs"].as< giskard::core::DoubleSpecPtr >());

      return true;
    }
  };

  inline bool is_fmod(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["fmod"] &&
      node["fmod"].IsSequence() && (node["fmod"].size() == 2);
  }

  template<>
  struct convert<giskard::core::FmodSpecPtr>
  {
    static Node encode(const giskard::core::FmodSpecPtr& rhs) 
    {
      Node node;
      node["fmod"][0] = rhs->get_nominator();
      node["fmod"][1] = rhs->get_denominator();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::FmodSpecPtr& rhs) 
    {
      if(!is_fmod(node))
        return false;

      rhs = giskard::core::FmodSpecPtr(new giskard::core::FmodSpec()); 
      rhs->set_nominator(node["fmod"][0].as< giskard::core::DoubleSpecPtr >());
      rhs->set_denominator(node["fmod"][1].as< giskard::core::DoubleSpecPtr >());

      return true;
    }
  };

  inline bool is_min(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["min"] &&
        node["min"].IsSequence() && (node["min"].size() == 2);
  }

  template<>
  struct convert<giskard::core::MinSpecPtr>
  {
    static Node encode(const giskard::core::MinSpecPtr& rhs) 
    {
      Node node;
      node["min"][0] = rhs->get_lhs();
      node["min"][1] = rhs->get_rhs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::MinSpecPtr& rhs) 
    {
      if(!is_min(node))
        return false;

      rhs = giskard::core::MinSpecPtr(new giskard::core::MinSpec()); 
      rhs->set_lhs(node["min"][0].as< giskard::core::DoubleSpecPtr >());
      rhs->set_rhs(node["min"][1].as< giskard::core::DoubleSpecPtr >());

      return true;
    }
  };

  inline bool is_double_if(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["double-if"] &&
        node["double-if"].IsSequence() && (node["double-if"].size() == 3);
  }

  template<>
  struct convert<giskard::core::DoubleIfSpecPtr>
  {
    static Node encode(const giskard::core::DoubleIfSpecPtr& rhs) 
    {
      Node node;
      node["double-if"][0] = rhs->get_condition();
      node["double-if"][1] = rhs->get_if();
      node["double-if"][2] = rhs->get_else();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::DoubleIfSpecPtr& rhs) 
    {
      if(!is_double_if(node))
        return false;

      rhs = giskard::core::DoubleIfSpecPtr(new giskard::core::DoubleIfSpec()); 
      rhs->set_condition(node["double-if"][0].as< giskard::core::DoubleSpecPtr >());
      rhs->set_if(node["double-if"][1].as< giskard::core::DoubleSpecPtr >());
      rhs->set_else(node["double-if"][2].as< giskard::core::DoubleSpecPtr >());

      return true;
    }
  };

  template<>
  struct convert<giskard::core::DoubleSpecPtr> 
  {
    
    static Node encode(const giskard::core::DoubleSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard::core::DoubleConstSpec>(rhs).get())
      {
        giskard::core::DoubleConstSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::DoubleConstSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::core::DoubleInputSpec>(rhs).get())
      {
        giskard::core::DoubleInputSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::DoubleInputSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::core::DoubleReferenceSpec>(rhs).get())
      {
        giskard::core::DoubleReferenceSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::DoubleReferenceSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::core::DoubleAdditionSpec>(rhs).get())
      {
        giskard::core::DoubleAdditionSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::DoubleAdditionSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::core::DoubleSubtractionSpec>(rhs).get())
      {
        giskard::core::DoubleSubtractionSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::DoubleSubtractionSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::core::DoubleNormOfSpec>(rhs).get())
      {
        giskard::core::DoubleNormOfSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::DoubleNormOfSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::core::DoubleMultiplicationSpec>(rhs).get())
      {
        giskard::core::DoubleMultiplicationSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::DoubleMultiplicationSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::core::DoubleDivisionSpec>(rhs).get())
      {
        giskard::core::DoubleDivisionSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::DoubleDivisionSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::core::DoubleXCoordOfSpec>(rhs).get())
      {
        giskard::core::DoubleXCoordOfSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::DoubleXCoordOfSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::core::DoubleYCoordOfSpec>(rhs).get())
      {
        giskard::core::DoubleYCoordOfSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::DoubleYCoordOfSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::core::DoubleZCoordOfSpec>(rhs).get())
      {
        giskard::core::DoubleZCoordOfSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::DoubleZCoordOfSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::core::VectorDotSpec>(rhs).get())
      {
        giskard::core::VectorDotSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::VectorDotSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::core::MinSpec>(rhs).get())
      {
        giskard::core::MinSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::MinSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::core::AbsSpec>(rhs).get())
      {
        giskard::core::AbsSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::AbsSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::core::FmodSpec>(rhs).get())
      {
        giskard::core::FmodSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::FmodSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::core::DoubleIfSpec>(rhs).get())
      {
        giskard::core::DoubleIfSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::core::DoubleIfSpec>(rhs);
        node = p;
      }

      return node;
    }
  
    static bool decode(const Node& node, giskard::core::DoubleSpecPtr& rhs) 
    {

      if(is_const_double(node))
      {
        rhs = node.as<giskard::core::DoubleConstSpecPtr>();
        return true;
      }
      else if(is_input(node))
      {
        rhs = node.as<giskard::core::DoubleInputSpecPtr>();
        return true;
      }
      else if(is_double_addition(node))
      {
        rhs = node.as<giskard::core::DoubleAdditionSpecPtr>();
        return true;
      }
      else if(is_double_subtraction(node))
      {
        rhs = node.as<giskard::core::DoubleSubtractionSpecPtr>();
        return true;
      }
      else if(is_double_multiplication(node))
      {
        rhs = node.as<giskard::core::DoubleMultiplicationSpecPtr>();
        return true;
      }
      else if(is_double_division(node))
      {
        rhs = node.as<giskard::core::DoubleDivisionSpecPtr>();
        return true;
      }
      else if(is_double_norm_of(node))
      {
        rhs = node.as<giskard::core::DoubleNormOfSpecPtr>();
        return true;
      }
      else if(is_x_coord_of(node))
      {
        rhs = node.as<giskard::core::DoubleXCoordOfSpecPtr>();
        return true;
      }
      else if(is_y_coord_of(node))
      {
        rhs = node.as<giskard::core::DoubleYCoordOfSpecPtr>();
        return true;
      }
      else if(is_z_coord_of(node))
      {
        rhs = node.as<giskard::core::DoubleZCoordOfSpecPtr>();
        return true;
      }
      else if(is_double_reference(node))
      {
        rhs = node.as<giskard::core::DoubleReferenceSpecPtr>();
        return true;
      }
      else if(is_vector_dot(node))
      {
        rhs = node.as<giskard::core::VectorDotSpecPtr>();
        return true;
      }
      else if(is_min(node))
      {
        rhs = node.as<giskard::core::MinSpecPtr>();
        return true;
      }
      else if(is_abs(node))
      {
        rhs = node.as<giskard::core::AbsSpecPtr>();
        return true;
      }
      else if(is_fmod(node))
      {
        rhs = node.as<giskard::core::FmodSpecPtr>();
        return true;
      }
      else if(is_double_if(node))
      {
        rhs = node.as<giskard::core::DoubleIfSpecPtr>();
        return true;
      }
      else
        return false;
    }
  };
 
  //
  // parsing of vector specs
  //

  inline bool is_cached_vector(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["cached-vector"];
  }

  template<>
  struct convert<giskard::core::VectorCachedSpecPtr> 
  {
    static Node encode(const giskard::core::VectorCachedSpecPtr& rhs) 
    {
      Node node;
      node["cached-vector"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::VectorCachedSpecPtr& rhs) 
    {
      if(!is_cached_vector(node))
        return false;

      rhs = giskard::core::VectorCachedSpecPtr(new giskard::core::VectorCachedSpec()); 
      rhs->set_vector(node["cached-vector"].as<giskard::core::VectorSpecPtr>());

      return true;
    }
  };

  inline bool is_constructor_vector(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["vector3"] &&
        node["vector3"].IsSequence() && (node["vector3"].size() == 3);
  }

  template<>
  struct convert<giskard::core::VectorConstructorSpecPtr> 
  {
    static Node encode(const giskard::core::VectorConstructorSpecPtr& rhs) 
    {
      Node node;
      node["vector3"].push_back(rhs->get_x());
      node["vector3"].push_back(rhs->get_y());
      node["vector3"].push_back(rhs->get_z());
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::VectorConstructorSpecPtr& rhs) 
    {
      if(!is_constructor_vector(node))
        return false;

      rhs = giskard::core::VectorConstructorSpecPtr(new giskard::core::VectorConstructorSpec()); 
      rhs->set_x(node["vector3"][0].as<giskard::core::DoubleSpecPtr>());
      rhs->set_y(node["vector3"][1].as<giskard::core::DoubleSpecPtr>());
      rhs->set_z(node["vector3"][2].as<giskard::core::DoubleSpecPtr>());

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
  struct convert<giskard::core::VectorReferenceSpecPtr> 
  {
    
    static Node encode(const giskard::core::VectorReferenceSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_reference_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::VectorReferenceSpecPtr& rhs) 
    {
      if(!is_vector_reference(node))
        return false;
  
      rhs = giskard::core::VectorReferenceSpecPtr(new giskard::core::VectorReferenceSpec());
      rhs->set_reference_name(node.as<std::string>());

      return true;
    }
  };

  inline bool is_vector_origin_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["origin-of"];
  }

  template<>
  struct convert<giskard::core::VectorOriginOfSpecPtr> 
  {
    
    static Node encode(const giskard::core::VectorOriginOfSpecPtr& rhs) 
    {
      Node node;
      node["origin-of"] = rhs->get_frame();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::VectorOriginOfSpecPtr& rhs) 
    {
      if(!is_vector_origin_of(node))
        return false;
  
      rhs = giskard::core::VectorOriginOfSpecPtr(new giskard::core::VectorOriginOfSpec());
      rhs->set_frame(node["origin-of"].as<giskard::core::FrameSpecPtr>());

      return true;
    }
  };

  inline bool is_vector_addition(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["vector-add"] &&
        node["vector-add"].IsSequence();
  }

  template<>
  struct convert<giskard::core::VectorAdditionSpecPtr> 
  {
    static Node encode(const giskard::core::VectorAdditionSpecPtr& rhs) 
    {
      Node node;
      node["vector-add"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::VectorAdditionSpecPtr& rhs) 
    {
      if(!is_vector_addition(node))
        return false;

      rhs = giskard::core::VectorAdditionSpecPtr(new giskard::core::VectorAdditionSpec()); 
      rhs->set_inputs(node["vector-add"].as< std::vector<giskard::core::VectorSpecPtr> >());

      return true;
    }
  };

  inline bool is_vector_subtraction(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["vector-sub"] &&
        node["vector-sub"].IsSequence();
  }

  template<>
  struct convert<giskard::core::VectorSubtractionSpecPtr> 
  {
    static Node encode(const giskard::core::VectorSubtractionSpecPtr& rhs) 
    {
      Node node;
      node["vector-sub"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::VectorSubtractionSpecPtr& rhs) 
    {
      if(!is_vector_subtraction(node))
        return false;

      rhs = giskard::core::VectorSubtractionSpecPtr(new giskard::core::VectorSubtractionSpec()); 
      rhs->set_inputs(node["vector-sub"].as< std::vector<giskard::core::VectorSpecPtr> >());

      return true;
    }
  };

  inline bool is_vector_rotation_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["rotate-vector"] &&
        node["rotate-vector"].IsSequence() && (node["rotate-vector"].size() == 2);
  }

  template<>
  struct convert<giskard::core::VectorRotationMultiplicationSpecPtr> 
  {
    static Node encode(const giskard::core::VectorRotationMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["rotate-vector"][0] = rhs->get_rotation();
      node["rotate-vector"][1] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::VectorRotationMultiplicationSpecPtr& rhs) 
    {
      if(!is_vector_rotation_multiplication(node))
        return false;

      rhs = giskard::core::VectorRotationMultiplicationSpecPtr(new giskard::core::VectorRotationMultiplicationSpec()); 
      rhs->set_rotation(node["rotate-vector"][0].as< giskard::core::RotationSpecPtr >());
      rhs->set_vector(node["rotate-vector"][1].as< giskard::core::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_vector_frame_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["transform-vector"] &&
        node["transform-vector"].IsSequence() && (node["transform-vector"].size() == 2);
  }

  template<>
  struct convert<giskard::core::VectorFrameMultiplicationSpecPtr> 
  {
    static Node encode(const giskard::core::VectorFrameMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["transform-vector"][0] = rhs->get_frame();
      node["transform-vector"][1] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::VectorFrameMultiplicationSpecPtr& rhs) 
    {
      if(!is_vector_frame_multiplication(node))
        return false;

      rhs = giskard::core::VectorFrameMultiplicationSpecPtr(new giskard::core::VectorFrameMultiplicationSpec()); 
      rhs->set_frame(node["transform-vector"][0].as< giskard::core::FrameSpecPtr >());
      rhs->set_vector(node["transform-vector"][1].as< giskard::core::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_vector_double_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["scale-vector"] &&
        node["scale-vector"].IsSequence() && (node["scale-vector"].size() == 2);
  }

  template<>
  struct convert<giskard::core::VectorDoubleMultiplicationSpecPtr> 
  {
    static Node encode(const giskard::core::VectorDoubleMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["scale-vector"][0] = rhs->get_double();
      node["scale-vector"][1] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::VectorDoubleMultiplicationSpecPtr& rhs) 
    {
      if(!is_vector_double_multiplication(node))
        return false;

      rhs = giskard::core::VectorDoubleMultiplicationSpecPtr(new giskard::core::VectorDoubleMultiplicationSpec()); 
      rhs->set_double(node["scale-vector"][0].as< giskard::core::DoubleSpecPtr >());
      rhs->set_vector(node["scale-vector"][1].as< giskard::core::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_vector_rotation_vector(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["rot-vector"];
  }

  template<>
  struct convert<giskard::core::VectorRotationVectorSpecPtr> 
  {
    
    static Node encode(const giskard::core::VectorRotationVectorSpecPtr& rhs) 
    {
      Node node;
      node["rot-vector"] = rhs->get_rotation();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::VectorRotationVectorSpecPtr& rhs) 
    {
      if(!is_vector_rotation_vector(node))
        return false;
  
      rhs = giskard::core::VectorRotationVectorSpecPtr(new giskard::core::VectorRotationVectorSpec());
      rhs->set_rotation(node["rot-vector"].as<giskard::core::RotationSpecPtr>());

      return true;
    }
  };

  template<>
  struct convert<giskard::core::VectorSpecPtr> 
  {
    static Node encode(const giskard::core::VectorSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard::core::VectorCachedSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::VectorCachedSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::VectorConstructorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::VectorConstructorSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::VectorReferenceSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::VectorReferenceSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::VectorOriginOfSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::VectorOriginOfSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::VectorAdditionSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::VectorAdditionSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::VectorSubtractionSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::VectorSubtractionSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::VectorFrameMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::VectorFrameMultiplicationSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::VectorRotationMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::VectorRotationMultiplicationSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::VectorDoubleMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::VectorDoubleMultiplicationSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::VectorRotationVectorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::VectorRotationVectorSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard::core::VectorSpecPtr& rhs) 
    {
      if(is_cached_vector(node))
      {
        rhs = node.as<giskard::core::VectorCachedSpecPtr>();
        return true;
      }
      else if(is_constructor_vector(node))
      {
        rhs = node.as<giskard::core::VectorConstructorSpecPtr>();
        return true;
      }
      else if(is_vector_reference(node))
      {
        rhs = node.as<giskard::core::VectorReferenceSpecPtr>();
        return true;
      }
      else if(is_vector_origin_of(node))
      {
        rhs = node.as<giskard::core::VectorOriginOfSpecPtr>();
        return true;
      }
      else if(is_vector_addition(node))
      {
        rhs = node.as<giskard::core::VectorAdditionSpecPtr>();
        return true;
      }
      else if(is_vector_subtraction(node))
      {
        rhs = node.as<giskard::core::VectorSubtractionSpecPtr>();
        return true;
      }
      else if(is_vector_frame_multiplication(node))
      {
        rhs = node.as<giskard::core::VectorFrameMultiplicationSpecPtr>();
        return true;
      }
      else if(is_vector_rotation_multiplication(node))
      {
        rhs = node.as<giskard::core::VectorRotationMultiplicationSpecPtr>();
        return true;
      }
      else if(is_vector_double_multiplication(node))
      {
        rhs = node.as<giskard::core::VectorDoubleMultiplicationSpecPtr>();
        return true;
      }
      else if(is_vector_rotation_vector(node))
      {
        rhs = node.as<giskard::core::VectorRotationVectorSpecPtr>();
        return true;
      }
      else
        return false;
    }
  };

  ///
  /// parsing rotation specs
  ///

  inline bool is_quaternion_constructor(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["quaternion"] &&
        node["quaternion"].IsSequence() && (node["quaternion"].size() == 4);
  }

  template<>
  struct convert<giskard::core::RotationQuaternionConstructorSpecPtr> 
  {
    static Node encode(const giskard::core::RotationQuaternionConstructorSpecPtr& rhs) 
    {
      Node node;
      node["quaternion"][0] = rhs->get_x();
      node["quaternion"][1] = rhs->get_y();
      node["quaternion"][2] = rhs->get_z();
      node["quaternion"][3] = rhs->get_w();

      return node;
    }
  
    static bool decode(const Node& node, giskard::core::RotationQuaternionConstructorSpecPtr& rhs) 
    {
      if(!is_quaternion_constructor(node))
        return false;

      rhs = giskard::core::RotationQuaternionConstructorSpecPtr(new giskard::core::RotationQuaternionConstructorSpec());
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
  struct convert<giskard::core::SlerpSpecPtr> 
  {
    static Node encode(const giskard::core::SlerpSpecPtr& rhs) 
    {
      Node node;
      node["slerp"][0] = rhs->get_from();
      node["slerp"][1] = rhs->get_to();
      node["slerp"][2] = rhs->get_param();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::SlerpSpecPtr& rhs) 
    {
      if(!is_slerp(node))
        return false;

      rhs = giskard::core::SlerpSpecPtr(new giskard::core::SlerpSpec()); 
      rhs->set_from(node["slerp"][0].as<giskard::core::RotationSpecPtr>());
      rhs->set_to(node["slerp"][1].as<giskard::core::RotationSpecPtr>());
      rhs->set_param(node["slerp"][2].as<giskard::core::DoubleSpecPtr>());

      return true;
    }
  };

  inline bool is_axis_angle(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["axis-angle"] &&
        node["axis-angle"].IsSequence() && (node["axis-angle"].size() == 2);
  }

  template<>
  struct convert<giskard::core::AxisAngleSpecPtr> 
  {
    static Node encode(const giskard::core::AxisAngleSpecPtr& rhs) 
    {
      Node node;
      node["axis-angle"][0] = rhs->get_axis();
      node["axis-angle"][1] = rhs->get_angle();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::AxisAngleSpecPtr& rhs) 
    {
      if(!is_axis_angle(node))
        return false;

      rhs = giskard::core::AxisAngleSpecPtr(new giskard::core::AxisAngleSpec()); 
      rhs->set_axis(node["axis-angle"][0].as<giskard::core::VectorSpecPtr>());
      rhs->set_angle(node["axis-angle"][1].as<giskard::core::DoubleSpecPtr>());

      return true;
    }
  };

  inline bool is_orientation_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["orientation-of"];
  }

  template<>
  struct convert<giskard::core::OrientationOfSpecPtr> 
  {
    
    static Node encode(const giskard::core::OrientationOfSpecPtr& rhs) 
    {
      Node node;
      node["orientation-of"] = rhs->get_frame();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::OrientationOfSpecPtr& rhs) 
    {
      if(!is_orientation_of(node))
        return false;
  
      rhs = giskard::core::OrientationOfSpecPtr(new giskard::core::OrientationOfSpec());
      rhs->set_frame(node["orientation-of"].as<giskard::core::FrameSpecPtr>());

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
  struct convert<giskard::core::RotationReferenceSpecPtr> 
  {
    
    static Node encode(const giskard::core::RotationReferenceSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_reference_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::RotationReferenceSpecPtr& rhs) 
    {
      if(!is_rotation_reference(node))
        return false;
 
      rhs = giskard::core::RotationReferenceSpecPtr(new giskard::core::RotationReferenceSpec());
      rhs->set_reference_name(node.as<std::string>());

      return true;
    }
  };

  inline bool is_inverse_rotation(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["inverse-rotation"];
  }

  template<>
  struct convert<giskard::core::InverseRotationSpecPtr> 
  {
    
    static Node encode(const giskard::core::InverseRotationSpecPtr& rhs) 
    {
      Node node;
      node["inverse-rotation"] = rhs->get_rotation();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::InverseRotationSpecPtr& rhs) 
    {
      if(!is_inverse_rotation(node))
        return false;

      rhs = giskard::core::InverseRotationSpecPtr(new giskard::core::InverseRotationSpec());
      rhs->set_rotation(node["inverse-rotation"].as<giskard::core::RotationSpecPtr>());

      return true;
    }
  };

  inline bool is_rotation_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["rotation-mul"] &&
        node["rotation-mul"].IsSequence();
  }

  template<>
  struct convert<giskard::core::RotationMultiplicationSpecPtr> 
  {
    static Node encode(const giskard::core::RotationMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["rotation-mul"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::RotationMultiplicationSpecPtr& rhs) 
    {
      if(!is_rotation_multiplication(node))
        return false;

      rhs = giskard::core::RotationMultiplicationSpecPtr(new giskard::core::RotationMultiplicationSpec()); 
      rhs->set_inputs(node["rotation-mul"].as< std::vector<giskard::core::RotationSpecPtr> >());

      return true;
    }
  };

  template<>
  struct convert<giskard::core::RotationSpecPtr> 
  {
    static Node encode(const giskard::core::RotationSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard::core::AxisAngleSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::AxisAngleSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::RotationQuaternionConstructorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::RotationQuaternionConstructorSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::OrientationOfSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::OrientationOfSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::RotationReferenceSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::RotationReferenceSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::InverseRotationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::InverseRotationSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::RotationMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::RotationMultiplicationSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::SlerpSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::SlerpSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard::core::RotationSpecPtr& rhs) 
    {
      if(is_axis_angle(node))
      {
        rhs = node.as<giskard::core::AxisAngleSpecPtr>();
        return true;
      } 
      else if(is_slerp(node))
      {
        rhs = node.as<giskard::core::SlerpSpecPtr>();
        return true;
      } 
      else if(is_quaternion_constructor(node))
      {
        rhs = node.as<giskard::core::RotationQuaternionConstructorSpecPtr>();
        return true;
      }
      else if(is_rotation_reference(node))
      {
        rhs = node.as<giskard::core::RotationReferenceSpecPtr>();
        return true;
      }
      else if(is_orientation_of(node))
      {
        rhs = node.as<giskard::core::OrientationOfSpecPtr>();
        return true;
      }
      else if(is_inverse_rotation(node))
      {
        rhs = node.as<giskard::core::InverseRotationSpecPtr>();
        return true;
      }
      else if(is_rotation_multiplication(node))
      {
        rhs = node.as<giskard::core::RotationMultiplicationSpecPtr>();
        return true;
      }
      else
        return false;
    }
  }; 

  ///
  /// parsing frame specifications
  ///

  inline bool is_cached_frame(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["cached-frame"];
  }

  template<>
  struct convert<giskard::core::FrameCachedSpecPtr> 
  {
    static Node encode(const giskard::core::FrameCachedSpecPtr& rhs) 
    {
      Node node;
      node["cached-frame"] = rhs->get_frame();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::FrameCachedSpecPtr& rhs) 
    {
      if(!is_cached_frame(node))
        return false;

      rhs = giskard::core::FrameCachedSpecPtr(new giskard::core::FrameCachedSpec()); 
      rhs->set_frame(node["cached-frame"].as<giskard::core::FrameSpecPtr>());

      return true;
    }
  };

  inline bool is_constructor_frame(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["frame"] &&
        node["frame"].IsSequence() && (node["frame"].size() == 2);
  }

  template<>
  struct convert<giskard::core::FrameConstructorSpecPtr> 
  {
    static Node encode(const giskard::core::FrameConstructorSpecPtr& rhs) 
    {
      Node node;
      node["frame"][0] = rhs->get_rotation();
      node["frame"][1] = rhs->get_translation();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::FrameConstructorSpecPtr& rhs) 
    {
      if(!is_constructor_frame(node))
        return false;

      rhs = giskard::core::FrameConstructorSpecPtr(new giskard::core::FrameConstructorSpec()); 
      rhs->set_rotation(node["frame"][0].as<giskard::core::RotationSpecPtr>());
      rhs->set_translation(node["frame"][1].as<giskard::core::VectorSpecPtr>());

      return true;
    }
  };

  inline bool is_frame_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["frame-mul"] &&
        node["frame-mul"].IsSequence();
  }

  template<>
  struct convert<giskard::core::FrameMultiplicationSpecPtr> 
  {
    static Node encode(const giskard::core::FrameMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["frame-mul"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::FrameMultiplicationSpecPtr& rhs) 
    {
      if(!is_frame_multiplication(node))
        return false;

      rhs = giskard::core::FrameMultiplicationSpecPtr(new giskard::core::FrameMultiplicationSpec()); 
      rhs->set_inputs(node["frame-mul"].as< std::vector<giskard::core::FrameSpecPtr> >());

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
  struct convert<giskard::core::FrameReferenceSpecPtr> 
  {
    
    static Node encode(const giskard::core::FrameReferenceSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_reference_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard::core::FrameReferenceSpecPtr& rhs) 
    {
      if(!is_frame_reference(node))
        return false;
  
      rhs = giskard::core::FrameReferenceSpecPtr(new giskard::core::FrameReferenceSpec());
      rhs->set_reference_name(node.as<std::string>());

      return true;
    }
  };

  template<>
  struct convert<giskard::core::FrameSpecPtr> 
  {
    static Node encode(const giskard::core::FrameSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard::core::FrameCachedSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::FrameCachedSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::core::FrameConstructorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::FrameConstructorSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard::core::FrameMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::FrameMultiplicationSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard::core::FrameReferenceSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::FrameReferenceSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard::core::FrameSpecPtr& rhs) 
    {
      if(is_cached_frame(node))
      {
        rhs = node.as<giskard::core::FrameCachedSpecPtr>();
        return true;
      }
      else if(is_constructor_frame(node))
      {
        rhs = node.as<giskard::core::FrameConstructorSpecPtr>();
        return true;
      }
      else if(is_frame_multiplication(node))
      {
        rhs = node.as<giskard::core::FrameMultiplicationSpecPtr>();
        return true;
      }
      else if(is_frame_reference(node))
      {
        rhs = node.as<giskard::core::FrameReferenceSpecPtr>();
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
    return is_const_double(node) || is_input(node) || is_double_reference(node) ||
        is_double_norm_of(node) || is_double_multiplication(node) || is_double_division(node) ||
        is_double_addition(node) || is_double_subtraction(node) ||
        is_x_coord_of(node) || is_y_coord_of(node) || is_z_coord_of(node) ||
        is_vector_dot(node) || is_min(node) || is_double_if(node) || is_abs(node) ||
        is_fmod(node);
  }

  inline bool is_vector_spec(const Node& node)
  {
    return is_cached_vector(node) || is_constructor_vector(node) || is_vector_reference(node) ||
        is_vector_origin_of(node) || is_vector_addition(node) ||
        is_vector_subtraction(node) ||
        is_vector_frame_multiplication(node) || is_vector_double_multiplication(node) ||
        is_vector_rotation_vector(node) || is_vector_rotation_multiplication(node);
  }

  inline bool is_rotation_spec(const Node& node)
  {
    return is_quaternion_constructor(node) || is_axis_angle(node) || 
      is_rotation_reference(node) || is_orientation_of(node) ||
      is_inverse_rotation(node) || is_rotation_multiplication(node) ||
      is_slerp(node);
  }

  inline bool is_frame_spec(const Node& node)
  {
    return is_cached_frame(node) || is_constructor_frame(node) || 
        is_frame_multiplication(node) || is_frame_reference(node);
  }

  template<>
  struct convert<giskard::core::SpecPtr> 
  {
    static Node encode(const giskard::core::SpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard::core::FrameSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::FrameSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard::core::VectorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::VectorSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard::core::RotationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::RotationSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard::core::DoubleSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::core::DoubleSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard::core::SpecPtr& rhs) 
    {
      if(is_double_spec(node))
      {
        rhs = node.as<giskard::core::DoubleSpecPtr>();
        return true;
      }
      else if(is_vector_spec(node))
      {
        rhs = node.as<giskard::core::VectorSpecPtr>();
        return true;
      }
      else if(is_rotation_spec(node))
      {
        rhs = node.as<giskard::core::RotationSpecPtr>();
        return true;
      }
      else if(is_frame_spec(node))
      {
        rhs = node.as<giskard::core::FrameSpecPtr>();
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
  struct convert<giskard::core::ScopeEntry> 
  {
    static Node encode(const giskard::core::ScopeEntry& rhs) 
    {
      YAML::Node node;

      node[rhs.name] = rhs.spec;

      return node;
    }
  
    static bool decode(const Node& node, giskard::core::ScopeEntry& rhs) 
    {
      if(!is_scope_entry(node))
        return false;

      rhs.name = node.begin()->first.as<std::string>();
      rhs.spec = node.begin()->second.as<giskard::core::SpecPtr>(); 

      return true;
    }
  };

  inline bool is_controllable_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["controllable-constraint"] &&
        node["controllable-constraint"].IsSequence() && (node["controllable-constraint"].size() == 5);
  }

  template<>
  struct convert<giskard::core::ControllableConstraintSpec> 
  {
    static Node encode(const giskard::core::ControllableConstraintSpec& rhs) 
    {
      YAML::Node node;

      node["controllable-constraint"][0] = rhs.lower_;
      node["controllable-constraint"][1] = rhs.upper_;
      node["controllable-constraint"][2] = rhs.weight_;
      node["controllable-constraint"][3] = rhs.input_number_;
      node["controllable-constraint"][4] = rhs.name_;

      return node;
    }
  
    static bool decode(const Node& node, giskard::core::ControllableConstraintSpec& rhs) 
    {
      if(!is_controllable_spec(node))
        return false;

      rhs.lower_ = node["controllable-constraint"][0].as<giskard::core::DoubleSpecPtr>();
      rhs.upper_ = node["controllable-constraint"][1].as<giskard::core::DoubleSpecPtr>();
      rhs.weight_ = node["controllable-constraint"][2].as<giskard::core::DoubleSpecPtr>();
      rhs.input_number_ = node["controllable-constraint"][3].as<size_t>();
      rhs.name_ = node["controllable-constraint"][4].as<std::string>();

      return true;
    }
  };

  inline bool is_soft_constraint_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["soft-constraint"] &&
        node["soft-constraint"].IsSequence() && (node["soft-constraint"].size() == 5);
  }

  template<>
  struct convert<giskard::core::SoftConstraintSpec> 
  {
    static Node encode(const giskard::core::SoftConstraintSpec& rhs) 
    {
      YAML::Node node;

      node["soft-constraint"][0] = rhs.lower_;
      node["soft-constraint"][1] = rhs.upper_;
      node["soft-constraint"][2] = rhs.weight_;
      node["soft-constraint"][3] = rhs.expression_;
      node["soft-constraint"][4] = rhs.name_;

      return node;
    }
  
    static bool decode(const Node& node, giskard::core::SoftConstraintSpec& rhs) 
    {
      if(!is_soft_constraint_spec(node))
        return false;

      rhs.lower_ = node["soft-constraint"][0].as<giskard::core::DoubleSpecPtr>();
      rhs.upper_ = node["soft-constraint"][1].as<giskard::core::DoubleSpecPtr>();
      rhs.weight_ = node["soft-constraint"][2].as<giskard::core::DoubleSpecPtr>();
      rhs.expression_ = node["soft-constraint"][3].as<giskard::core::DoubleSpecPtr>();
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
  struct convert<giskard::core::HardConstraintSpec> 
  {
    static Node encode(const giskard::core::HardConstraintSpec& rhs) 
    {
      YAML::Node node;

      node["hard-constraint"][0] = rhs.lower_;
      node["hard-constraint"][1] = rhs.upper_;
      node["hard-constraint"][2] = rhs.expression_;

      return node;
    }
  
    static bool decode(const Node& node, giskard::core::HardConstraintSpec& rhs) 
    {
      if(!is_hard_constraint_spec(node))
        return false;

      rhs.lower_ = node["hard-constraint"][0].as<giskard::core::DoubleSpecPtr>();
      rhs.upper_ = node["hard-constraint"][1].as<giskard::core::DoubleSpecPtr>();
      rhs.expression_ = node["hard-constraint"][2].as<giskard::core::DoubleSpecPtr>();

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
  struct convert<giskard::core::QPControllerSpec> 
  {
    static Node encode(const giskard::core::QPControllerSpec& rhs) 
    {
      YAML::Node node;

      node["scope"] = rhs.scope_;
      node["controllable-constraints"] = rhs.controllable_constraints_;
      node["soft-constraints"] = rhs.soft_constraints_;
      node["hard-constraints"] = rhs.hard_constraints_;

      return node;
    }
  
    static bool decode(const Node& node, giskard::core::QPControllerSpec& rhs) 
    {
      if(!is_qp_controller_spec(node))
        return false;

      rhs.scope_ = node["scope"].as< std::vector<giskard::core::ScopeEntry> >();
      rhs.controllable_constraints_ = 
          node["controllable-constraints"].as< std::vector<giskard::core::ControllableConstraintSpec> >();
      rhs.soft_constraints_ = 
          node["soft-constraints"].as< std::vector<giskard::core::SoftConstraintSpec> >();
      rhs.hard_constraints_ = 
          node["hard-constraints"].as< std::vector<giskard::core::HardConstraintSpec> >();

      return true;
    }
  };

}

#endif // GISKARD_CORE_YAML_PARSER_HPP
