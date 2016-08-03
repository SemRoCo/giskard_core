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

#ifndef GISKARD_YAML_PARSER_HPP
#define GISKARD_YAML_PARSER_HPP

#include <yaml-cpp/yaml.h>
#include <vector>
#include <giskard/specifications.hpp>

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
  struct convert<giskard::DoubleConstSpecPtr> 
  {
    
    static Node encode(const giskard::DoubleConstSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_value();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleConstSpecPtr& rhs) 
    {
      if(!is_const_double(node))
        return false;
  
      rhs = giskard::DoubleConstSpecPtr(new giskard::DoubleConstSpec());
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
  struct convert<giskard::DoubleInputSpecPtr> 
  {
    
    static Node encode(const giskard::DoubleInputSpecPtr& rhs) 
    {
      Node node;
      node["input-var"] = rhs->get_input_num();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleInputSpecPtr& rhs) 
    {
      if(!is_input(node))
        return false;
  
      rhs = giskard::DoubleInputSpecPtr(new giskard::DoubleInputSpec());
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
  struct convert<giskard::DoubleReferenceSpecPtr> 
  {
    
    static Node encode(const giskard::DoubleReferenceSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_reference_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleReferenceSpecPtr& rhs) 
    {
      if(!is_double_reference(node))
        return false;
 
      rhs = giskard::DoubleReferenceSpecPtr(new giskard::DoubleReferenceSpec());
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
  struct convert<giskard::DoubleAdditionSpecPtr> 
  {
    static Node encode(const giskard::DoubleAdditionSpecPtr& rhs) 
    {
      Node node;
      node["double-add"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleAdditionSpecPtr& rhs) 
    {
      if(!is_double_addition(node))
        return false;

      rhs = giskard::DoubleAdditionSpecPtr(new giskard::DoubleAdditionSpec()); 
      rhs->set_inputs(node["double-add"].as< std::vector<giskard::DoubleSpecPtr> >());

      return true;
    }
  };

  inline bool is_double_subtraction(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["double-sub"] &&
        node["double-sub"].IsSequence();
  }

  template<>
  struct convert<giskard::DoubleSubtractionSpecPtr> 
  {
    static Node encode(const giskard::DoubleSubtractionSpecPtr& rhs) 
    {
      Node node;
      node["double-sub"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleSubtractionSpecPtr& rhs) 
    {
      if(!is_double_subtraction(node))
        return false;

      rhs = giskard::DoubleSubtractionSpecPtr(new giskard::DoubleSubtractionSpec()); 
      rhs->set_inputs(node["double-sub"].as< std::vector<giskard::DoubleSpecPtr> >());

      return true;
    }
  };

  inline bool is_double_norm_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["vector-norm"];
  }

  template<>
  struct convert<giskard::DoubleNormOfSpecPtr> 
  {
    
    static Node encode(const giskard::DoubleNormOfSpecPtr& rhs) 
    {
      Node node;
      node["vector-norm"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleNormOfSpecPtr& rhs) 
    {
      if(!is_double_norm_of(node))
        return false;
  
      rhs = giskard::DoubleNormOfSpecPtr(new giskard::DoubleNormOfSpec());
      rhs->set_vector(node["vector-norm"].as<giskard::VectorSpecPtr>());

      return true;
    }
  };

  inline bool is_double_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["double-mul"] &&
        node["double-mul"].IsSequence();
  }

  template<>
  struct convert<giskard::DoubleMultiplicationSpecPtr> 
  {
    static Node encode(const giskard::DoubleMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["double-mul"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleMultiplicationSpecPtr& rhs) 
    {
      if(!is_double_multiplication(node))
        return false;

      rhs = giskard::DoubleMultiplicationSpecPtr(new giskard::DoubleMultiplicationSpec()); 
      rhs->set_inputs(node["double-mul"].as< std::vector<giskard::DoubleSpecPtr> >());

      return true;
    }
  };

  inline bool is_double_division(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["double-div"] &&
        node["double-div"].IsSequence();
  }

  template<>
  struct convert<giskard::DoubleDivisionSpecPtr> 
  {
    static Node encode(const giskard::DoubleDivisionSpecPtr& rhs) 
    {
      Node node;
      node["double-div"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleDivisionSpecPtr& rhs) 
    {
      if(!is_double_division(node))
        return false;

      rhs = giskard::DoubleDivisionSpecPtr(new giskard::DoubleDivisionSpec()); 
      rhs->set_inputs(node["double-div"].as< std::vector<giskard::DoubleSpecPtr> >());

      return true;
    }
  };

  inline bool is_x_coord_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["x-coord"];
  }

  template<>
  struct convert<giskard::DoubleXCoordOfSpecPtr> 
  {
    static Node encode(const giskard::DoubleXCoordOfSpecPtr& rhs) 
    {
      Node node;
      node["x-coord"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleXCoordOfSpecPtr& rhs) 
    {
      if(!is_x_coord_of(node))
        return false;

      rhs = giskard::DoubleXCoordOfSpecPtr(new giskard::DoubleXCoordOfSpec()); 
      rhs->set_vector(node["x-coord"].as< giskard::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_y_coord_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["y-coord"];
  }

  template<>
  struct convert<giskard::DoubleYCoordOfSpecPtr> 
  {
    static Node encode(const giskard::DoubleYCoordOfSpecPtr& rhs) 
    {
      Node node;
      node["y-coord"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleYCoordOfSpecPtr& rhs) 
    {
      if(!is_y_coord_of(node))
        return false;

      rhs = giskard::DoubleYCoordOfSpecPtr(new giskard::DoubleYCoordOfSpec()); 
      rhs->set_vector(node["y-coord"].as< giskard::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_z_coord_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["z-coord"];
  }

  template<>
  struct convert<giskard::DoubleZCoordOfSpecPtr> 
  {
    static Node encode(const giskard::DoubleZCoordOfSpecPtr& rhs) 
    {
      Node node;
      node["z-coord"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleZCoordOfSpecPtr& rhs) 
    {
      if(!is_z_coord_of(node))
        return false;

      rhs = giskard::DoubleZCoordOfSpecPtr(new giskard::DoubleZCoordOfSpec()); 
      rhs->set_vector(node["z-coord"].as< giskard::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_vector_dot(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["vector-dot"] &&
        node["vector-dot"].IsSequence() && (node["vector-dot"].size() == 2);
  }

  template<>
  struct convert<giskard::VectorDotSpecPtr>
  {
    static Node encode(const giskard::VectorDotSpecPtr& rhs) 
    {
      Node node;
      node["vector-dot"][0] = rhs->get_lhs();
      node["vector-dot"][1] = rhs->get_rhs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorDotSpecPtr& rhs) 
    {
      if(!is_vector_dot(node))
        return false;

      rhs = giskard::VectorDotSpecPtr(new giskard::VectorDotSpec()); 
      rhs->set_lhs(node["vector-dot"][0].as< giskard::VectorSpecPtr >());
      rhs->set_rhs(node["vector-dot"][1].as< giskard::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_min(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["min"] &&
        node["min"].IsSequence() && (node["min"].size() == 2);
  }

  template<>
  struct convert<giskard::MinSpecPtr>
  {
    static Node encode(const giskard::MinSpecPtr& rhs) 
    {
      Node node;
      node["min"][0] = rhs->get_lhs();
      node["min"][1] = rhs->get_rhs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::MinSpecPtr& rhs) 
    {
      if(!is_min(node))
        return false;

      rhs = giskard::MinSpecPtr(new giskard::MinSpec()); 
      rhs->set_lhs(node["min"][0].as< giskard::DoubleSpecPtr >());
      rhs->set_rhs(node["min"][1].as< giskard::DoubleSpecPtr >());

      return true;
    }
  };

  template<>
  struct convert<giskard::DoubleSpecPtr> 
  {
    
    static Node encode(const giskard::DoubleSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rhs).get())
      {
        giskard::DoubleConstSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::DoubleConstSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::DoubleInputSpec>(rhs).get())
      {
        giskard::DoubleInputSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::DoubleInputSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::DoubleReferenceSpec>(rhs).get())
      {
        giskard::DoubleReferenceSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::DoubleReferenceSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::DoubleAdditionSpec>(rhs).get())
      {
        giskard::DoubleAdditionSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::DoubleAdditionSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::DoubleSubtractionSpec>(rhs).get())
      {
        giskard::DoubleSubtractionSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::DoubleSubtractionSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::DoubleNormOfSpec>(rhs).get())
      {
        giskard::DoubleNormOfSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::DoubleNormOfSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::DoubleMultiplicationSpec>(rhs).get())
      {
        giskard::DoubleMultiplicationSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::DoubleMultiplicationSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::DoubleDivisionSpec>(rhs).get())
      {
        giskard::DoubleDivisionSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::DoubleDivisionSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::DoubleXCoordOfSpec>(rhs).get())
      {
        giskard::DoubleXCoordOfSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::DoubleXCoordOfSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::DoubleYCoordOfSpec>(rhs).get())
      {
        giskard::DoubleYCoordOfSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::DoubleYCoordOfSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::DoubleZCoordOfSpec>(rhs).get())
      {
        giskard::DoubleZCoordOfSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::DoubleZCoordOfSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::VectorDotSpec>(rhs).get())
      {
        giskard::VectorDotSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::VectorDotSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::MinSpec>(rhs).get())
      {
        giskard::MinSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::MinSpec>(rhs);
        node = p;
      }

      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleSpecPtr& rhs) 
    {

      if(is_const_double(node))
      {
        rhs = node.as<giskard::DoubleConstSpecPtr>();
        return true;
      }
      else if(is_input(node))
      {
        rhs = node.as<giskard::DoubleInputSpecPtr>();
        return true;
      }
      else if(is_double_addition(node))
      {
        rhs = node.as<giskard::DoubleAdditionSpecPtr>();
        return true;
      }
      else if(is_double_subtraction(node))
      {
        rhs = node.as<giskard::DoubleSubtractionSpecPtr>();
        return true;
      }
      else if(is_double_multiplication(node))
      {
        rhs = node.as<giskard::DoubleMultiplicationSpecPtr>();
        return true;
      }
      else if(is_double_division(node))
      {
        rhs = node.as<giskard::DoubleDivisionSpecPtr>();
        return true;
      }
      else if(is_double_norm_of(node))
      {
        rhs = node.as<giskard::DoubleNormOfSpecPtr>();
        return true;
      }
      else if(is_x_coord_of(node))
      {
        rhs = node.as<giskard::DoubleXCoordOfSpecPtr>();
        return true;
      }
      else if(is_y_coord_of(node))
      {
        rhs = node.as<giskard::DoubleYCoordOfSpecPtr>();
        return true;
      }
      else if(is_z_coord_of(node))
      {
        rhs = node.as<giskard::DoubleZCoordOfSpecPtr>();
        return true;
      }
      else if(is_double_reference(node))
      {
        rhs = node.as<giskard::DoubleReferenceSpecPtr>();
        return true;
      }
      else if(is_vector_dot(node))
      {
        rhs = node.as<giskard::VectorDotSpecPtr>();
        return true;
      }
      else if(is_min(node))
      {
        rhs = node.as<giskard::MinSpecPtr>();
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
  struct convert<giskard::VectorCachedSpecPtr> 
  {
    static Node encode(const giskard::VectorCachedSpecPtr& rhs) 
    {
      Node node;
      node["cached-vector"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorCachedSpecPtr& rhs) 
    {
      if(!is_cached_vector(node))
        return false;

      rhs = giskard::VectorCachedSpecPtr(new giskard::VectorCachedSpec()); 
      rhs->set_vector(node["cached-vector"].as<giskard::VectorSpecPtr>());

      return true;
    }
  };

  inline bool is_constructor_vector(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["vector3"] &&
        node["vector3"].IsSequence() && (node["vector3"].size() == 3);
  }

  template<>
  struct convert<giskard::VectorConstructorSpecPtr> 
  {
    static Node encode(const giskard::VectorConstructorSpecPtr& rhs) 
    {
      Node node;
      node["vector3"].push_back(rhs->get_x());
      node["vector3"].push_back(rhs->get_y());
      node["vector3"].push_back(rhs->get_z());
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorConstructorSpecPtr& rhs) 
    {
      if(!is_constructor_vector(node))
        return false;

      rhs = giskard::VectorConstructorSpecPtr(new giskard::VectorConstructorSpec()); 
      rhs->set_x(node["vector3"][0].as<giskard::DoubleSpecPtr>());
      rhs->set_y(node["vector3"][1].as<giskard::DoubleSpecPtr>());
      rhs->set_z(node["vector3"][2].as<giskard::DoubleSpecPtr>());

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
  struct convert<giskard::VectorReferenceSpecPtr> 
  {
    
    static Node encode(const giskard::VectorReferenceSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_reference_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorReferenceSpecPtr& rhs) 
    {
      if(!is_vector_reference(node))
        return false;
  
      rhs = giskard::VectorReferenceSpecPtr(new giskard::VectorReferenceSpec());
      rhs->set_reference_name(node.as<std::string>());

      return true;
    }
  };

  inline bool is_vector_origin_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["origin-of"];
  }

  template<>
  struct convert<giskard::VectorOriginOfSpecPtr> 
  {
    
    static Node encode(const giskard::VectorOriginOfSpecPtr& rhs) 
    {
      Node node;
      node["origin-of"] = rhs->get_frame();
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorOriginOfSpecPtr& rhs) 
    {
      if(!is_vector_origin_of(node))
        return false;
  
      rhs = giskard::VectorOriginOfSpecPtr(new giskard::VectorOriginOfSpec());
      rhs->set_frame(node["origin-of"].as<giskard::FrameSpecPtr>());

      return true;
    }
  };

  inline bool is_vector_addition(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["vector-add"] &&
        node["vector-add"].IsSequence();
  }

  template<>
  struct convert<giskard::VectorAdditionSpecPtr> 
  {
    static Node encode(const giskard::VectorAdditionSpecPtr& rhs) 
    {
      Node node;
      node["vector-add"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorAdditionSpecPtr& rhs) 
    {
      if(!is_vector_addition(node))
        return false;

      rhs = giskard::VectorAdditionSpecPtr(new giskard::VectorAdditionSpec()); 
      rhs->set_inputs(node["vector-add"].as< std::vector<giskard::VectorSpecPtr> >());

      return true;
    }
  };

  inline bool is_vector_subtraction(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["vector-sub"] &&
        node["vector-sub"].IsSequence();
  }

  template<>
  struct convert<giskard::VectorSubtractionSpecPtr> 
  {
    static Node encode(const giskard::VectorSubtractionSpecPtr& rhs) 
    {
      Node node;
      node["vector-sub"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorSubtractionSpecPtr& rhs) 
    {
      if(!is_vector_subtraction(node))
        return false;

      rhs = giskard::VectorSubtractionSpecPtr(new giskard::VectorSubtractionSpec()); 
      rhs->set_inputs(node["vector-sub"].as< std::vector<giskard::VectorSpecPtr> >());

      return true;
    }
  };

  inline bool is_vector_frame_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["transform-vector"] &&
        node["transform-vector"].IsSequence() && (node["transform-vector"].size() == 2);
  }

  template<>
  struct convert<giskard::VectorFrameMultiplicationSpecPtr> 
  {
    static Node encode(const giskard::VectorFrameMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["transform-vector"][0] = rhs->get_frame();
      node["transform-vector"][1] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorFrameMultiplicationSpecPtr& rhs) 
    {
      if(!is_vector_frame_multiplication(node))
        return false;

      rhs = giskard::VectorFrameMultiplicationSpecPtr(new giskard::VectorFrameMultiplicationSpec()); 
      rhs->set_frame(node["transform-vector"][0].as< giskard::FrameSpecPtr >());
      rhs->set_vector(node["transform-vector"][1].as< giskard::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_vector_double_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["scale-vector"] &&
        node["scale-vector"].IsSequence() && (node["scale-vector"].size() == 2);
  }

  template<>
  struct convert<giskard::VectorDoubleMultiplicationSpecPtr> 
  {
    static Node encode(const giskard::VectorDoubleMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["scale-vector"][0] = rhs->get_double();
      node["scale-vector"][1] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorDoubleMultiplicationSpecPtr& rhs) 
    {
      if(!is_vector_double_multiplication(node))
        return false;

      rhs = giskard::VectorDoubleMultiplicationSpecPtr(new giskard::VectorDoubleMultiplicationSpec()); 
      rhs->set_double(node["scale-vector"][0].as< giskard::DoubleSpecPtr >());
      rhs->set_vector(node["scale-vector"][1].as< giskard::VectorSpecPtr >());

      return true;
    }
  };

  inline bool is_vector_rotation_vector(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["rot-vector"];
  }

  template<>
  struct convert<giskard::VectorRotationVectorSpecPtr> 
  {
    
    static Node encode(const giskard::VectorRotationVectorSpecPtr& rhs) 
    {
      Node node;
      node["rot-vector"] = rhs->get_rotation();
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorRotationVectorSpecPtr& rhs) 
    {
      if(!is_vector_rotation_vector(node))
        return false;
  
      rhs = giskard::VectorRotationVectorSpecPtr(new giskard::VectorRotationVectorSpec());
      rhs->set_rotation(node["rot-vector"].as<giskard::RotationSpecPtr>());

      return true;
    }
  };

  template<>
  struct convert<giskard::VectorSpecPtr> 
  {
    static Node encode(const giskard::VectorSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard::VectorCachedSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::VectorCachedSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::VectorReferenceSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::VectorReferenceSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::VectorOriginOfSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::VectorOriginOfSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::VectorAdditionSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::VectorAdditionSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::VectorSubtractionSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::VectorSubtractionSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::VectorFrameMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::VectorFrameMultiplicationSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::VectorDoubleMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::VectorDoubleMultiplicationSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::VectorRotationVectorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::VectorRotationVectorSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorSpecPtr& rhs) 
    {
      if(is_cached_vector(node))
      {
        rhs = node.as<giskard::VectorCachedSpecPtr>();
        return true;
      }
      else if(is_constructor_vector(node))
      {
        rhs = node.as<giskard::VectorConstructorSpecPtr>();
        return true;
      }
      else if(is_vector_reference(node))
      {
        rhs = node.as<giskard::VectorReferenceSpecPtr>();
        return true;
      }
      else if(is_vector_origin_of(node))
      {
        rhs = node.as<giskard::VectorOriginOfSpecPtr>();
        return true;
      }
      else if(is_vector_addition(node))
      {
        rhs = node.as<giskard::VectorAdditionSpecPtr>();
        return true;
      }
      else if(is_vector_subtraction(node))
      {
        rhs = node.as<giskard::VectorSubtractionSpecPtr>();
        return true;
      }
      else if(is_vector_frame_multiplication(node))
      {
        rhs = node.as<giskard::VectorFrameMultiplicationSpecPtr>();
        return true;
      }
      else if(is_vector_double_multiplication(node))
      {
        rhs = node.as<giskard::VectorDoubleMultiplicationSpecPtr>();
        return true;
      }
      else if(is_vector_rotation_vector(node))
      {
        rhs = node.as<giskard::VectorRotationVectorSpecPtr>();
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
  struct convert<giskard::RotationQuaternionConstructorSpecPtr> 
  {
    static Node encode(const giskard::RotationQuaternionConstructorSpecPtr& rhs) 
    {
      Node node;
      node["quaternion"][0] = rhs->get_x();
      node["quaternion"][1] = rhs->get_y();
      node["quaternion"][2] = rhs->get_z();
      node["quaternion"][3] = rhs->get_w();

      return node;
    }
  
    static bool decode(const Node& node, giskard::RotationQuaternionConstructorSpecPtr& rhs) 
    {
      if(!is_quaternion_constructor(node))
        return false;

      rhs = giskard::RotationQuaternionConstructorSpecPtr(new giskard::RotationQuaternionConstructorSpec());
      rhs->set_x(node["quaternion"][0].as<double>());
      rhs->set_y(node["quaternion"][1].as<double>());
      rhs->set_z(node["quaternion"][2].as<double>());
      rhs->set_w(node["quaternion"][3].as<double>());

      return true;
    }
  };

  inline bool is_axis_angle(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["axis-angle"] &&
        node["axis-angle"].IsSequence() && (node["axis-angle"].size() == 2);
  }

  template<>
  struct convert<giskard::AxisAngleSpecPtr> 
  {
    static Node encode(const giskard::AxisAngleSpecPtr& rhs) 
    {
      Node node;
      node["axis-angle"][0] = rhs->get_axis();
      node["axis-angle"][1] = rhs->get_angle();
      return node;
    }
  
    static bool decode(const Node& node, giskard::AxisAngleSpecPtr& rhs) 
    {
      if(!is_axis_angle(node))
        return false;

      rhs = giskard::AxisAngleSpecPtr(new giskard::AxisAngleSpec()); 
      rhs->set_axis(node["axis-angle"][0].as<giskard::VectorSpecPtr>());
      rhs->set_angle(node["axis-angle"][1].as<giskard::DoubleSpecPtr>());

      return true;
    }
  };

  inline bool is_orientation_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["orientation-of"];
  }

  template<>
  struct convert<giskard::OrientationOfSpecPtr> 
  {
    
    static Node encode(const giskard::OrientationOfSpecPtr& rhs) 
    {
      Node node;
      node["orientation-of"] = rhs->get_frame();
      return node;
    }
  
    static bool decode(const Node& node, giskard::OrientationOfSpecPtr& rhs) 
    {
      if(!is_orientation_of(node))
        return false;
  
      rhs = giskard::OrientationOfSpecPtr(new giskard::OrientationOfSpec());
      rhs->set_frame(node["orientation-of"].as<giskard::FrameSpecPtr>());

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
  struct convert<giskard::RotationReferenceSpecPtr> 
  {
    
    static Node encode(const giskard::RotationReferenceSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_reference_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard::RotationReferenceSpecPtr& rhs) 
    {
      if(!is_rotation_reference(node))
        return false;
 
      rhs = giskard::RotationReferenceSpecPtr(new giskard::RotationReferenceSpec());
      rhs->set_reference_name(node.as<std::string>());

      return true;
    }
  };

  inline bool is_inverse_rotation(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["inverse-rotation"];
  }

  template<>
  struct convert<giskard::InverseRotationSpecPtr> 
  {
    
    static Node encode(const giskard::InverseRotationSpecPtr& rhs) 
    {
      Node node;
      node["inverse-rotation"] = rhs->get_rotation();
      return node;
    }
  
    static bool decode(const Node& node, giskard::InverseRotationSpecPtr& rhs) 
    {
      if(!is_inverse_rotation(node))
        return false;

      rhs = giskard::InverseRotationSpecPtr(new giskard::InverseRotationSpec());
      rhs->set_rotation(node["inverse-rotation"].as<giskard::RotationSpecPtr>());

      return true;
    }
  };

  inline bool is_rotation_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["rotation-mul"] &&
        node["rotation-mul"].IsSequence();
  }

  template<>
  struct convert<giskard::RotationMultiplicationSpecPtr> 
  {
    static Node encode(const giskard::RotationMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["rotation-mul"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::RotationMultiplicationSpecPtr& rhs) 
    {
      if(!is_rotation_multiplication(node))
        return false;

      rhs = giskard::RotationMultiplicationSpecPtr(new giskard::RotationMultiplicationSpec()); 
      rhs->set_inputs(node["rotation-mul"].as< std::vector<giskard::RotationSpecPtr> >());

      return true;
    }
  };

  template<>
  struct convert<giskard::RotationSpecPtr> 
  {
    static Node encode(const giskard::RotationSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::RotationQuaternionConstructorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::RotationQuaternionConstructorSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::OrientationOfSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::OrientationOfSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::RotationReferenceSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::RotationReferenceSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::InverseRotationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::InverseRotationSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::RotationMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::RotationMultiplicationSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard::RotationSpecPtr& rhs) 
    {
      if(is_axis_angle(node))
      {
        rhs = node.as<giskard::AxisAngleSpecPtr>();
        return true;
      } 
      else if(is_quaternion_constructor(node))
      {
        rhs = node.as<giskard::RotationQuaternionConstructorSpecPtr>();
        return true;
      }
      else if(is_rotation_reference(node))
      {
        rhs = node.as<giskard::RotationReferenceSpecPtr>();
        return true;
      }
      else if(is_orientation_of(node))
      {
        rhs = node.as<giskard::OrientationOfSpecPtr>();
        return true;
      }
      else if(is_inverse_rotation(node))
      {
        rhs = node.as<giskard::InverseRotationSpecPtr>();
        return true;
      }
      else if(is_rotation_multiplication(node))
      {
        rhs = node.as<giskard::RotationMultiplicationSpecPtr>();
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
  struct convert<giskard::FrameCachedSpecPtr> 
  {
    static Node encode(const giskard::FrameCachedSpecPtr& rhs) 
    {
      Node node;
      node["cached-frame"] = rhs->get_frame();
      return node;
    }
  
    static bool decode(const Node& node, giskard::FrameCachedSpecPtr& rhs) 
    {
      if(!is_cached_frame(node))
        return false;

      rhs = giskard::FrameCachedSpecPtr(new giskard::FrameCachedSpec()); 
      rhs->set_frame(node["cached-frame"].as<giskard::FrameSpecPtr>());

      return true;
    }
  };

  inline bool is_constructor_frame(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["frame"] &&
        node["frame"].IsSequence() && (node["frame"].size() == 2);
  }

  template<>
  struct convert<giskard::FrameConstructorSpecPtr> 
  {
    static Node encode(const giskard::FrameConstructorSpecPtr& rhs) 
    {
      Node node;
      node["frame"][0] = rhs->get_rotation();
      node["frame"][1] = rhs->get_translation();
      return node;
    }
  
    static bool decode(const Node& node, giskard::FrameConstructorSpecPtr& rhs) 
    {
      if(!is_constructor_frame(node))
        return false;

      rhs = giskard::FrameConstructorSpecPtr(new giskard::FrameConstructorSpec()); 
      rhs->set_rotation(node["frame"][0].as<giskard::RotationSpecPtr>());
      rhs->set_translation(node["frame"][1].as<giskard::VectorSpecPtr>());

      return true;
    }
  };

  inline bool is_frame_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["frame-mul"] &&
        node["frame-mul"].IsSequence();
  }

  template<>
  struct convert<giskard::FrameMultiplicationSpecPtr> 
  {
    static Node encode(const giskard::FrameMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["frame-mul"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::FrameMultiplicationSpecPtr& rhs) 
    {
      if(!is_frame_multiplication(node))
        return false;

      rhs = giskard::FrameMultiplicationSpecPtr(new giskard::FrameMultiplicationSpec()); 
      rhs->set_inputs(node["frame-mul"].as< std::vector<giskard::FrameSpecPtr> >());

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
  struct convert<giskard::FrameReferenceSpecPtr> 
  {
    
    static Node encode(const giskard::FrameReferenceSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_reference_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard::FrameReferenceSpecPtr& rhs) 
    {
      if(!is_frame_reference(node))
        return false;
  
      rhs = giskard::FrameReferenceSpecPtr(new giskard::FrameReferenceSpec());
      rhs->set_reference_name(node.as<std::string>());

      return true;
    }
  };

  template<>
  struct convert<giskard::FrameSpecPtr> 
  {
    static Node encode(const giskard::FrameSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard::FrameCachedSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::FrameCachedSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::FrameConstructorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::FrameConstructorSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard::FrameMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::FrameMultiplicationSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard::FrameReferenceSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::FrameReferenceSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard::FrameSpecPtr& rhs) 
    {
      if(is_cached_frame(node))
      {
        rhs = node.as<giskard::FrameCachedSpecPtr>();
        return true;
      }
      else if(is_constructor_frame(node))
      {
        rhs = node.as<giskard::FrameConstructorSpecPtr>();
        return true;
      }
      else if(is_frame_multiplication(node))
      {
        rhs = node.as<giskard::FrameMultiplicationSpecPtr>();
        return true;
      }
      else if(is_frame_reference(node))
      {
        rhs = node.as<giskard::FrameReferenceSpecPtr>();
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
        is_vector_dot(node) || is_min(node);
  }

  inline bool is_vector_spec(const Node& node)
  {
    return is_cached_vector(node) || is_constructor_vector(node) || is_vector_reference(node) ||
        is_vector_origin_of(node) || is_vector_addition(node) ||
        is_vector_subtraction(node) ||
        is_vector_frame_multiplication(node) || is_vector_double_multiplication(node) ||
        is_vector_rotation_vector(node);
  }

  inline bool is_rotation_spec(const Node& node)
  {
    return is_quaternion_constructor(node) || is_axis_angle(node) || 
      is_rotation_reference(node) || is_orientation_of(node) ||
      is_inverse_rotation(node) || is_rotation_multiplication(node);
  }

  inline bool is_frame_spec(const Node& node)
  {
    return is_cached_frame(node) || is_constructor_frame(node) || 
        is_frame_multiplication(node) || is_frame_reference(node);
  }

  template<>
  struct convert<giskard::SpecPtr> 
  {
    static Node encode(const giskard::SpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard::FrameSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::FrameSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard::VectorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::VectorSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard::RotationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::RotationSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard::DoubleSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::DoubleSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard::SpecPtr& rhs) 
    {
      if(is_double_spec(node))
      {
        rhs = node.as<giskard::DoubleSpecPtr>();
        return true;
      }
      else if(is_vector_spec(node))
      {
        rhs = node.as<giskard::VectorSpecPtr>();
        return true;
      }
      else if(is_rotation_spec(node))
      {
        rhs = node.as<giskard::RotationSpecPtr>();
        return true;
      }
      else if(is_frame_spec(node))
      {
        rhs = node.as<giskard::FrameSpecPtr>();
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
  struct convert<giskard::ScopeEntry> 
  {
    static Node encode(const giskard::ScopeEntry& rhs) 
    {
      YAML::Node node;

      node[rhs.name] = rhs.spec;

      return node;
    }
  
    static bool decode(const Node& node, giskard::ScopeEntry& rhs) 
    {
      if(!is_scope_entry(node))
        return false;

      rhs.name = node.begin()->first.as<std::string>();
      rhs.spec = node.begin()->second.as<giskard::SpecPtr>(); 

      return true;
    }
  };

  inline bool is_controllable_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["controllable-constraint"] &&
        node["controllable-constraint"].IsSequence() && (node["controllable-constraint"].size() == 5);
  }

  template<>
  struct convert<giskard::ControllableConstraintSpec> 
  {
    static Node encode(const giskard::ControllableConstraintSpec& rhs) 
    {
      YAML::Node node;

      node["controllable-constraint"][0] = rhs.lower_;
      node["controllable-constraint"][1] = rhs.upper_;
      node["controllable-constraint"][2] = rhs.weight_;
      node["controllable-constraint"][3] = rhs.input_number_;
      node["controllable-constraint"][4] = rhs.name_;

      return node;
    }
  
    static bool decode(const Node& node, giskard::ControllableConstraintSpec& rhs) 
    {
      if(!is_controllable_spec(node))
        return false;

      rhs.lower_ = node["controllable-constraint"][0].as<giskard::DoubleSpecPtr>();
      rhs.upper_ = node["controllable-constraint"][1].as<giskard::DoubleSpecPtr>();
      rhs.weight_ = node["controllable-constraint"][2].as<giskard::DoubleSpecPtr>();
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
  struct convert<giskard::SoftConstraintSpec> 
  {
    static Node encode(const giskard::SoftConstraintSpec& rhs) 
    {
      YAML::Node node;

      node["soft-constraint"][0] = rhs.lower_;
      node["soft-constraint"][1] = rhs.upper_;
      node["soft-constraint"][2] = rhs.weight_;
      node["soft-constraint"][3] = rhs.expression_;
      node["soft-constraint"][4] = rhs.name_;

      return node;
    }
  
    static bool decode(const Node& node, giskard::SoftConstraintSpec& rhs) 
    {
      if(!is_soft_constraint_spec(node))
        return false;

      rhs.lower_ = node["soft-constraint"][0].as<giskard::DoubleSpecPtr>();
      rhs.upper_ = node["soft-constraint"][1].as<giskard::DoubleSpecPtr>();
      rhs.weight_ = node["soft-constraint"][2].as<giskard::DoubleSpecPtr>();
      rhs.expression_ = node["soft-constraint"][3].as<giskard::DoubleSpecPtr>();
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
  struct convert<giskard::HardConstraintSpec> 
  {
    static Node encode(const giskard::HardConstraintSpec& rhs) 
    {
      YAML::Node node;

      node["hard-constraint"][0] = rhs.lower_;
      node["hard-constraint"][1] = rhs.upper_;
      node["hard-constraint"][2] = rhs.expression_;

      return node;
    }
  
    static bool decode(const Node& node, giskard::HardConstraintSpec& rhs) 
    {
      if(!is_hard_constraint_spec(node))
        return false;

      rhs.lower_ = node["hard-constraint"][0].as<giskard::DoubleSpecPtr>();
      rhs.upper_ = node["hard-constraint"][1].as<giskard::DoubleSpecPtr>();
      rhs.expression_ = node["hard-constraint"][2].as<giskard::DoubleSpecPtr>();

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
  struct convert<giskard::QPControllerSpec> 
  {
    static Node encode(const giskard::QPControllerSpec& rhs) 
    {
      YAML::Node node;

      node["scope"] = rhs.scope_;
      node["controllable-constraints"] = rhs.controllable_constraints_;
      node["soft-constraints"] = rhs.soft_constraints_;
      node["hard-constraints"] = rhs.hard_constraints_;

      return node;
    }
  
    static bool decode(const Node& node, giskard::QPControllerSpec& rhs) 
    {
      if(!is_qp_controller_spec(node))
        return false;

      rhs.scope_ = node["scope"].as< std::vector<giskard::ScopeEntry> >();
      rhs.controllable_constraints_ = 
          node["controllable-constraints"].as< std::vector<giskard::ControllableConstraintSpec> >();
      rhs.soft_constraints_ = 
          node["soft-constraints"].as< std::vector<giskard::SoftConstraintSpec> >();
      rhs.hard_constraints_ = 
          node["hard-constraints"].as< std::vector<giskard::HardConstraintSpec> >();

      return true;
    }
  };

}

#endif // GISKARD_YAML_PARSER_HPP
