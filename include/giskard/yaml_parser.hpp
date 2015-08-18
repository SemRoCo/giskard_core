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
    return node.IsScalar();
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
    return node.IsMap() && (node.size() == 2) && node["type"] && node["input-number"] &&
        (node["type"].as<std::string>().compare("INPUT") == 0);
  }

  template<>
  struct convert<giskard::DoubleInputSpecPtr> 
  {
    
    static Node encode(const giskard::DoubleInputSpecPtr& rhs) 
    {
      Node node;
      node["input-number"] = rhs->get_input_num();
      node["type"] = "INPUT";
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleInputSpecPtr& rhs) 
    {
      if(!is_input(node))
        return false;
  
      rhs = giskard::DoubleInputSpecPtr(new giskard::DoubleInputSpec());
      rhs->set_input_num(node["input-number"].as<size_t>());

      return true;
    }
  };

  inline bool is_double_reference(const Node& node)
  {
    return node.IsMap() && (node.size() == 2) && node["type"] &&
        (node["type"].as<std::string>().compare("DOUBLE-REFERENCE") == 0) &&
        node["reference"];
  }

  template<>
  struct convert<giskard::DoubleReferenceSpecPtr> 
  {
    
    static Node encode(const giskard::DoubleReferenceSpecPtr& rhs) 
    {
      Node node;
      node["reference"] = rhs->get_reference_name();
      node["type"] = "DOUBLE-REFERENCE";
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleReferenceSpecPtr& rhs) 
    {
      if(!is_double_reference(node))
        return false;
  
      rhs = giskard::DoubleReferenceSpecPtr(new giskard::DoubleReferenceSpec());
      rhs->set_reference_name(node["reference"].as<std::string>());

      return true;
    }
  };

  inline bool is_double_addition(const Node& node)
  {
    return node.IsMap() && (node.size() == 2) && node["type"] &&
        (node["type"].as<std::string>().compare("DOUBLE-ADDITION") == 0) &&
        node["inputs"] && node["inputs"].IsSequence();
  }

  template<>
  struct convert<giskard::DoubleAdditionSpecPtr> 
  {
    static Node encode(const giskard::DoubleAdditionSpecPtr& rhs) 
    {
      Node node;
      node["type"] = "DOUBLE-ADDITION";
      node["inputs"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleAdditionSpecPtr& rhs) 
    {
      if(!is_double_addition(node))
        return false;

      rhs = giskard::DoubleAdditionSpecPtr(new giskard::DoubleAdditionSpec()); 
      rhs->set_inputs(node["inputs"].as< std::vector<giskard::DoubleSpecPtr> >());

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
      else if(is_double_reference(node))
      {
        rhs = node.as<giskard::DoubleReferenceSpecPtr>();
        return true;
      }
      else if(is_double_addition(node))
      {
        rhs = node.as<giskard::DoubleAdditionSpecPtr>();
        return true;
      }
      else
        return false;
    }
  };
 
  //
  // parsing of vector specs
  //

  inline bool is_constructor_vector(const Node& node)
  {
    return node.IsMap() && (node.size() == 2) && node["type"] &&
        (node["type"].as<std::string>().compare("VECTOR3") == 0) && 
        node["inputs"] && node["inputs"].IsSequence() && (node["inputs"].size() == 3);
  }

  template<>
  struct convert<giskard::VectorConstructorSpecPtr> 
  {
    static Node encode(const giskard::VectorConstructorSpecPtr& rhs) 
    {
      Node node;
      node["type"] = "VECTOR3";
      node["inputs"].push_back(rhs->get_x());
      node["inputs"].push_back(rhs->get_y());
      node["inputs"].push_back(rhs->get_z());
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorConstructorSpecPtr& rhs) 
    {
      if(!is_constructor_vector(node))
        return false;

      rhs = giskard::VectorConstructorSpecPtr(new giskard::VectorConstructorSpec()); 
      rhs->set_x(node["inputs"][0].as<giskard::DoubleSpecPtr>());
      rhs->set_y(node["inputs"][1].as<giskard::DoubleSpecPtr>());
      rhs->set_z(node["inputs"][2].as<giskard::DoubleSpecPtr>());

      return true;
    }
  };

  inline bool is_vector_reference(const Node& node)
  {
    return node.IsMap() && (node.size() == 2) && node["type"] &&
        (node["type"].as<std::string>().compare("VECTOR-REFERENCE") == 0) &&
        node["reference"];
  }

  template<>
  struct convert<giskard::VectorReferenceSpecPtr> 
  {
    
    static Node encode(const giskard::VectorReferenceSpecPtr& rhs) 
    {
      Node node;
      node["reference"] = rhs->get_reference_name();
      node["type"] = "VECTOR-REFERENCE";
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorReferenceSpecPtr& rhs) 
    {
      if(!is_vector_reference(node))
        return false;
  
      rhs = giskard::VectorReferenceSpecPtr(new giskard::VectorReferenceSpec());
      rhs->set_reference_name(node["reference"].as<std::string>());

      return true;
    }
  };

  inline bool is_vector_origin_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 2) && node["type"] &&
        (node["type"].as<std::string>().compare("ORIGIN-OF") == 0) &&
        node["frame"];
  }

  template<>
  struct convert<giskard::VectorOriginOfSpecPtr> 
  {
    
    static Node encode(const giskard::VectorOriginOfSpecPtr& rhs) 
    {
      Node node;
      node["type"] = "ORIGIN-OF";
      node["frame"] = rhs->get_frame();
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorOriginOfSpecPtr& rhs) 
    {
      if(!is_vector_origin_of(node))
        return false;
  
      rhs = giskard::VectorOriginOfSpecPtr(new giskard::VectorOriginOfSpec());
      rhs->set_frame(node["frame"].as<giskard::FrameSpecPtr>());

      return true;
    }
  };

  template<>
  struct convert<giskard::VectorSpecPtr> 
  {
    static Node encode(const giskard::VectorSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::VectorConstructorSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::VectorReferenceSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::VectorReferenceSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::VectorOriginOfSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::VectorOriginOfSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorSpecPtr& rhs) 
    {
      if(is_constructor_vector(node))
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
      else
        return false;
    }
  };

  ///
  /// parsing rotation specs
  ///

  inline bool is_axis_angle(const Node& node)
  {
    return node.IsMap() && (node.size() == 3) && node["type"] &&
        (node["type"].as<std::string>().compare("ROTATION") == 0) &&
        node["axis"] && node["angle"];
  }

  template<>
  struct convert<giskard::AxisAngleSpecPtr> 
  {
    static Node encode(const giskard::AxisAngleSpecPtr& rhs) 
    {
      Node node;
      node["type"] = "ROTATION";
      node["axis"] = rhs->get_axis();
      node["angle"] = rhs->get_angle();
      return node;
    }
  
    static bool decode(const Node& node, giskard::AxisAngleSpecPtr& rhs) 
    {
      if(!is_axis_angle(node))
        return false;

      rhs = giskard::AxisAngleSpecPtr(new giskard::AxisAngleSpec()); 
      rhs->set_angle(node["angle"].as<giskard::DoubleSpecPtr>());
      rhs->set_axis(node["axis"].as<giskard::VectorSpecPtr>());

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

      return node;
    }
  
    static bool decode(const Node& node, giskard::RotationSpecPtr& rhs) 
    {
      if(is_axis_angle(node))
      {
        rhs = node.as<giskard::AxisAngleSpecPtr>();
        return true;
      }
      else
        return false;
    }
  }; 

  ///
  /// parsing frame specifications
  ///

  inline bool is_constructor_frame(const Node& node)
  {
    return node.IsMap() && (node.size() == 3) && node["type"] &&
        (node["type"].as<std::string>().compare("FRAME") == 0) &&
        node["translation"] && node["rotation"];
  }

  template<>
  struct convert<giskard::FrameConstructorSpecPtr> 
  {
    static Node encode(const giskard::FrameConstructorSpecPtr& rhs) 
    {
      Node node;
      node["type"] = "FRAME";
      node["translation"] = rhs->get_translation();
      node["rotation"] = rhs->get_rotation();
      return node;
    }
  
    static bool decode(const Node& node, giskard::FrameConstructorSpecPtr& rhs) 
    {
      if(!is_constructor_frame(node))
        return false;

      rhs = giskard::FrameConstructorSpecPtr(new giskard::FrameConstructorSpec()); 
      rhs->set_translation(node["translation"].as<giskard::VectorSpecPtr>());
      rhs->set_rotation(node["rotation"].as<giskard::RotationSpecPtr>());

      return true;
    }
  };

  inline bool is_frame_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 2) && node["type"] &&
        (node["type"].as<std::string>().compare("FRAME-MULTIPLICATION") == 0) &&
        node["inputs"] && node["inputs"].IsSequence();
  }

  template<>
  struct convert<giskard::FrameMultiplicationSpecPtr> 
  {
    static Node encode(const giskard::FrameMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["type"] = "FRAME-MULTIPLICATION";
      node["inputs"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::FrameMultiplicationSpecPtr& rhs) 
    {
      if(!is_frame_multiplication(node))
        return false;

      rhs = giskard::FrameMultiplicationSpecPtr(new giskard::FrameMultiplicationSpec()); 
      rhs->set_inputs(node["inputs"].as< std::vector<giskard::FrameSpecPtr> >());

      return true;
    }
  };

  inline bool is_frame_reference(const Node& node)
  {
    return node.IsMap() && (node.size() == 2) && node["type"] &&
        (node["type"].as<std::string>().compare("FRAME-REFERENCE") == 0) &&
        node["reference"];
  }

  template<>
  struct convert<giskard::FrameReferenceSpecPtr> 
  {
    
    static Node encode(const giskard::FrameReferenceSpecPtr& rhs) 
    {
      Node node;
      node["reference"] = rhs->get_reference_name();
      node["type"] = "FRAME-REFERENCE";
      return node;
    }
  
    static bool decode(const Node& node, giskard::FrameReferenceSpecPtr& rhs) 
    {
      if(!is_frame_reference(node))
        return false;
  
      rhs = giskard::FrameReferenceSpecPtr(new giskard::FrameReferenceSpec());
      rhs->set_reference_name(node["reference"].as<std::string>());

      return true;
    }
  };

  template<>
  struct convert<giskard::FrameSpecPtr> 
  {
    static Node encode(const giskard::FrameSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard::FrameConstructorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::FrameConstructorSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard::FrameMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::FrameMultiplicationSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard::FrameReferenceSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::FrameReferenceSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard::FrameSpecPtr& rhs) 
    {
      if(is_constructor_frame(node))
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
    return is_const_double(node) || is_input(node) || is_double_reference(node);
  }

  inline bool is_vector_spec(const Node& node)
  {
    return is_constructor_vector(node) || is_vector_reference(node) ||
        is_vector_origin_of(node);
  }

  inline bool is_rotation_spec(const Node& node)
  {
    return is_axis_angle(node);
  }

  inline bool is_frame_spec(const Node& node)
  {
    return is_constructor_frame(node) || is_frame_multiplication(node) || 
        is_frame_reference(node);
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

//std::cout << "Decode expression: " << node.begin()->first.as<std::string>() << "\n";

      rhs.name = node.begin()->first.as<std::string>();
      rhs.spec = node.begin()->second.as<giskard::SpecPtr>(); 

      return true;
    }
  };
}

#endif // GISKARD_YAML_PARSER_HPP
