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
  struct convert<giskard::ConstDoubleSpecPtr> 
  {
    
    static Node encode(const giskard::ConstDoubleSpecPtr& rhs) 
    {
      Node node;
      node = rhs->get_value();
      return node;
    }
  
    static bool decode(const Node& node, giskard::ConstDoubleSpecPtr& rhs) 
    {
      if(!is_const_double(node))
        return false;
  
      rhs = giskard::ConstDoubleSpecPtr(new giskard::ConstDoubleSpec());
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
  struct convert<giskard::InputDoubleSpecPtr> 
  {
    
    static Node encode(const giskard::InputDoubleSpecPtr& rhs) 
    {
      Node node;
      node["input-number"] = rhs->get_input_num();
      node["type"] = "INPUT";
      return node;
    }
  
    static bool decode(const Node& node, giskard::InputDoubleSpecPtr& rhs) 
    {
      if(!is_input(node))
        return false;
  
      rhs = giskard::InputDoubleSpecPtr(new giskard::InputDoubleSpec());
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
  struct convert<giskard::ReferenceDoubleSpecPtr> 
  {
    
    static Node encode(const giskard::ReferenceDoubleSpecPtr& rhs) 
    {
      Node node;
      node["reference"] = rhs->get_reference_name();
      node["type"] = "DOUBLE-REFERENCE";
      return node;
    }
  
    static bool decode(const Node& node, giskard::ReferenceDoubleSpecPtr& rhs) 
    {
      if(!is_double_reference(node))
        return false;
  
      rhs = giskard::ReferenceDoubleSpecPtr(new giskard::ReferenceDoubleSpec());
      rhs->set_reference_name(node["reference"].as<std::string>());

      return true;
    }
  };

  template<>
  struct convert<giskard::DoubleSpecPtr> 
  {
    
    static Node encode(const giskard::DoubleSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rhs).get())
      {
        giskard::ConstDoubleSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::ConstDoubleSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(rhs).get())
      {
        giskard::InputDoubleSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::InputDoubleSpec>(rhs);
        node = p;
      }
      else if(boost::dynamic_pointer_cast<giskard::ReferenceDoubleSpec>(rhs).get())
      {
        giskard::ReferenceDoubleSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::ReferenceDoubleSpec>(rhs);
        node = p;
      }

      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleSpecPtr& rhs) 
    {
      if(is_const_double(node))
      {
        rhs = node.as<giskard::ConstDoubleSpecPtr>();
        return true;
      }
      else if(is_input(node))
      {
        rhs = node.as<giskard::InputDoubleSpecPtr>();
        return true;
      }
      else if(is_double_reference(node))
      {
        rhs = node.as<giskard::ReferenceDoubleSpecPtr>();
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
  struct convert<giskard::ConstructorVectorSpecPtr> 
  {
    static Node encode(const giskard::ConstructorVectorSpecPtr& rhs) 
    {
      Node node;
      node["type"] = "VECTOR3";
      node["inputs"].push_back(rhs->get_x());
      node["inputs"].push_back(rhs->get_y());
      node["inputs"].push_back(rhs->get_z());
      return node;
    }
  
    static bool decode(const Node& node, giskard::ConstructorVectorSpecPtr& rhs) 
    {
      if(!is_constructor_vector(node))
        return false;

      rhs = giskard::ConstructorVectorSpecPtr(new giskard::ConstructorVectorSpec()); 
      rhs->set_x(node["inputs"][0].as<giskard::DoubleSpecPtr>());
      rhs->set_y(node["inputs"][1].as<giskard::DoubleSpecPtr>());
      rhs->set_z(node["inputs"][2].as<giskard::DoubleSpecPtr>());

      return true;
    }
  };

  template<>
  struct convert<giskard::VectorSpecPtr> 
  {
    static Node encode(const giskard::VectorSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::ConstructorVectorSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorSpecPtr& rhs) 
    {
      if(is_constructor_vector(node))
      {
        rhs = node.as<giskard::ConstructorVectorSpecPtr>();
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
  struct convert<giskard::ConstructorFrameSpecPtr> 
  {
    static Node encode(const giskard::ConstructorFrameSpecPtr& rhs) 
    {
      Node node;
      node["type"] = "FRAME";
      node["translation"] = rhs->get_translation();
      node["rotation"] = rhs->get_rotation();
      return node;
    }
  
    static bool decode(const Node& node, giskard::ConstructorFrameSpecPtr& rhs) 
    {
      if(!is_constructor_frame(node))
        return false;

      rhs = giskard::ConstructorFrameSpecPtr(new giskard::ConstructorFrameSpec()); 
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
  struct convert<giskard::MultiplicationFrameSpecPtr> 
  {
    static Node encode(const giskard::MultiplicationFrameSpecPtr& rhs) 
    {
      Node node;
      node["type"] = "FRAME-MULTIPLICATION";
      node["inputs"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::MultiplicationFrameSpecPtr& rhs) 
    {
      if(!is_frame_multiplication(node))
        return false;

      rhs = giskard::MultiplicationFrameSpecPtr(new giskard::MultiplicationFrameSpec()); 
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

      if(boost::dynamic_pointer_cast<giskard::ConstructorFrameSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::ConstructorFrameSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard::MultiplicationFrameSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::MultiplicationFrameSpec>(rhs);
      else if (boost::dynamic_pointer_cast<giskard::FrameReferenceSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::FrameReferenceSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard::FrameSpecPtr& rhs) 
    {
      if(is_constructor_frame(node))
      {
        rhs = node.as<giskard::ConstructorFrameSpecPtr>();
        return true;
      }
      else if(is_frame_multiplication(node))
      {
        rhs = node.as<giskard::MultiplicationFrameSpecPtr>();
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
    return is_constructor_vector(node);
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
  /// parsing of maps of expressions
  ///
  inline giskard::SpecMap parse_scope(const Node& node)
  {
    return node.as< giskard::SpecMap>();
  }

}

#endif // GISKARD_YAML_PARSER_HPP
