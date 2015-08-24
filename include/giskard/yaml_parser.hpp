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
    return node.IsMap() && (node.size() == 1) && node["double"];
  }

  template<>
  struct convert<giskard::DoubleReferenceSpecPtr> 
  {
    
    static Node encode(const giskard::DoubleReferenceSpecPtr& rhs) 
    {
      Node node;
      node["double"] = rhs->get_reference_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleReferenceSpecPtr& rhs) 
    {
      if(!is_double_reference(node))
        return false;
  
      rhs = giskard::DoubleReferenceSpecPtr(new giskard::DoubleReferenceSpec());
      rhs->set_reference_name(node["double"].as<std::string>());

      return true;
    }
  };

  inline bool is_double_addition(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["double-addition"] &&
        node["double-addition"].IsSequence();
  }

  template<>
  struct convert<giskard::DoubleAdditionSpecPtr> 
  {
    static Node encode(const giskard::DoubleAdditionSpecPtr& rhs) 
    {
      Node node;
      node["double-addition"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleAdditionSpecPtr& rhs) 
    {
      if(!is_double_addition(node))
        return false;

      rhs = giskard::DoubleAdditionSpecPtr(new giskard::DoubleAdditionSpec()); 
      rhs->set_inputs(node["double-addition"].as< std::vector<giskard::DoubleSpecPtr> >());

      return true;
    }
  };

  inline bool is_double_subtraction(const Node& node)
  {
    return node.IsMap() && (node.size() == 2) && node["type"] &&
        (node["type"].as<std::string>().compare("DOUBLE-SUBTRACTION") == 0) &&
        node["inputs"] && node["inputs"].IsSequence();
  }

  template<>
  struct convert<giskard::DoubleSubtractionSpecPtr> 
  {
    static Node encode(const giskard::DoubleSubtractionSpecPtr& rhs) 
    {
      Node node;
      node["type"] = "DOUBLE-SUBTRACTION";
      node["inputs"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleSubtractionSpecPtr& rhs) 
    {
      if(!is_double_subtraction(node))
        return false;

      rhs = giskard::DoubleSubtractionSpecPtr(new giskard::DoubleSubtractionSpec()); 
      rhs->set_inputs(node["inputs"].as< std::vector<giskard::DoubleSpecPtr> >());

      return true;
    }
  };

  inline bool is_double_norm_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 2) && node["type"] &&
        (node["type"].as<std::string>().compare("NORM-OF") == 0) &&
        node["vector"];
  }

  template<>
  struct convert<giskard::DoubleNormOfSpecPtr> 
  {
    
    static Node encode(const giskard::DoubleNormOfSpecPtr& rhs) 
    {
      Node node;
      node["type"] = "NORM-OF";
      node["vector"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleNormOfSpecPtr& rhs) 
    {
      if(!is_double_norm_of(node))
        return false;
  
      rhs = giskard::DoubleNormOfSpecPtr(new giskard::DoubleNormOfSpec());
      rhs->set_vector(node["vector"].as<giskard::VectorSpecPtr>());

      return true;
    }
  };

  inline bool is_double_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 2) && node["type"] &&
        (node["type"].as<std::string>().compare("DOUBLE-MULTIPLICATION") == 0) &&
        node["inputs"] && node["inputs"].IsSequence();
  }

  template<>
  struct convert<giskard::DoubleMultiplicationSpecPtr> 
  {
    static Node encode(const giskard::DoubleMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["type"] = "DOUBLE-MULTIPLICATION";
      node["inputs"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleMultiplicationSpecPtr& rhs) 
    {
      if(!is_double_multiplication(node))
        return false;

      rhs = giskard::DoubleMultiplicationSpecPtr(new giskard::DoubleMultiplicationSpec()); 
      rhs->set_inputs(node["inputs"].as< std::vector<giskard::DoubleSpecPtr> >());

      return true;
    }
  };

  inline bool is_z_coord_of(const Node& node)
  {
    return node.IsMap() && (node.size() == 2) && node["type"] &&
        (node["type"].as<std::string>().compare("Z-COORD-OF") == 0) &&
        node["vector"];
  }

  template<>
  struct convert<giskard::DoubleZCoordOfSpecPtr> 
  {
    static Node encode(const giskard::DoubleZCoordOfSpecPtr& rhs) 
    {
      Node node;
      node["type"] = "Z-COORD-OF";
      node["vector"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::DoubleZCoordOfSpecPtr& rhs) 
    {
      if(!is_z_coord_of(node))
        return false;

      rhs = giskard::DoubleZCoordOfSpecPtr(new giskard::DoubleZCoordOfSpec()); 
      rhs->set_vector(node["vector"].as< giskard::VectorSpecPtr >());

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
      else if(boost::dynamic_pointer_cast<giskard::DoubleZCoordOfSpec>(rhs).get())
      {
        giskard::DoubleZCoordOfSpecPtr p = 
            boost::dynamic_pointer_cast<giskard::DoubleZCoordOfSpec>(rhs);
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
      else if(is_double_norm_of(node))
      {
        rhs = node.as<giskard::DoubleNormOfSpecPtr>();
        return true;
      }
      else if(is_z_coord_of(node))
      {
        rhs = node.as<giskard::DoubleZCoordOfSpecPtr>();
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
    return node.IsMap() && (node.size() == 1) && node["vector3"] &&
        node["vector3"].IsScalar();
  }

  template<>
  struct convert<giskard::VectorReferenceSpecPtr> 
  {
    
    static Node encode(const giskard::VectorReferenceSpecPtr& rhs) 
    {
      Node node;
      node["vector3"] = rhs->get_reference_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorReferenceSpecPtr& rhs) 
    {
      if(!is_vector_reference(node))
        return false;
  
      rhs = giskard::VectorReferenceSpecPtr(new giskard::VectorReferenceSpec());
      rhs->set_reference_name(node["vector3"].as<std::string>());

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

  inline bool is_vector_subtraction(const Node& node)
  {
    return node.IsMap() && (node.size() == 2) && node["type"] &&
        (node["type"].as<std::string>().compare("VECTOR-SUBTRACTION") == 0) &&
        node["inputs"] && node["inputs"].IsSequence();
  }

  template<>
  struct convert<giskard::VectorSubtractionSpecPtr> 
  {
    static Node encode(const giskard::VectorSubtractionSpecPtr& rhs) 
    {
      Node node;
      node["type"] = "VECTOR-SUBTRACTION";
      node["inputs"] = rhs->get_inputs();
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorSubtractionSpecPtr& rhs) 
    {
      if(!is_vector_subtraction(node))
        return false;

      rhs = giskard::VectorSubtractionSpecPtr(new giskard::VectorSubtractionSpec()); 
      rhs->set_inputs(node["inputs"].as< std::vector<giskard::VectorSpecPtr> >());

      return true;
    }
  };

  inline bool is_vector_frame_multiplication(const Node& node)
  {
    return node.IsMap() && (node.size() == 3) &&  node["type"] &&
        (node["type"].as<std::string>().compare("MULTIPLICATION") == 0) &&
        node["frame"] && node["vector"];
  }

  template<>
  struct convert<giskard::VectorFrameMultiplicationSpecPtr> 
  {
    static Node encode(const giskard::VectorFrameMultiplicationSpecPtr& rhs) 
    {
      Node node;
      node["type"] = "MULTIPLICAION";
      node["frame"] = rhs->get_frame();
      node["vector"] = rhs->get_vector();
      return node;
    }
  
    static bool decode(const Node& node, giskard::VectorFrameMultiplicationSpecPtr& rhs) 
    {
      if(!is_vector_frame_multiplication(node))
        return false;

      rhs = giskard::VectorFrameMultiplicationSpecPtr(new giskard::VectorFrameMultiplicationSpec()); 
      rhs->set_frame(node["frame"].as< giskard::FrameSpecPtr >());
      rhs->set_vector(node["vector"].as< giskard::VectorSpecPtr >());

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
      else if(boost::dynamic_pointer_cast<giskard::VectorSubtractionSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::VectorSubtractionSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::VectorFrameMultiplicationSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::VectorFrameMultiplicationSpec>(rhs);

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
    return node.IsMap() && (node.size() == 1) && node["frame"];
  }

  template<>
  struct convert<giskard::FrameReferenceSpecPtr> 
  {
    
    static Node encode(const giskard::FrameReferenceSpecPtr& rhs) 
    {
      Node node;
      node["frame"] = rhs->get_reference_name();
      return node;
    }
  
    static bool decode(const Node& node, giskard::FrameReferenceSpecPtr& rhs) 
    {
      if(!is_frame_reference(node))
        return false;
  
      rhs = giskard::FrameReferenceSpecPtr(new giskard::FrameReferenceSpec());
      rhs->set_reference_name(node["frame"].as<std::string>());

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
    return is_const_double(node) || is_input(node) || is_double_reference(node) ||
        is_double_norm_of(node) || is_double_multiplication(node) || is_double_subtraction(node) ||
        is_z_coord_of(node);
  }

  inline bool is_vector_spec(const Node& node)
  {
    return is_constructor_vector(node) || is_vector_reference(node) ||
        is_vector_origin_of(node) || is_vector_subtraction(node) ||
        is_vector_frame_multiplication(node);
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

      rhs.name = node.begin()->first.as<std::string>();
      rhs.spec = node.begin()->second.as<giskard::SpecPtr>(); 

      return true;
    }
  };

  inline bool is_controllable_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 4) && node["lower"] &&
        node["upper"] && node["weight"] && node["input-number"];
  }

  template<>
  struct convert<giskard::ControllableConstraintSpec> 
  {
    static Node encode(const giskard::ControllableConstraintSpec& rhs) 
    {
      YAML::Node node;

      node["lower"] = rhs.lower_;
      node["upper"] = rhs.upper_;
      node["weight"] = rhs.weight_;
      node["input-number"] = rhs.input_number_;

      return node;
    }
  
    static bool decode(const Node& node, giskard::ControllableConstraintSpec& rhs) 
    {
      if(!is_controllable_spec(node))
        return false;

      rhs.lower_ = node["lower"].as<giskard::DoubleSpecPtr>();
      rhs.upper_ = node["upper"].as<giskard::DoubleSpecPtr>();
      rhs.weight_ = node["weight"].as<giskard::DoubleSpecPtr>();
      rhs.input_number_ = node["input-number"].as<size_t>();

      return true;
    }
  };

  inline bool is_soft_constraint_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 4) && node["lower"] &&
        node["upper"] && node["weight"] && node["expression"];
  }

  template<>
  struct convert<giskard::SoftConstraintSpec> 
  {
    static Node encode(const giskard::SoftConstraintSpec& rhs) 
    {
      YAML::Node node;

      node["lower"] = rhs.lower_;
      node["upper"] = rhs.upper_;
      node["weight"] = rhs.weight_;
      node["expression"] = rhs.expression_;

      return node;
    }
  
    static bool decode(const Node& node, giskard::SoftConstraintSpec& rhs) 
    {
      if(!is_soft_constraint_spec(node))
        return false;

      rhs.lower_ = node["lower"].as<giskard::DoubleSpecPtr>();
      rhs.upper_ = node["upper"].as<giskard::DoubleSpecPtr>();
      rhs.weight_ = node["weight"].as<giskard::DoubleSpecPtr>();
      rhs.expression_ = node["expression"].as<giskard::DoubleSpecPtr>();

      return true;
    }
  };

  inline bool is_hard_constraint_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 3) && node["lower"] &&
        node["upper"] && node["expression"];
  }

  template<>
  struct convert<giskard::HardConstraintSpec> 
  {
    static Node encode(const giskard::HardConstraintSpec& rhs) 
    {
      YAML::Node node;

      node["lower"] = rhs.lower_;
      node["upper"] = rhs.upper_;
      node["expression"] = rhs.expression_;

      return node;
    }
  
    static bool decode(const Node& node, giskard::HardConstraintSpec& rhs) 
    {
      if(!is_hard_constraint_spec(node))
        return false;

      rhs.lower_ = node["lower"].as<giskard::DoubleSpecPtr>();
      rhs.upper_ = node["upper"].as<giskard::DoubleSpecPtr>();
      rhs.expression_ = node["expression"].as<giskard::DoubleSpecPtr>();

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
