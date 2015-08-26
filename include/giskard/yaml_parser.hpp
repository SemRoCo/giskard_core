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

  template<>
  struct convert<giskard::RotationSpecPtr> 
  {
    static Node encode(const giskard::RotationSpecPtr& rhs) 
    {
      Node node;

      if(boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::AxisAngleSpec>(rhs);
      else if(boost::dynamic_pointer_cast<giskard::RotationReferenceSpec>(rhs).get())
        node = boost::dynamic_pointer_cast<giskard::RotationReferenceSpec>(rhs);

      return node;
    }
  
    static bool decode(const Node& node, giskard::RotationSpecPtr& rhs) 
    {
      if(is_axis_angle(node))
      {
        rhs = node.as<giskard::AxisAngleSpecPtr>();
        return true;
      }
      if(is_rotation_reference(node))
      {
        rhs = node.as<giskard::RotationReferenceSpecPtr>();
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
        is_x_coord_of(node) || is_y_coord_of(node) || is_z_coord_of(node);
  }

  inline bool is_vector_spec(const Node& node)
  {
    return is_constructor_vector(node) || is_vector_reference(node) ||
        is_vector_origin_of(node) || is_vector_subtraction(node) ||
        is_vector_frame_multiplication(node);
  }

  inline bool is_rotation_spec(const Node& node)
  {
    return is_axis_angle(node) || is_rotation_reference(node);
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
    return node.IsMap() && (node.size() == 1) && node["controllable-constraint"] &&
        node["controllable-constraint"].IsSequence() && (node["controllable-constraint"].size() == 4);
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

      return true;
    }
  };

  inline bool is_soft_constraint_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 1) && node["soft-constraint"] &&
        node["soft-constraint"].IsSequence() && (node["soft-constraint"].size() == 4);
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
