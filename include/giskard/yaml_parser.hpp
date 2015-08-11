#ifndef GISKARD_YAML_PARSER_HPP
#define GISKARD_YAML_PARSER_HPP

#include <yaml-cpp/yaml.h>
#include <vector>
#include <giskard/specifications.hpp>

namespace YAML {

  template<>
  struct convert<giskard::ConstDoubleSpec> {
    
    static Node encode(const giskard::ConstDoubleSpec& rhs) {
      Node node;
      node = rhs.get_value();
      return node;
    }
  
    static bool decode(const Node& node, giskard::ConstDoubleSpec& rhs) {
      if(!node.IsScalar())
        return false;
  
      rhs.clear();
      rhs.set_value(node.as<double>());

      return true;
    }
  };

  template<>
  struct convert<giskard::InputDoubleSpec> {
    
    static Node encode(const giskard::InputDoubleSpec& rhs) {
      Node node;
      node["input-number"] = rhs.get_input_num();
      node["type"] = "INPUT";
      return node;
    }
  
    static bool decode(const Node& node, giskard::InputDoubleSpec& rhs) {
      if(!(node.IsMap() && (node.size() == 2) && node["type"] && node["input-number"] &&
          (node["type"].as<std::string>().compare("INPUT") == 0)))
        return false;
  
      rhs.clear();
      rhs.set_input_num(node["input-number"].as<double>());

      return true;
    }
  };


//  inline bool is_observable_spec(const Node& node)
//  {
//    return node.IsMap() && (node.size() == 2) &&
//        node["name"] && node["type"];
//  }
//
//  inline bool is_controllable_spec(const Node& node)
//  {
//    return node.IsMap() && (node.size() == 3) &&
//        node["name"] && node["type"] && node["reference"];
//  }
//
//  inline bool is_named_expression_spec (const Node& node)
//  {
//    return node.IsMap() && (node.size() == 3) &&
//        node["name"] && node["type"] && node["inputs"] &&
//        node["inputs"].IsSequence();
//  }
//
//  inline bool is_anonymous_expression_spec (const Node& node)
//  {
//    return node.IsMap() && (node.size() == 2) &&
//        node["type"] && node["inputs"] && node["inputs"].IsSequence();
//  }
//
//  inline bool is_expression_spec(const Node& node)
//  {
//    return node.IsScalar() || is_named_expression_spec(node) ||
//       is_anonymous_expression_spec(node);
//  }
//
//  inline bool is_hard_constraint_spec(const Node& node)
//  {
//    return node.IsMap() && (node.size() == 6) && node["type"] &&
//        (node["type"].as<std::string>().compare("HARD-CONSTRAINT") == 0) &&
//        node["name"] && node["lower"] && node["upper"] && node["weight"] &&
//        node["expression"];
//  }
//
//  inline bool is_soft_constraint_spec(const Node& node)
//  {
//    return node.IsMap() && (node.size() == 7) && node["type"] &&
//        (node["type"].as<std::string>().compare("SOFT-CONSTRAINT") == 0) &&
//        node["name"] && node["lower"] && node["upper"] && node["weight"] &&
//        node["gain"] && node["expression"];
//  }
//
//  inline bool is_controller_spec(const Node& node)
//  {
//    return node.IsMap() && (node.size() == 4) && 
//      node["observables"] && node["observables"].IsSequence() && 
//      node["expressions"] && node["expressions"].IsSequence() && 
//      node["controllables"] && node["controllables"].IsSequence() && 
//      node["constraints"] && node["constraints"].IsSequence(); 
//  }
//
//  template<>
//  struct convert<giskard::ObservableSpec> {
//    
//    static Node encode(const giskard::ObservableSpec& rhs) {
//      Node node;
//      node["name"] = rhs.name_;
//      node["type"] = rhs.type_;
//      return node;
//    }
//  
//    static bool decode(const Node& node, giskard::ObservableSpec& rhs) {
//      if(!is_observable_spec(node))
//        return false;
//  
//      rhs.clear();
//
//      rhs.name_ = node["name"].as<std::string>();
//      rhs.type_ = node["type"].as<std::string>();
//      return true;
//    }
//  };
//
//  template<>
//  struct convert<giskard::ControllableSpec> {
//    
//    static Node encode(const giskard::ControllableSpec& rhs) {
//      Node node;
//      node["name"] = rhs.name_;
//      node["type"] = rhs.type_;
//      node["reference"] = rhs.reference_;
//      return node;
//    }
//  
//    static bool decode(const Node& node, giskard::ControllableSpec& rhs) {
//      if(!is_controllable_spec(node))
//        return false;
//
//      rhs.clear();
//  
//      rhs.name_ = node["name"].as<std::string>();
//      rhs.type_ = node["type"].as<std::string>();
//      rhs.reference_ = node["reference"].as<std::string>();
//
//      return true;
//    }
//  };
//
//  template<>
//  struct convert<giskard::ExpressionSpec> {
//    
//    static Node encode(const giskard::ExpressionSpec& rhs) {
//      Node node;
//
//      if(rhs.type_.compare("CONSTANT") == 0)
//      {
//        node = rhs.value_;
//      }
//      else if(rhs.type_.compare("REFERENCE") == 0)
//      {
//        node = rhs.name_;
//      } 
//      else 
//      {
//        if(rhs.name_.length() > 0)
//          node["name"] = rhs.name_;
//        node["type"] = rhs.type_;
//        for(unsigned int i=0; i<rhs.inputs_.size(); ++i)
//          node["inputs"][i] = rhs.inputs_[i];
//      }
//
//      return node;
//    }
//  
//    static bool decode(const Node& node, giskard::ExpressionSpec& rhs) {
//      if(!is_expression_spec(node))
//        return false;
//      rhs.clear();
//
//      if(node.IsScalar())
//      {
//        try 
//        {
//          rhs.value_ = node.as<double>();
//          rhs.type_ = "CONSTANT";
//        } catch(const YAML::RepresentationException& e) {
//          rhs.name_ = node.as<std::string>();
//          rhs.type_ = "REFERENCE";
//        }
//        return true;
//      }
//
//      rhs.type_ = node["type"].as<std::string>();
//      rhs.inputs_ = node["inputs"].as< std::vector<giskard::ExpressionSpec> >();
//      if(node["name"])
//        rhs.name_ = node["name"].as<std::string>();
//
//      return true;
//    }
//  };
//
//  template<>
//  struct convert<giskard::ConstraintSpec> {
//    
//    static Node encode(const giskard::ConstraintSpec& rhs) {
//      Node node;
//      node["name"] = rhs.name_;
//      node["type"] = rhs.type_;
//      node["expression"] = rhs.expression_;
//      node["lower"] = rhs.lower_;
//      node["upper"] = rhs.upper_;
//      node["weight"] = rhs.weight_;
//
//      if(rhs.type_.compare("SOFT-CONSTRAINT") == 0)
//        node["gain"] = rhs.gain_;
//
//      return node;
//    }
//  
//    static bool decode(const Node& node, giskard::ConstraintSpec& rhs) {
//      if(is_soft_constraint_spec(node))
//      {
//        rhs.clear();
//        rhs.name_ = node["name"].as<std::string>();
//        rhs.type_ = node["type"].as<std::string>();
//        rhs.expression_ = node["expression"].as<std::string>();
//        rhs.weight_ = node["weight"].as<double>();
//        rhs.lower_ = node["lower"].as<double>();
//        rhs.upper_ = node["upper"].as<double>();
//        rhs.gain_ = node["gain"].as<double>();
// 
//        return true;
//      }
//
//      if(is_hard_constraint_spec(node))
//      {
//        rhs.clear();
//        rhs.name_ = node["name"].as<std::string>();
//        rhs.type_ = node["type"].as<std::string>();
//        rhs.expression_ = node["expression"].as<std::string>();
//        rhs.weight_ = node["weight"].as<double>();
//        rhs.lower_ = node["lower"].as<double>();
//        rhs.upper_ = node["upper"].as<double>();
// 
//        return true;
//      }
//
//      return false;
//    }
//  };
//
//  template<>
//  struct convert<giskard::ControllerSpec> 
//  {
//    
//    static Node encode(const giskard::ControllerSpec& rhs) 
//    {
//      Node node;
//      node["observables"] = rhs.observables_;
//      node["expressions"] = rhs.expressions_;
//      node["controllables"] = rhs.controllables_;
//      node["constraints"] = rhs.constraints_;
//
//      return node;
//    }
//  
//    static bool decode(const Node& node, giskard::ControllerSpec& rhs) 
//    {
//      if(is_controller_spec(node))
//      {
//        rhs.clear();
//        rhs.observables_ = node["observables"].as< std::vector<giskard::ObservableSpec> >();
//        rhs.expressions_ = node["expressions"].as< std::vector<giskard::ExpressionSpec> >();
//        rhs.controllables_ = node["controllables"].as< std::vector<giskard::ControllableSpec> >();
//        rhs.constraints_ = node["constraints"].as< std::vector<giskard::ConstraintSpec> >();
// 
//        return true;
//      }
//      else
//        return false;
//    }
//  };

}

#endif // GISKARD_YAML_PARSER_HPP
