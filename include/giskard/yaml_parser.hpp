#ifndef GISKARD_YAML_PARSER_HPP
#define GISKARD_YAML_PARSER_HPP

#include <yaml-cpp/yaml.h>
#include <vector>
#include <giskard/structs.hpp>

namespace YAML {
  inline bool is_observable_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 2) &&
        node["name"] && node["type"];
  }

  inline bool is_controllable_spec(const Node& node)
  {
    return node.IsMap() && (node.size() == 4) &&
        node["name"] && node["weight"] &&
        node["lower_velocity_limit"] && 
        node["upper_velocity_limit"];
  }

  inline bool is_named_expression_spec (const Node& node)
  {
    return node.IsMap() && (node.size() == 3) &&
        node["name"] && node["type"] && node["inputs"] &&
        node["inputs"].IsSequence();
  }

  inline bool is_anonymous_expression_spec (const Node& node)
  {
    return node.IsMap() && (node.size() == 2) &&
        node["type"] && node["inputs"] && node["inputs"].IsSequence();
  }

  inline bool is_expression_spec(const Node& node)
  {
    return node.IsScalar() || is_named_expression_spec(node) ||
       is_anonymous_expression_spec(node);
  }

  template<>
  struct convert<giskard::ObservableSpec> {
    
    static Node encode(const giskard::ObservableSpec& rhs) {
      Node node;
      node["name"] = rhs.name_;
      node["type"] = rhs.type_;
      return node;
    }
  
    static bool decode(const Node& node, giskard::ObservableSpec& rhs) {
      if(!is_observable_spec(node))
        return false;
  
      rhs.clear();

      rhs.name_ = node["name"].as<std::string>();
      rhs.type_ = node["type"].as<std::string>();
      return true;
    }
  };

  template<>
  struct convert<giskard::ControllableSpec> {
    
    static Node encode(const giskard::ControllableSpec& rhs) {
      Node node;
      node["name"] = rhs.name_;
      node["lower_velocity_limit"] = rhs.lower_vel_limit_;
      node["upper_velocity_limit"] = rhs.upper_vel_limit_;
      node["weight"] = rhs.weight_;
      return node;
    }
  
    static bool decode(const Node& node, giskard::ControllableSpec& rhs) {
      if(!is_controllable_spec(node))
        return false;

      rhs.clear();
  
      rhs.name_ = node["name"].as<std::string>();
      rhs.lower_vel_limit_ = node["lower_velocity_limit"].as<double>();
      rhs.upper_vel_limit_ = node["upper_velocity_limit"].as<double>();
      rhs.weight_ = node["weight"].as<double>();

      return true;
    }
  };

  template<>
  struct convert<giskard::ExpressionSpec> {
    
    static Node encode(const giskard::ExpressionSpec& rhs) {
      Node node;

      if(rhs.type_.compare("CONSTANT") == 0)
      {
        node = rhs.value_;
      }
      else if(rhs.type_.compare("REFERENCE") == 0)
      {
        node = rhs.name_;
      } 
      else 
      {
        if(rhs.name_.length() > 0)
          node["name"] = rhs.name_;
        node["type"] = rhs.type_;
        for(unsigned int i=0; i<rhs.inputs_.size(); ++i)
          node["inputs"][i] = rhs.inputs_[i];
      }

      return node;
    }
  
    static bool decode(const Node& node, giskard::ExpressionSpec& rhs) {
      if(!is_expression_spec(node))
        return false;
      rhs.clear();

      if(node.IsScalar())
      {
        try 
        {
          rhs.value_ = node.as<double>();
          rhs.type_ = "CONSTANT";
        } catch(const YAML::RepresentationException& e) {
          rhs.name_ = node.as<std::string>();
          rhs.type_ = "REFERENCE";
        }
        return true;
      }

      rhs.type_ = node["type"].as<std::string>();
      rhs.inputs_ = node["inputs"].as< std::vector<giskard::ExpressionSpec> >();
      if(node["name"])
        rhs.name_ = node["name"].as<std::string>();

      return true;
    }
  };

}

#endif // GISKARD_YAML_PARSER_HPP
