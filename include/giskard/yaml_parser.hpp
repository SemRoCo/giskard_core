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
  
      rhs.name_ = node["name"].as<std::string>();
      rhs.lower_vel_limit_ = node["lower_velocity_limit"].as<double>();
      rhs.upper_vel_limit_ = node["upper_velocity_limit"].as<double>();
      rhs.weight_ = node["weight"].as<double>();

      return true;
    }
  };

}

#endif // GISKARD_YAML_PARSER_HPP
