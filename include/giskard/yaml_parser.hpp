#ifndef GISKARD_YAML_PARSER_HPP
#define GISKARD_YAML_PARSER_HPP

#include <string>
#include <yaml-cpp/yaml.h>
#include <kdl/expressiontree.hpp>
#include <giskard/exceptions.hpp>
#include <giskard/kdl_typedefs.hpp>

namespace giskard 
{
  inline bool is_input_variable(const YAML::Node& node)
  {
    return node.IsMap() && node["name"] && node["type"] &&
      (node["type"].as<std::string>().compare("INPUT-VAR") == 0);
  }

  inline KDL::Expression<double>::Ptr parse_input_variable(const YAML::Node& node, 
      unsigned int var_index)
  {
    if(!is_input_variable(node))
      throw YamlParserException("Given yaml-node does not represent an input-variable.");

    KDL::InputTypePtr input(new KDL::InputType(var_index, 0.0));
    input->name = node["name"].as<std::string>();
    return input;
  }
}

#endif // GISKARD_YAML_PARSER_HPP
