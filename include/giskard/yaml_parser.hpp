#ifndef GISKARD_YAML_PARSER_HPP
#define GISKARD_YAML_PARSER_HPP

#include <yaml-cpp/yaml.h>
#include <kdl/expressiontree.hpp>

namespace giskard 
{
  bool is_input_variable(const YAML::Node& node);

  KDL::Expression<double>::Ptr parse_input_variable(const YAML::Node& node, 
      unsigned int var_index);

  bool is_input_frame(const YAML::Node& node);

  std::vector< KDL::Expression<double>::Ptr > parse_input_frame(
      const YAML::Node& node, unsigned int start_var_index);

  bool is_input_list(const YAML::Node& node);

  std::vector< KDL::Expression<double>::Ptr > parse_input_list(const YAML::Node& node);
}

#endif // GISKARD_YAML_PARSER_HPP
