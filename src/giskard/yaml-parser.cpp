#include <giskard/yaml_parser.hpp>
#include <giskard/kdl_typedefs.hpp>
#include <giskard/exceptions.hpp>
#include <boost/assign/list_of.hpp>

namespace giskard
{
  bool is_input_variable(const YAML::Node& node)
  {
    return node.IsMap() && node["name"] && node["type"] &&
      (node["type"].as<std::string>().compare("INPUT-VARIABLE") == 0);
  }

  KDL::Expression<double>::Ptr parse_input_variable(const YAML::Node& node, 
      unsigned int var_index)
  {
    if(!is_input_variable(node))
      throw YamlParserException("Given yaml-node does not represent an input-variable.");

    KDL::InputTypePtr input(new KDL::InputType(var_index, 0.0));
    input->name = node["name"].as<std::string>();
    return input;
  }

  //
  // parse input-frame description
  //

  bool is_input_frame(const YAML::Node& node)
  {
    return node.IsMap() && node["name"] && node["type"] &&
        (node["type"].as<std::string>().compare("INPUT-FRAME") == 0);
  }

  std::vector< KDL::Expression<double>::Ptr > parse_input_frame(
      const YAML::Node& node, unsigned int start_var_index)
  {
    if(!is_input_frame(node))
      throw YamlParserException("Given yaml-node does not represent an input-frame.");

    std::vector<std::string> postfixes = boost::assign::list_of(" translation-x")
        (" translation-y")(" translation-z")(" rotation-x")(" rotation-y")(" rotation-z");
    std::string frame_name = node["name"].as<std::string>();

    std::vector< KDL::Expression<double>::Ptr > result;
    for(unsigned int i=0; i<postfixes.size(); ++i)
    {
      KDL::InputTypePtr input(new KDL::InputType(start_var_index + i, 0.0));
      input->name = frame_name + postfixes[i];
      result.push_back(input); 
    }
    
    return result;
  }

}
