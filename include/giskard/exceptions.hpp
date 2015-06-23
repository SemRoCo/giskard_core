#ifndef GISKARD_EXCEPTIONS_HPP
#define GISKARD_EXCEPTIONS_HPP

#include <stdexcept>

namespace giskard
{
  class Exception : public std::runtime_error 
  {
    public:
      Exception(const std::string& what) : 
          std::runtime_error(what) {}
  };

  class YamlParserException : public Exception 
  {
    public:
      YamlParserException(const std::string& what) : 
          Exception(what) {}
  };
}

#endif // GISKARD_EXCEPTIONS_HPP
