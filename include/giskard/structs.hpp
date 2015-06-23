#ifndef GISKARD_STRUCTS_HPP
#define GISKARD_STRUCTS_HPP

#include <string>

namespace giskard
{
  class OutputSpec
  {
    public:
      std::string name_;
      double lower_vel_limit_, upper_vel_limit_, weight_;
  };
}

#endif // GISKARD_STRUCTS_HPP
