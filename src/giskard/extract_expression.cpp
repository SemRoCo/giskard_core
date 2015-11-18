/*
* Copyright (C) 2015 Jannik Buckelo <jannikbu@cs.uni-bremen.de>
*
* This file is part of giskard.
*
* giskard is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <iostream>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <giskard/giskard.hpp>

int main(int argc, char **argv)
{
  if (argc != 4 && argc != 5)
  {
    std::cout << "Usage: rosrun giskard extract_expression <start_link> <end_link> <urdf> (optional <output_file>)" << std::endl;
    return 0;
  }
  std::string start_link = argv[1];
  std::string end_link = argv[2];
  std::string urdf_path = argv[3];
  YAML::Node yaml = giskard::ExpressionExtractor::extract(start_link, end_link, urdf_path);
  YAML::Emitter out;
  out << yaml;
  if (argc == 5)
  {
    std::ofstream output_file;
    output_file.open(argv[4]);
    if (!output_file.is_open())
      throw giskard::WriteError(argv[4]);
    output_file << out.c_str();
    output_file.close();
  }
  else
  {
    std::cout << out.c_str() << std::endl;
  }


  return 0;
}
