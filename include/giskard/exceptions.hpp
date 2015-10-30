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

#ifndef GISKARD_EXCEPTIONS_HPP
#define GISKARD_EXCEPTIONS_HPP

namespace giskard
{
  namespace ErrorMsg
  {
    const char* const DEFAULT_ERR = "An error occured.";
    const char* const INVALID_CHAIN = "No transform between the specified links found.";
    const char* const INVALID_URDF = "Couldn't parse urdf.";
  }

  class Exception: public std::exception
  {
    protected:
      const char* err_msg;

    public:
      Exception() : err_msg(ErrorMsg::DEFAULT_ERR) {};
      Exception(const char* msg) : err_msg(msg) {};

      virtual const char* what() const throw()
      {
        return err_msg;
      }
  };

  class InvalidChain: public Exception
  {
    public:
      InvalidChain() : Exception(ErrorMsg::INVALID_CHAIN) {};
      InvalidChain(std::string start_link, std::string end_link)
      {
          std::string msg = "No transform found from '" + start_link + "' to '" + end_link + "'.";
          err_msg = msg.c_str();
      }
  };

  class InvalidUrdf: public Exception
  {
    public:
      InvalidUrdf() : Exception(ErrorMsg::INVALID_URDF) {};
      InvalidUrdf(std::string path)
      {
          std::string msg = "Couldn't parse urdf: '" + path + "'.";
          err_msg = msg.c_str();
      }
  };

  class WriteError: public Exception
  {
    public:
      WriteError(std::string path)
      {
          std::string msg = "Failed to write file '" + path + "'.";
          err_msg = msg.c_str();
      }
  };
}

#endif // GISKARD_EXCEPTIONS_HPP
