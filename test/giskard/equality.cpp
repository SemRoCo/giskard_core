/*
 * Copyright (C) 2016 Georg Bartels <georg.bartels@cs.uni-bremen.de>
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

#include <gtest/gtest.h>
#include <giskard/giskard.hpp>

class EqualityTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
    }

    virtual void TearDown(){}


};

TEST_F(EqualityTest, SlerpControl)
{
  std::vector<std::string> names;
  names.push_back("l_trans_control");
  names.push_back("r_trans_control");
  names.push_back("l_rot_control");
  names.push_back("r_rot_control");

  YAML::Node node = YAML::LoadFile("pr2_cart_cart_control.yaml");
  ASSERT_NO_THROW(node.as<giskard::QPControllerSpec>());

  giskard::QPControllerSpec spec = node.as<giskard::QPControllerSpec>();

  for (size_t i=0; i<names.size(); ++i)
    for (size_t j=0; j<names.size(); ++j)
    {
      giskard::SpecPtr spec1, spec2;
      for (size_t k=0; k<spec.scope_.size(); ++k)
      {
        if (spec.scope_[k].name.find(names[i]) == 0)
          spec1 = spec.scope_[k].spec;
        if (spec.scope_[k].name.find(names[j]) == 0)
          spec2 = spec.scope_[k].spec;
      }

      EXPECT_EQ(i==j, spec1->equals(*spec2));
    }
}
