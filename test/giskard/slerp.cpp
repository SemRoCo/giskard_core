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

class SlerpTest : public ::testing::Test
{
   protected:
    virtual void SetUp()
    {
      from = KDL::Rotation::Quaternion(0.348, -0.52, 0.616, -0.479);
      to = KDL::Rotation::Quaternion(0.845, 0.262, 0.363, 0.293);
    }

    virtual void TearDown(){}

    KDL::Rotation from, to;
};

TEST_F(SlerpTest, Interpolation)
{
  using namespace KDL;
  EXPECT_TRUE(Equal(giskard::slerp(from, to, 0.1),
        Rotation::Quaternion(0.448726, -0.462604, 0.639831, -0.419085),
        0.001));
  EXPECT_TRUE(Equal(giskard::slerp(from, to, 0.5),
        Rotation::Quaternion(0.757226, -0.163759, 0.621395, -0.118059),
        0.001));
  EXPECT_TRUE(Equal(giskard::slerp(from, to, 0.8),
        Rotation::Quaternion(0.854405, 0.095176, 0.493150, 0.134153),
        0.001));
  EXPECT_TRUE(Equal(giskard::slerp(from, to, 1.0),
        Rotation::Quaternion(0.845000, 0.262000, 0.363000, 0.293000),
        0.001));
  EXPECT_TRUE(Equal(giskard::slerp(to, from, 0.2),
        Rotation::Quaternion(0.854405, 0.095176, 0.493150, 0.134153),
        0.001));
  EXPECT_TRUE(Equal(giskard::slerp(to, from, 0.4),
        Rotation::Quaternion(0.803953, -0.078316, 0.588752, -0.034092),
        0.001));
  EXPECT_TRUE(Equal(giskard::slerp(to, from, 1.0),
        Rotation::Quaternion(0.348000, -0.520000, 0.616000, -0.479000),
        0.001));

//  std::cout << giskard::slerp(from, to, 0.1) << std::endl;
//  std::cout << Rotation::Quaternion(0.448726, -0.462604, 0.639831, -0.419085) << std::endl;

}
