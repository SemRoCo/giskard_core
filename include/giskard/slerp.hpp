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

#ifndef GISKARD_SLERP_HPP
#define GISKARD_SLERP_HPP

#include <kdl/frames.hpp>
#include <kdl/expressiontree.hpp>

namespace giskard
{
  inline KDL::Rotation slerp(const KDL::Rotation& a, const KDL::Rotation& b,
      double t)
  {
    using namespace std;

    double xa, ya, za, wa, xb, yb, zb, wb;
    a.GetQuaternion(xa, ya, za, wa);
    b.GetQuaternion(xb, yb, zb, wb);
    double cos_half_theta = wa*wb + xa*xb + ya*yb + za*zb;

    if (cos_half_theta < 0)
    {
      wb = -wb;
      xb = -xb;
      yb = -yb;
      zb = -zb;
      cos_half_theta = -cos_half_theta;
    }

    if (abs(cos_half_theta) >= 1.0)
      return a;

    double half_theta = acos(cos_half_theta);
    double sin_half_theta = sqrt(1.0 - cos_half_theta * cos_half_theta);

    if (abs(sin_half_theta) < 0.001)
      return KDL::Rotation::Quaternion(
          0.5*xa + 0.5*xb,
          0.5*ya + 0.5*yb,
          0.5*za + 0.5*zb,
          0.5*wa + 0.5*wb);

    double ratio_a = sin((1.0 -t) * half_theta) / sin_half_theta;
    double ratio_b = sin(t * half_theta) / sin_half_theta;

    return KDL::Rotation::Quaternion(
        ratio_a * xa + ratio_b *xb,
        ratio_a * ya + ratio_b *yb,
        ratio_a * za + ratio_b *zb,
        ratio_a * wa + ratio_b *wb);
  }

}

namespace KDL
{
  class Slerp_Rotation:
      public TernaryExpression<KDL::Rotation, KDL::Rotation, KDL::Rotation, double>
  {
    public:
      typedef TernaryExpression<KDL::Rotation, KDL::Rotation, KDL::Rotation, double> TExpr;
    public:
      Slerp_Rotation() {}  
      Slerp_Rotation( const TExpr::Argument1Expr::Ptr& arg1,
                      const TExpr::Argument2Expr::Ptr& arg2,
                      const TExpr::Argument3Expr::Ptr& arg3) : TExpr("slerp_rotation",arg1,arg2,arg3) {}  
      virtual KDL::Rotation value() 
      {
        return giskard::slerp(argument1->value(), argument2->value(), argument3->value());
      }
      virtual KDL::Vector derivative(int i) 
      {
        /* not yet implemented */
        return KDL::Vector();
      }
      virtual Expression<Vector>::Ptr derivativeExpression(int i) 
      {
        assert( 0 /*not yet implemented */ );
      }
      virtual TExpr::Ptr clone() 
      {
        TExpr::Ptr expr( new Slerp_Rotation( argument1->clone(), argument2->clone(), argument3->clone()));
        return expr;
      }
  };


  inline Expression<KDL::Rotation>::Ptr slerp( Expression<KDL::Rotation>::Ptr a, 
      Expression<KDL::Rotation>::Ptr b, Expression<double>::Ptr c) 
  {
    Expression<KDL::Rotation>::Ptr expr(new Slerp_Rotation(a,b,c));
    return expr;
  }
}

#endif // GISKARD_SLERP_HPP
