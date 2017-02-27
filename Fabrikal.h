/*
   Fabrikal.h - This file is part of FABRIKAL - Forward And Backward Reaching Inverse Kinematics Arduino Library
   Arduino implementation of the FABRIK algorithm presented in FABRIK: A fast, iterative solver for the Inverse Kinematics problem by A. Aristidou, J. Lasenby, published in Graphical Models 73 (2011) 243–260
   Copyright (C) 2016  Luca Marchiorello

   FABRIKAL is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   FABRIKAL is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with FABRIKAL.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Fabrikal_h
#define Fabrikal_h

#include "Arduino.h"


class Point_2d {
public:
								Point_2d (double, double);
								Point_2d (const Point_2d&);
								double getX();
								double setX(double);
								double getY();
								double setY(double);
								bool equals(Point_2d);
private:
								double _x;
								double _y;
};

class Fabrikal
{
public:
								Fabrikal (int, Point_2d[],double[]);
								Fabrikal (int, Point_2d[],double[], double[], double[]);
								Fabrikal (int, Point_2d[],double[], double[], double[], double[]);
								int numAngles();
								int numJoints();
								void updateAngles();
								double* getDists();
								double getToll();
								void setToll(double);
								int ik(Point_2d);
private:
								int _joints_num;
								double _toll;
								double _dist_sum;
								Point_2d* _joints;
								double* _angles;
								double* _dist;
								double* _offsets;
								double* _lowLim;
								double* _uppLim;
								double _baseToTarget;
								double _endEffToTarget;
								double _r;
								double _landa;
								double calcAngle(int);
								double posXAngle (double, double, double, double);
								double artsAngle (double, double, double, double, double, double);
								double calcDist(int);
								double dist(Point_2d,Point_2d);
								void scalarProd(double, Point_2d&);
								void vectProd(Point_2d, Point_2d&);
								void vectSum(Point_2d, Point_2d&);
								void vectDiff(Point_2d, Point_2d&);
								void vectRotate(double, Point_2d&);
};

#endif
