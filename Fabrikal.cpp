/*
   Fabrikal.cpp - This file is part of FABRIKAL - Forward And Backward Reaching Inverse Kinematics Arduino Library
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

#include "Arduino.h"
#include "Fabrikal.h"

// Point_2d members
Point_2d::Point_2d(double x, double y){
								_x = x;
								_y = y;
}

Point_2d::Point_2d(const Point_2d& p){
								_x = p._x;
								_y = p._y;
}

double Point_2d::getX(){
								return _x;
}

double Point_2d::setX(double x){
								_x = x;
								return _x;
}

double Point_2d::getY(){
								return _y;
}

double Point_2d::setY(double y){
								_y = y;
								return _y;
}

bool Point_2d::equals (Point_2d p) {
								return ((p.getX() == _x) && (p.getY() == _y));
}

// Fabrik members
Fabrikal::Fabrikal(int joints_num, Point_2d joints[], double angles[] )
{
								_joints_num = joints_num;
								_joints = joints;
								_toll = 0.5; // 0.5 mm
								_offsets = new double[_joints_num -1];
								for (int i=0; i < _joints_num -1; i++) {
																_offsets[i] = 0;
								}
								_lowLim = new double[_joints_num -1];
								for (int i=0; i < _joints_num -1; i++) {
																_lowLim[i] = -M_PI;
								}
								_uppLim = new double[_joints_num -1];
								for (int i=0; i < _joints_num -1; i++) {
																_uppLim[i] = M_PI;
								}
								_angles = angles;
								for (int i=0; i < _joints_num -1; i++) {
																_angles[i] = calcAngle(i);
								}
								_dist = new double[_joints_num -1];
								_dist_sum=0;
								for (int i=0; i < _joints_num -1; i++) {
																_dist[i] = calcDist(i);
																_dist_sum += _dist[i];
								}
}

Fabrikal::Fabrikal(int joints_num, Point_2d joints[], double angles[], double lowLim[], double uppLim[])
{
								_joints_num = joints_num;
								_joints = joints;
								_toll = 0.5; // 0.5 mm
								_offsets = new double[_joints_num -1];
								for (int i=0; i < _joints_num -1; i++) {
																_offsets[i] = 0;
								}
								_lowLim = lowLim;
								_uppLim = uppLim;
								_angles = angles;
								for (int i=0; i < _joints_num -1; i++) {
																_angles[i] = calcAngle(i);
								}
								_dist = new double[_joints_num -1];
								_dist_sum=0;
								for (int i=0; i < _joints_num -1; i++) {
																_dist[i] = calcDist(i);
																_dist_sum += _dist[i];
								}
}

Fabrikal::Fabrikal(int joints_num, Point_2d joints[], double angles[], double offsets[], double lowLim[], double uppLim[])
{
								_joints_num = joints_num;
								_joints = joints;
								_toll = 0.5; // 0.5 mm
								_offsets = offsets;
								_lowLim = lowLim;
								for (int i=0; i < _joints_num -1; i++) {
																_lowLim[i] = lowLim[i] - offsets[i];
								}
								_uppLim = uppLim;
								for (int i=0; i < _joints_num -1; i++) {
																_uppLim[i] = uppLim[i] - offsets[i];
								}
								_angles = angles;
								for (int i=0; i < _joints_num -1; i++) {
																_angles[i] = calcAngle(i);
								}
								_dist = new double[_joints_num -1];
								_dist_sum=0;
								for (int i=0; i < _joints_num -1; i++) {
																_dist[i] = calcDist(i);
																_dist_sum += _dist[i];
								}
}

// return the angle with respect to the positive X axe
double Fabrikal::posXAngle (double fromX, double fromY, double toX, double toY){
								double angle;
								double deltaX = toX - fromX;
								double deltaY = toY - fromY;
								if (deltaX < 0 && deltaY== 0) angle = M_PI;
								else angle =
																								M_PI*(deltaY<0) +
																								M_PI*((deltaY/deltaX)<0) +
																								atan(deltaY/deltaX);
								return angle;
}

// return the angle with respect to the previous braccio ranging from -180° to 180°
double Fabrikal::artsAngle(double prev_angle, double fromX, double fromY, double toX, double toY, double offset){
								double angle = posXAngle(fromX, fromY, toX, toY);
								angle = angle - prev_angle;
								if (angle > M_PI) return -2*M_PI + angle;
								else if (angle < -M_PI) return 2*M_PI + angle;
								else return angle;
}

// first angle is calculated with respect to the positive X axe. Next angles are calculated with respect to the previous angle.
double Fabrikal::calcAngle(int i){
								if (i == 0) {
																return artsAngle(0, _joints[0].getX(), _joints[0].getY(), _joints[1].getX(), _joints[1].getY(), _offsets[0]);
								}
								else {
																double prev_angle = posXAngle(_joints[i-1].getX(), _joints[i-1].getY(), _joints[i].getX(), _joints[i].getY());
																return artsAngle(prev_angle, _joints[i].getX(), _joints[i].getY(), _joints[i+1].getX(), _joints[i+1].getY(), _offsets[i]);
								}
}

void Fabrikal::updateAngles(){
								for (int i=0; i < numAngles(); i++) {
																_angles[i] = calcAngle(i) + _offsets[i];
								}
}

int Fabrikal::numAngles(){
								return _joints_num - 1;
}

int Fabrikal::numJoints(){
								return _joints_num;
}

double Fabrikal::getToll(){
								return _toll;
}

void Fabrikal::setToll(double toll){
								_toll = toll;
}

double Fabrikal::calcDist(int i){
								return dist(_joints[i],_joints[i+1]);
}

double Fabrikal::dist(Point_2d a,Point_2d b){
								return sqrt(square (abs(a.getY() - b.getY())) + square (abs(a.getX() - b.getX())));
}

double* Fabrikal::getDists(){
								return _dist;
}

void Fabrikal::scalarProd(double landa, Point_2d& p){
								p.setX(landa * p.getX());
								p.setY(landa * p.getY());
}

void Fabrikal::vectProd(Point_2d a, Point_2d& b){
								b.setX(a.getX() * b.getX());
								b.setY(a.getY() * b.getY());
}

void Fabrikal::vectSum(Point_2d a, Point_2d& b){
								b.setX(a.getX() + b.getX());
								b.setY(a.getY() + b.getY());
}

void Fabrikal::vectDiff(Point_2d a, Point_2d& b){
								b.setX(a.getX() - b.getX());
								b.setY(a.getY() - b.getY());
}

void Fabrikal::vectRotate(double alpha, Point_2d& p){
								double x = p.getX() * cos(alpha) - p.getY() * sin(alpha);
								double y = p.getX() * sin(alpha) + p.getY() * cos(alpha);
								p.setX(x);
								p.setY(y);
}

int Fabrikal::ik(Point_2d target){
								Point_2d p = Point_2d(0,0);
								Point_2d q = Point_2d(0,0);
								Point_2d old = Point_2d(_joints[0]);
								bool reversed = false;
								bool reset = false;
								//the distance between root and target
								_baseToTarget =  dist(target, _joints[0]);
								if (_baseToTarget > _dist_sum) return 0;
								// The target is reachable; thus, set as b the initial position of the joint p1
								Point_2d base = Point_2d(_joints[0]);
								_endEffToTarget = dist(target, _joints[numJoints()-1]);
								while ( _endEffToTarget > _toll) {
																if (reset) {
																								_joints[0].setX(base.getX());
																								_joints[0].setY(base.getY());
																								for (int i=1; i<numJoints(); i++) {
																																_joints[i].setX(0);
																																_joints[i].setY(_dist[i-1] + _joints[i-1].getY());
																								}
																								reversed = true;
																								reset = false;
																}
																//  // STAGE 1: FORWARD REACHING
																//  // set the end effector Pn as target t
																_joints[numJoints()-1].setX(target.getX());
																_joints[numJoints()-1].setY(target.getY());
																for (int i=numJoints()-2; i>=0; i--) {
																								// Find the distance ri between the new joint position pi+1 and the joint pi
																								_r = dist(_joints[i+1], _joints[i]);
																								_landa = _dist[i]/_r;
																								// Find the new joint positions pi
																								p = Point_2d(_joints[i+1]);
																								scalarProd(1-_landa, p);
																								scalarProd(_landa, _joints[i]);
																								vectSum(p, _joints[i]);

																								// apply constraints
																								if (i < numAngles()-1) {
																																_angles[i+1] = calcAngle(i+1);
																																if (_angles[i+1] < _lowLim[i+1]) {
																																								p = Point_2d(_joints[i+1]);
																																								vectDiff(_joints[i], p);
																																								q = Point_2d (p);
																																								vectRotate(_angles[i+1] - _lowLim[i+1], q);
																																								vectDiff(q,p);
																																								vectSum(p, _joints[i]);
																																								_angles[i+1]=_lowLim[i+1];
																																}
																																if (_angles[i+1] > _uppLim[i+1]) {
																																								p = Point_2d(_joints[i+1]);
																																								vectDiff(_joints[i], p);
																																								q = Point_2d (p);
																																								vectRotate(_angles[i+1] - _uppLim[i+1], q);
																																								vectDiff(q,p);
																																								vectSum(p, _joints[i]);
																																								_angles[i+1]=_lowLim[i+1];
																																}
																								}
																}
																// if the algorithm is bouncing we try to reach the target from the other side
																if (old.equals(_joints[0]) && !old.equals(base)) {
																								// if already tryed the other side the target is unreachable
																								if (reversed) return 0;
																								// try reverse side
																								else {
																																reset = true;
																								}
																}
																// STAGE 2: BACKWARD REACHING
																// Set the root p1 its initial position.
																old = Point_2d(_joints[0]);
																_joints[0].setX(base.getX());
																_joints[0].setY(base.getY());
																for (int i=0; i<numJoints()-1; i++) {
																								// Find the distance ri between the new joint position pi+1 and the joint pi
																								_r = dist(_joints[i+1], _joints[i]);
																								_landa = _dist[i]/_r;
																								// Find the new joint positions pi
																								p = Point_2d(_joints[i]);
																								scalarProd(1-_landa, p); // a
																								scalarProd(_landa, _joints[i+1]);
																								vectSum(p, _joints[i+1]);

																								// apply constraints
																								_angles[i] = calcAngle(i);
																								if (_angles[i] < _lowLim[i]) {
																																p = Point_2d(_joints[i]);
																																vectDiff(_joints[i+1], p);
																																vectRotate(_lowLim[i] - _angles[i], p);
																																vectSum(_joints[i], p);
																																_joints[i+1].setX(p.getX());
																																_joints[i+1].setY(p.getY());
																																_angles[i]=_lowLim[i];
																								}
																								if (_angles[i] > _uppLim[i]) {
																																p = Point_2d(_joints[i]);
																																vectDiff(_joints[i+1], p);
																																vectRotate( _uppLim[i] - _angles[i], p);
																																vectSum( _joints[i], p);
																																_joints[i+1].setX(p.getX());
																																_joints[i+1].setY(p.getY());
																																_angles[i]=_lowLim[i];
																								}
																}
																_endEffToTarget = dist(target, _joints[numJoints()-1]);
								}
								updateAngles();
								return 1;
}
