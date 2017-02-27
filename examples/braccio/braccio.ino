/*
   braccio.ino - This file is part of FABRIKAL - Forward And Backward Reaching Inverse Kinematics Arduino Library
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

 #include <Fabrikal.h>

// joints coordinates for arduino.org "Braccio" robotic arm in neutral position.
Point_2d joints[] = {Point_2d(0,72.2), Point_2d(-88.39,160.59), Point_2d(0,248.98), Point_2d(134.35, 114.63)};
double angles[3];
// the first angle offest is the value required by the servo to get a zero angle between the first art and the positive X semi axe. Expressed in radiants.
// the n-th angle offset is the value required by the servo to get a zero angle between n-th art and previous art, expressed in radiants. Expressed in radiants.
double offsets[] = {0,M_PI/2,M_PI/2};

// minimum value for n-th angle of the robotic arm
double lowLim[] = {15 * (M_PI/180), 0, 0};
// maximum value for n-th angle of the robotic arm
double uppLim[] = {165 * (M_PI/180), M_PI, M_PI};

Fabrikal fabrikal(4,joints, angles, offsets, lowLim, uppLim);

void setup()
{
        Serial.begin(9600);

        // solve inverse kinematics for target (300,150)
        if (fabrikal.ik(Point_2d(300,150))) {
                // print angles values in degrees at target position
                for (int i=0; i< fabrikal.numAngles(); i++) {
                        Serial.println(angles[i] * (180 / M_PI));
                }
        } else {
                Serial.println("target is unreachable!");
        }
}

void loop()
{

}
