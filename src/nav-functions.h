/*
Copyright (c) 2016 - 2020 Regents of the University of Minnesota and Bolder Flight Systems Inc.
MIT License; See LICENSE.md for complete details
Adapted for RAPTRS: Brian Taylor and Chris Regan

Adapted from prior versions
Copyright 2011 Regents of the University of Minnesota. All rights reserved.
Original Author: Adhika Lie, Gokhan Inalhan, Demoz Gebre, Jung Soon Jang

Reference Frames and Coordinates from nav-functions()
I - ECI (Earch Center Inertial): origin at Earth center
E - ECEF (Earch Center Earth Fixed): origin at Earth center
D - Geodetic: origin at Earth center, Uses earth ellisoid definition (example WGS84)
G - Geocentric: origin at Earth center, Uses spheroid definition
NED - Local Level: origin at specified reference, [x- North, y- East, z- Down]
ENU - Local Level: origin at specified reference, [x- East, y- North, z- Up]
B - Body: origin at Body CG, [x- Fwd, y- Starboard, z- Down]

All units meters and radians
"Acceleration" is actually "specific gravity", ie. gravity is removed.
*/

#pragma once

#include <math.h>
#include "eigen.h"

using namespace Eigen;

// Constants
const double EarthRadius = 6378137.0;        // earth semi-major axis radius (m)
const double ECC2 = 0.0066943799901;         // major eccentricity squared

Vector3d NED2D_Rate(Vector3d v_NED, Vector3d pRef_D);
Vector3f NED2D_Rate(Vector3f v_NED, Vector3d pRef_D);

Vector3d NavRate(Vector3d v_NED, Vector3d pRef_D);
Vector3f NavRate(Vector3f v_NED, Vector3d pRef_D);

Vector3d D2E(Vector3d p_D);

Vector3d E2D(Vector3d p_E);

Vector3d E2NED(Vector3d p_E, Vector3d pRef_D);
Matrix3d TransE2NED(Vector3d pRef_D);
Quaterniond TransE2NED_Quat(Vector3d pRef_D);

Matrix3d Skew(Vector3d w);
Matrix3f Skew(Vector3f w);

Vector3f SkewInverse(const Matrix3f &m);
Vector3d SkewInverse(const Matrix3d &m);

Vector3d Quat2Euler(Quaterniond quat);
Vector3f Quat2Euler(Quaternionf quat);

Quaterniond Euler2Quat(Vector3d euler);
Quaternionf Euler2Quat(Vector3f euler);

Matrix3d Quat2DCM(Quaterniond quat);
Matrix3f Quat2DCM(Quaternionf quat);

Matrix3f Euler2DCM(const Vector3f &euler);
Vector3f DCM2Euler(const Matrix3f &dcm);

void EarthRad(double lat, double *Rew, double *Rns);

double WrapToPi(double a);
float WrapToPi(float a);
double WrapTo2Pi(double a);
float WrapTo2Pi(float a);