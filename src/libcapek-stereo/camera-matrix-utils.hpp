
#pragma once

// Copyright (c) 2015, Aaron Michaux
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, 
//    this list of conditions and the following disclaimer. 
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// The views and conclusions contained in the software and documentation are
// those of the authors and should not be interpreted as representing official 
// policies, either expressed or implied, of the FreeBSD Project.

#include "stdinc.hpp"

// ------------------------------------------------------------- Camera Matrices

// Create a new camera matrix from the given rotation and translation
void make_camera_matrix(const Matrix3d& R, const Vector3d& t, Matrix34d& P);

// Factorise a camera matrix in K [R | t]. 
// R must be invertable (meaning we have a finite camera)
bool factorize_camera_matrix(const Matrix34d& P, // 3x4 camera matrix
                             Matrix3d& K,
                             Matrix3d& R,
                             Vector3d& t,
                             bool fsign); // force sign of focal-length

// ------------------------------------------------------------ Essential Matrix

// Find 'E' from normalized points
Matrix3d calc_E(const vector<Vector4d>& norm_corresp,
                uint max_iterations=2000);

Matrix34d P1_from_E(const Matrix3d& E, const vector<Vector4d>& corresp);

// Make sure E has singular values: 1 1 0
void condition_E(Matrix3d& E);

// ---------------------------------------------------------- Fundamental Matrix

// Calculate F using the 8-point algorithm
Matrix3d calc_F_8point(const vector<Vector4d>& corresp);

Matrix3d RANSAC_F(const vector<Vector4d>& norm_corresp,
                  vector<bool>& inliers,    // output vector that gives inliers
                  double inlier_err = 2.5, // 
                  double prob = 0.99999);

Matrix3d LMEDS_F(const vector<Vector4d>& corresp,
                 vector<bool>& inliers,    // output vector that gives inliers
                 double inlier_err = 2.5, // 
                 double prob = 0.99999);

// Calculate the fundamental matrix from two camera matrices
Matrix3d fund_from_cameras(const Matrix34d& P1, const Matrix34d& P2);

// Make sure F is rank two by setting last eigen-vector to zero
void condition_F(Matrix3d& F);

// -------------------------------------------------------------------- Epipoles

uint find_quadrant(Vector3d e, uint w, uint h);

// --------------------------------------------------------------- Rectification

// Find rectification following Andrea Fusiello's Epipolar Rectification Toolkit
// This wil FAIL if the epipoles are inside (or close to) either image
bool find_rectification(const Matrix34d& Po0, const Matrix34d& Po1,//old cameras
                        double ppt_x, double ppt_y, // principal point
                        Matrix34d& Pn0, Matrix34d& Pn1, // new cameras
                        Matrix3d& T0, Matrix3d& T1); // rect homographies

bool find_rectification_hz(const Matrix3d& K0, const Matrix3d& K1,
                           const Matrix34d& KP0, const Matrix34d& KP1,
                           const vector<Vector4d>& corresp,
                           Matrix3d& T0, Matrix3d& T1);

// -------------------------------------------------------- Recovering 3d points

// Triangulate method involves an SVD (finding a null space of a matrix)
Vector3d triangulate_SVD(const Matrix34d& P0, const Matrix34d& P1,
                     Vector2d x0, Vector2d x1);

// Finding the null-space directly
Vector3d triangulate(const Matrix34d& P0, const Matrix34d& P1,
                     Vector2d x0, Vector2d x1);

// ---------------------------------------------------------------- Homographies

Matrix3d calc_transform_H(uint in_w, uint in_h,
                          const Matrix3d& H1, 
                          const Matrix3d& H2, 
                          uint out_w, uint out_h);

cv::Mat straight_warpX(const cv::Mat& im, const MatrixXd& H, uint bw, uint bh);
cv::Mat straight_warp(const cv::Mat& im, const Matrix3d& H, uint bw, uint bh);

// --------------------------------------------------------------- Testing stack

bool test_find_E(Vector3d rot_axis, double rot_angle, Vector3d centre);

// Run 'test-find-E', but over a wide array of angles and centres
void test_find_E_comprehensive();


