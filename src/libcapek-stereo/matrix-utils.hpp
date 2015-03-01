
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
#include <opencv2/core/core.hpp>

// ----------------------------------------------------------- Matrix Operations

MatrixXd null(const MatrixXd& A);
MatrixXd left_pinv(const MatrixXd& P);
MatrixXd right_pinv(const MatrixXd& P);
MatrixXd pinv(const MatrixXd& P);
MatrixXd svd_V(const MatrixXd& A);
void svd_UV(const Matrix3d& E, Matrix3d& U, Matrix3d& V);
void svd_UDV(const Matrix3d& E, Matrix3d& U, Vector3d& D, Matrix3d& V);
inline Vector3d cross(const Vector3d& u, const Vector3d& v);
void vector3_to_cross_product_matrix(const Vector3d& t, Matrix3d& M);

// --------------------------------- Conerting to/from Eigen and OpenCV matrices

template<typename T> void to_matrix_T(const cv::Mat& m, T& M);
inline void to_matrix3d(const cv::Mat& m, Matrix3d& M);
MatrixXd to_matrix(const cv::Mat& m);
MatrixXd to_matrix(const double * p, int rows, int cols);
cv::Mat to_mat_64(const vector< vector<Vector2d> >& pts);
template<typename T> cv::Mat to_mat_64(const T& P);

// ---------------------------------------------------------- Matrix Information
// True iff a matrix has a NAN value
template<typename T> bool has_nan(const T& M);

// True iff all matrix values are finite
template<typename T> bool is_finite(const T& M);

// Debug function that displays information about a matrix
void display_matrix(const MatrixXd M, const char * name = "");

// ------------------------------------------------------------------- Rotations

void quaternion_to_rotation(const Vector4d& q, Matrix3d& R);
void rotation_to_quaternion(const Matrix3d& R, Vector4d& q);

inline Vector4d rotation_to_quaternion(const Matrix3d& R) 
{
    Vector4d q; rotation_to_quaternion(R, q); return q;
}


// Axis-angle, where the axis is a unit vector expressed in spherical co-ords
void quaternion_to_spherical_axis_angle(const Vector4d& q, Vector3d& a);
void spherical_axis_angle_to_quaternion(const Vector3d& a, Vector4d& q);

void spherical_axis_angle_to_rotation(const Vector3d& a, Matrix3d& R);
void spherical_axis_angle_to_rotation(double inclination, double azimuth,
                                      double rotation, Matrix3d& R);
void rotation_to_spherical_axis_angle(const Matrix3d& R, Vector3d& a);

Vector4d rotation_to_axis_angle(const Matrix3d& R);
Vector4d spherical_axis_angle_to_axis_angle(const Vector3d& r);

Vector3d spherical_to_unit_axis(double inclination, double azimuth);
Vector3d spherical_to_unit_axis(const Vector2d& u);
Vector2d unit_axis_to_spherical(const Vector3d& u);











// ---------------------------------------------------- Template implementations

inline Vector3d cross(const Vector3d& u, const Vector3d& v) 
{
    return u.cross(v);
}

template<typename T> void to_matrix_T(const cv::Mat& m, T& M)
{
    if(M.rows() != m.rows || M.cols() != m.cols) {
        fprintf(stderr, "[%d/%d] => [%d/%d]\n", 
               int(m.rows), int(m.cols), int(M.rows()), int(M.cols()));
        throw std::invalid_argument("row/columns do not match");
    }

    if(m.depth() == CV_64F) {
        for(int r = 0; r < m.rows; ++r)
            for(int c = 0; c < m.cols; ++c)
                M(r, c) = m.at<double>(r, c);
    } else if(m.depth() == CV_32F) {
        for(int r = 0; r < m.rows; ++r)
            for(int c = 0; c < m.cols; ++c) 
                M(r, c) = m.at<float>(r, c);
    } else if(m.depth() == CV_8U) {
        for(int r = 0; r < m.rows; ++r)
            for(int c = 0; c < m.cols; ++c)
                M(r, c) = double(m.at<unsigned char>(r, c)) / 255.0;
    } else if(m.depth() == CV_16S) {
        for(int r = 0; r < m.rows; ++r)
            for(int c = 0; c < m.cols; ++c)
                M(r, c) = double(m.at<unsigned char>(r, c));        
    } else {
        throw std::invalid_argument("bad depth in cv::Mat");
    }
}

inline void to_matrix3d(const cv::Mat& m, Matrix3d& M)
{
    to_matrix_T(m, M);
}

template<typename T> bool has_nan(const T& M)
{
    for(uint r = 0; r < M.rows(); ++r)
        for(uint c = 0; c < M.cols(); ++c)
            if(std::isnan(M(r, c)))
                return true;
    return false;
}

template<typename T> bool is_finite(const T& M)
{    
    for(uint r = 0; r < M.rows(); ++r)
        for(uint c = 0; c < M.cols(); ++c)
            if(!std::isfinite(M(r, c)))
                return false;
    return true;
}

template<typename T> cv::Mat to_mat_64(const T& P)
{
    cv::Mat Q(P.rows(), P.cols(), CV_64F);
    for(int r = 0; r < P.rows(); ++r)
        for(int c = 0; c < P.cols(); ++c)
            Q.at<double>(r, c) = P(r, c);
    return Q;
}

