
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
#include "matrix-utils.hpp"
#include "math-utils.hpp"

#include <opencv2/core/core.hpp>

using namespace cv;
using namespace std;

// ----------------------------------------------------------- Matrix Operations

MatrixXd null(const MatrixXd& A)
{
    Eigen::FullPivLU<MatrixXd> lu_decomp(A);
    MatrixXd ret = lu_decomp.kernel();
    return ret;
}

MatrixXd left_pinv(const MatrixXd& P)
{
    MatrixXd Pt = P.transpose();
    MatrixXd PtP = Pt * P;
    MatrixXd PtP_inv = PtP.inverse();
    return PtP_inv * Pt;
}

MatrixXd right_pinv(const MatrixXd& P)
{
    MatrixXd Pt = P.transpose();
    MatrixXd PPt = P * Pt;
    MatrixXd PPt_inv = PPt.inverse();
    return Pt * PPt_inv;
}

MatrixXd pinv(const MatrixXd& P)
{
    if(P.rows() == P.cols())
        return P.inverse();
    if(P.rows() > P.cols())
        return left_pinv(P);
    return right_pinv(P);
}

MatrixXd svd_V(const MatrixXd& A)
{
    Eigen::JacobiSVD<MatrixXd> svd(A, Eigen::ComputeThinV);
    return svd.matrixV();
}

void svd_UV(const Matrix3d& E, Matrix3d& U, Matrix3d& V)
{
    Eigen::JacobiSVD<MatrixXd> svd(E,Eigen::ComputeThinU | Eigen::ComputeThinV);
    U = svd.matrixU();    
    V = svd.matrixV();
}

void svd_UDV(const Matrix3d& E, Matrix3d& U, Vector3d& D, Matrix3d& V)
{
    Eigen::JacobiSVD<MatrixXd> svd(E,Eigen::ComputeThinU | Eigen::ComputeThinV);
    U = svd.matrixU();    
    V = svd.matrixV();
    auto s = svd.singularValues();
    for(uint i = 0; i < 3; ++i)
        D(i) = s(i);
}

void vector3_to_cross_product_matrix(const Vector3d& t, Matrix3d& M)
{
    M << 0, -t(2), t(1), 
        t(2), 0 , -t(0),
        -t(1), t(0), 0;
}

// --------------------------------- Conerting to/from Eigen and OpenCV matrices

MatrixXd to_matrix(const cv::Mat& m)
{
    MatrixXd M(m.rows, m.cols);
    to_matrix_T(m, M);
    return M;    
}

MatrixXd to_matrix(const double * p, int rows, int cols)
{
    MatrixXd P(rows, cols);
    for(int r = 0; r < rows; ++r)
        for(int c = 0; c < cols; ++c)
            P(r, c) = p[r*cols + c];
    return P;
}

Mat to_mat_64(const vector< vector<Vector2d> >& pts)
{
    if(pts.size() == 0)
        throw invalid_argument("pts.size() must be > 0");
    if(pts[0].size() == 0)
        throw invalid_argument("invalid size");

    int n_rows = pts.size() * pts[0].size();
    int n_cols = 2;
    Mat m(n_rows, n_cols, CV_64F);
    int row = 0;
    for(const auto& im_pts: pts) {
        for(const auto& vec: im_pts) {
            m.at<double>(row, 0) = vec(0);
            m.at<double>(row, 1) = vec(1);
            ++row;
        }
    }

    if(row != n_rows)
        throw logic_error("bad dimensions in 'pts' somewhere");

    return m;
}

// ---------------------------------------------------------- Matrix Information

void display_matrix(const MatrixXd M, const char * name)
{
    if(strcmp(name, "") == 0) name = "matrix";
    cout << name << "--------------------------------------------------------\n"
         << M << endl << endl;
    printf("det(M) = %g\n", M.determinant());
    
    MatrixXd U, V;
    Eigen::JacobiSVD<MatrixXd> svd(M,Eigen::ComputeThinU | Eigen::ComputeThinV);
    U = svd.matrixU();    
    V = svd.matrixV();
    auto D = svd.singularValues();
    
    cout << "D = " << D.transpose() << endl;

    cout << "U = (det =" << U.determinant() << ")\n" << U << endl << endl;
    cout << "V = (det =" << V.determinant() << ")\n" << V << endl << endl;

    cout << "inv =\n" << M.inverse() << endl << endl;
}

// ------------------------------------------------------------------- Rotations

void quaternion_to_rotation(const Vector4d& q, Matrix3d& R)
{
    double x = q(0), y = q(1), z = q(2), w = q(3);
    double n = w * w + x * x + y * y + z * z;
    double s = (n == 0) ? 0.0 : 2.0 / n;
    
    double wx = s * w * x, wy = s * w * y, wz = s * w * z;
    double xx = s * x * x, xy = s * x * y, xz = s * x * z;
    double yy = s * y * y, yz = s * y * z, zz = s * z * z;

    R << 1 - (yy + zz),       xy - wz,         xz + wy,
        xy + wz,        1 - (xx + zz),         yz - wx,
        xz - wy,              yz + wx,   1 - (xx + yy);
}

void rotation_to_quaternion(const Matrix3d& R, Vector4d& q)
{
    double trace = R(0,0) + R(1,1) + R(2,2);

    if(trace > 0) {
        double s = 0.5 / sqrt(trace + 1.0);
        q(3) = 0.25 / s;
        q(0) = (R(2,1) - R(1,2)) * s;
        q(1) = (R(0,2) - R(2,0)) * s;
        q(2) = (R(1,0) - R(0,1)) * s;
    } else if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
        double s = 2.0 * sqrt(1.0 + R(0,0) - R(1,1) - R(2,2));
        q(3) = (R(2,1) - R(1,2)) / s;
        q(0) = 0.25 * s;
        q(1) = (R(0,1) + R(1,0)) / s;
        q(2) = (R(0,2) + R(2,0)) / s;
    } else if (R(1,1) > R(2,2)) {
        double s = 2.0 * sqrt(1.0 + R(1,1) - R(0,0) - R(2,2));
        q(3) = (R(0,2) - R(2,0)) / s;
        q(0) = (R(0,1) + R(1,0)) / s;
        q(1) = 0.25 * s;
        q(2) = (R(1,2) + R(2,1)) / s;
    } else {
        double s = 2.0 * sqrt(1.0 + R(2,2) - R(0,0) - R(1,1));
        q(3) = (R(1,0) - R(0,1)) / s;
        q(0) = (R(0,2) + R(2,0)) / s;
        q(1) = (R(1,2) + R(2,1)) / s;
        q(2) = 0.25 * s;
    }

    q /= q.norm();
}

void quaternion_to_spherical_axis_angle(const Vector4d& q, Vector3d& a)
{
    if(fabs(q(3) - 1.0) < 1e-9) {
        a = Vector3d::Zero();
    } else {
        double theta = 2.0 * acos(q(3)); // angle of rotation
        double s = 1.0 / sqrt(1.0 - q(3)*q(3));
        double x = s * q(0), y = s * q(1), z = s * q(2);
       
        a(0) = acos(z / sqrt(x*x + y*y + z*z)); // inclination
        a(1) = atan2(y, x);                     // azimuth
        a(2) = normalise_angle(theta);          // angle of rotation
    }
}

void spherical_axis_angle_to_quaternion(const Vector3d& a, Vector4d& q)
{
    // a(inclination, azimuth, amount-of-rotation)
    double half_theta = 0.5 * a(2);
    double sin_half_theta = sin(half_theta);
    q(0) = sin(a(0)) * cos(a(1)) * sin_half_theta; 
    q(1) = sin(a(0)) * sin(a(1)) * sin_half_theta;
    q(2) = cos(a(0)) * sin_half_theta;
    q(3) = cos(half_theta);
}

void spherical_axis_angle_to_rotation(const Vector3d& a, Matrix3d& R)
{
    Vector4d q;
    spherical_axis_angle_to_quaternion(a, q);
    quaternion_to_rotation(q, R);
}

void spherical_axis_angle_to_rotation(double inclination, double azimuth,
                                      double rotation, Matrix3d& R)
{
    return spherical_axis_angle_to_rotation(Vector3d(inclination, 
                                                     azimuth,
                                                     rotation), 
                                            R);
}

void rotation_to_spherical_axis_angle(const Matrix3d& R, Vector3d& a)
{
    Vector4d q;
    rotation_to_quaternion(R, q);
    quaternion_to_spherical_axis_angle(q, a);
}

Vector4d rotation_to_axis_angle(const Matrix3d& R)
{
    Vector3d s;
    rotation_to_spherical_axis_angle(R, s);
    Vector3d axis = spherical_to_unit_axis(s(0), s(1));
    return Vector4d(axis(0), axis(1), axis(2), s(2));
}


Vector4d spherical_axis_angle_to_axis_angle(const Vector3d& r)
{
    Vector3d axis = spherical_to_unit_axis(r(0), r(1));
    return Vector4d(axis(0), axis(1), axis(2), r(2));
}

Vector3d spherical_to_unit_axis(double inclination, double azimuth)
{
    return Vector3d(sin(inclination) * cos(azimuth),
                    sin(inclination) * sin(azimuth),
                    cos(inclination));
}

Vector3d spherical_to_unit_axis(const Vector2d& u)
{
    return spherical_to_unit_axis(u(0), u(1));
}

Vector2d unit_axis_to_spherical(const Vector3d& u)
{
    double x = u(0), y = u(1), z = u(2);
    double r = sqrt(x*x + y*y + z*z);
    return Vector2d(acos(z/r), atan2(y, x));
}

