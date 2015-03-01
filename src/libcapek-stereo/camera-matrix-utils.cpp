
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
#include "camera-matrix-utils.hpp"
#include "matrix-utils.hpp"
#include "math-utils.hpp"

#include "nelder-mead.hpp"

#include "intrinsic.hpp"

#include <Eigen/QR>

using namespace std;

void make_camera_matrix(const Matrix3d& R, const Vector3d& t, Matrix34d& P)
{
    P.block(0, 0, 3, 3) = R;
    P.block(0, 3, 3, 1) = t;
}

// Reimplementation of Andrea Fusiello's Epipolar Rectification Toolkit
// Factorise P = K [R | t]
bool factorize_camera_matrix(const Matrix34d& P, // 3x4 camera matrix
                             Matrix3d& K,
                             Matrix3d& R,
                             Vector3d& t,
                             bool fsign) // force sign of focal-length
{
    const bool print_debug = false;
    bool success = true; // until otherwise noted

    MatrixXd s = P.block(0, 3, 3, 1);
    MatrixXd Q = P.block(0, 0, 3, 3).inverse();

    Eigen::HouseholderQR<MatrixXd> qr(Q);
    MatrixXd U = qr.householderQ();
    MatrixXd B = pinv(U) * Q;

    // fix the sign of B(3,3). 
    // This can possibly change the sign of the resulting matrix,
    // which is defined up to a scale factor, however.
    double sig = B(2, 2) > 0.0 ? 1.0 : B(2, 2) < 0.0 ? -1.0 : 0.0;
    B = B * sig;
    s = s * sig;

    // if the sign of the focal lenght is not the required one,
    // change it, and change the rotation accordingly.
    double fsig = fsign ? 1.0 : -1.0;
    if(fsig * B(0, 0) < 0.0) {
        MatrixXd E = MatrixXd::Identity(3, 3);
        E(0, 0) = -1.0;
        B = E*B;
        U = U*E;
    }

    if(fsig * B(1, 1) < 0.0) {
        MatrixXd E = MatrixXd::Identity(3, 3);
        E(1, 1) = -1.0;
        B = E*B;
        U = U*E;
    }

    // if U is not a rotation, fix the sign. This can possibly change the sign
    // of the resulting matrix, which is defined up to a scale factor, however.
    if(U.determinant() < 0.0) {
        U *= -1.0;
        s *= -1.0;
    }

    // sanity check
    if((Q - U*B).norm() > 1e-10 && (Q + U*B).norm() > 1e-10) {
        if(print_debug)
            fprintf(stderr, "Something wrong with the QR factorization: %g\n", 
                    std::min((Q - U*B).norm(), (Q + U*B).norm()));
        success = false;
    }

    R = U.transpose();
    t = B*s;
    K = B.inverse();
    K /= K(2, 2);

    //sanity check
    if(fabs(R.determinant() - 1.0) > 1e-10) {
        if(print_debug)
            fprintf(stderr, "R is not a rotation matrix\n");
        success = false;
    }
    if(K(2, 2) < 0.0) {
        if(print_debug)
            fprintf(stderr, "Wrong sign for K(2, 2)\n");
        success = false;
    }

    // this guarantee that the result *is* a factorization of the given P, 
    // up to a scale factor    
    if(success) {
        MatrixXd Rt(3, 4);
        Rt.block(0, 0, 3, 3) = R;
        Rt.block(0, 3, 3, 1) = t;
        MatrixXd W = K * Rt;
        for(int r = 0; r < P.rows(); ++r)
            for(int c = 0; c < P.cols(); ++c)
                if(fabs(W(r, c) - P(r, c)) > 1e-10) {
                    if(print_debug)
                        fprintf(stderr, "|W(%d %d) - P(%d %d)| = %g\n",
                                r, c, r, c, fabs(W(r, c) - P(r, c)));
                    success = false;
                }
    }

    if(success) {
        if(std::isnan(K.norm()) ||std::isnan(K.norm()) || std::isnan(K.norm())){
            if(print_debug) fprintf(stderr, "Creeping nan in 'K'\n");
            success = false;
        }
    }

    return success;
}

// ------------------------------------------------------------ Essential Matrix

Matrix3d refine_E(const Matrix3d& R0, const Vector3d& t,
                  const vector<Vector4d>& norm_corresp, 
                  uint max_iterations) 
{
    Matrix3d R = R0;
    Vector2d s = unit_axis_to_spherical(t / t.norm());
    Vector3d r;
    rotation_to_spherical_axis_angle(R, r);
    
    // We optimize over five dimensions: 3 rotation + 2 translation
    // Rotation is represented as axis-angle, where the axis is a unit
    // vector in spherical co-ordinates. The translation is also represented
    // as the spherical co-ordinates of a unit vector. The math of the E
    // matrix makes it impossible to know the overall scale of the scene, 
    // and thus we cannot know the amount of translation: just the direction
    
    // Cost function in non-linear optimization
    Matrix3d E_opt; // temporary E matrix    
    auto cost_fn = [&] (double x[]) -> double {
        // Get rotation matrix
        spherical_axis_angle_to_rotation(x[0], x[1], x[2], R);

        // Get Tx
        Matrix3d Tx; // convert 'x' vector to E = Tx * R
        Vector3d saa = spherical_to_unit_axis(x[3], x[4]);
        vector3_to_cross_product_matrix(saa, Tx);            

        E_opt = Tx * R;

        vector3_to_cross_product_matrix(R.transpose() * saa, Tx);
        E_opt = R * Tx;

        // Return the mean error in xEx over the normalised co-ords
        double err = 0.0;
        for(const auto& xx: norm_corresp) {
            Vector3d x0(xx(0), xx(1), 1.0);
            Vector3d x1(xx(2), xx(3), 1.0);
            err += fabs(x1.transpose() * E_opt * x0);
        }
        return err / double(norm_corresp.size());
    };
        
    // Optimization parameters
    const uint n_params = 5;
    double start[n_params] = { r(0), // rotation's axis' inclination
                               r(1), // rotation's axis' azimuth 
                               r(2), // rotation's rotation
                               s(0), // translation's inclination
                               s(1) }; // translation's azimuth

    double xmin[n_params]; // optimal parameters are written here
    double ynewlo; // output variable: evaluation of cost_fn at xmin
    double reqmin = pow(0.1 * M_PI / 180.0, 2); // terimating variance
    const double deg = 5 * M_PI / 180.0; // initial shape of simplex
    double step[n_params] = { deg, deg, deg, deg, deg };
    int konvge = 50; // convergence check after this many iterations
    int kcount = max_iterations; // maximum number of iterations
    int icount = 0; // output variable: number of cost-function evaluations
    int numres = 0; // output variable: number of restarts
    int ifault = 0; // output variable: fault code (0, 1, 2)

    // The "amoeba" optimization routine (does not use derivatives)
    nelder_mead(cost_fn, n_params, start, xmin, 
                &ynewlo, reqmin, step, konvge, kcount,
                &icount, &numres, &ifault);

    // Return our new E
    cost_fn(xmin); // evaluating the cost-function sets 'E_opt'
    return E_opt;
}

Matrix3d calc_E(const vector<Vector4d>& norm_corresp, uint max_iterations) 
{       
    Matrix3d E8 = calc_F_8point(norm_corresp); // Initial guess
    Matrix34d P1 = P1_from_E(E8, norm_corresp);
    
    // Initial rotation and translation
    Matrix3d R = P1.block(0, 0, 3, 3);
    Vector3d t = P1.block(0, 3, 3, 1);

    return refine_E(R, t, norm_corresp, max_iterations);
}

Matrix34d P1_from_E(const Matrix3d& E, const vector<Vector4d>& corresp)
{
    // Hartley and Zisserman, 9.6.2
    Matrix3d U, V;
    svd_UV(E, U, V);

    // Make sure our rotation matrix has determinant > 0
    if((U.determinant() > 0) != (V.determinant() > 0)) 
        for(uint r = 0; r < 3; ++r) // Swap sign of last column of U
            U(r, 2) *= -1.0; // so that results below don't have reflections

    Matrix3d W = Matrix3d::Zero();
    Matrix3d Z = Matrix3d::Zero();
    W(0, 1) = -1.0;        
    W(1, 0) =  1.0;        
    W(2, 2) =  1.0;        
    Z(0, 1) =  1.0;        
    Z(1, 0) = -1.0; 

    // Translation comes from last column of U
    Vector3d t;
    for(uint i = 0; i < 3; ++i)
        t(i) = U(i, 2);

    // Matrix3d S = U * Z * U.transpose();
    Matrix3d R0 = U * W * V.transpose();
    Matrix3d R1 = U * W.transpose() * V.transpose();
   
    // Sign ambiguity in SVD means that there are four possible solutions
    Matrix3d Rs[4] = { R0, R0, R1, R1 };
    Vector3d ts[4] = {  t, -t,  t, -t };
    // Rs[0] = R0; Rs[1] = R0; Rs[2] = R1; Rs[3] = R1;
    // ts[0] = t;  ts[1] = -t; ts[2] = t;  ts[3] = -t;

    // Only one solution should have points in front of both cameras
    // So make cameras
    Matrix34d P0; // reference camera '0'
    Matrix34d Ps[4]; // four posibilities for camera '1'    

    P0 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0; // Canonical camera
    for(uint i = 0; i < 4; ++i) 
        make_camera_matrix(Rs[i], ts[i], Ps[i]); // camera '1' 

    // The correct solution is distinguished by veridical (3d) points
    // being in front of both camera. Here we count up the possibilites
    // and present the results to the user, because noise could cause
    // problems.
    uint counters[4] = { 0, 0, 0, 0 };
    for(const auto& xx: corresp) {
        Vector2d x0(xx(0), xx(1));
        Vector2d x1(xx(2), xx(3));
        for(uint i = 0; i < 4; ++i) {
            auto X = to_hom4(triangulate(P0, Ps[i], x0, x1));
            auto u = P0 * X;
            auto v = Ps[i] * X;
            bool in_front = u(2) > 0.0 && v(2) > 0.0;
            if(in_front) counters[i] += 1;
        }
    }

    // What was the best camera?
    uint best_i = 0;
    for(uint i = 1; i < 4; ++i)
        if(counters[i] > counters[best_i])
            best_i = i;

    // Do we have multiple best cameras?
    bool has_err = false;
    for(uint i = 0; i < 4; ++i)
        if(i != best_i)
            if(counters[i] > 0)
                has_err = true;

    // Just report on the best match found
    if(has_err) {
        fprintf(stderr, "\nWARNING: no unique setup for 'E'\n");
        for(uint i = 0; i < 4; ++i) {
            Vector3d c = from_hom4(null(Ps[i]));
            Vector3d t = ts[i];
            Vector3d saa;
            rotation_to_spherical_axis_angle(Rs[i], saa);
            Vector3d axis = spherical_to_unit_axis(saa(0), saa(1));
            fprintf(stderr, " + [%d] rot-axis = [%g %g %g] theta = %g (%g deg),"
                   " c = [%g %g %g], %d matches%s\n", 
                   i, 
                   axis(0), axis(1), axis(2), 
                   saa(2),
                   saa(2) * 180.0 / M_PI,
                   c(0), c(1), c(2), counters[i],
                   (i == best_i ? " (*)" : ""));
        }
        fprintf(stderr, "\n");
        // throw runtime_error("No unique camera setup for E.");
    }

    return Ps[best_i];
}

void condition_E(Matrix3d& E)
{
    Eigen::JacobiSVD<MatrixXd> svd(E,Eigen::ComputeThinU | Eigen::ComputeThinV);
    MatrixXd U = svd.matrixU();    
    MatrixXd V = svd.matrixV();
    auto s = svd.singularValues();
    MatrixXd D = MatrixXd::Zero(3, 3);
    D(0, 0) = 1.0;
    D(1, 1) = 1.0;
    D(2, 2) = 0.0;

    if((U.determinant() > 0) != (V.determinant() > 0)) 
        for(uint r = 0; r < 3; ++r) // Swap sign of last column of U
            U(r, 2) *= -1.0; // so that results below don't have reflections

    E = U * D * V.transpose();
}

// ---------------------------------------------------------- Fundamental Matrix

Matrix3d calc_F_8point(const vector<Vector4d>& corresp)
{
    const uint n = corresp.size();
    MatrixXd A(n, 9);
    for(uint row = 0; row < n; ++row) {
        const Vector4d& xx = corresp[row];
        A(row, 0) = xx(2) * xx(0); // x'x
        A(row, 1) = xx(2) * xx(1); // x'y
        A(row, 2) = xx(2) * 1.0;   // x'1
        A(row, 3) = xx(3) * xx(0); // y'x
        A(row, 4) = xx(3) * xx(1); // y'y
        A(row, 5) = xx(3) * 1.0;   // y'1
        A(row, 6) = 1.0   * xx(0); // 1 x
        A(row, 7) = 1.0   * xx(1); // 1 y
        A(row, 8) = 1.0   * 1.0;   // 1 1
    }

    // We have Af = 0, use linear-least-squares
    MatrixXd At = A.transpose();
    MatrixXd V = svd_V(At * A);
    Matrix3d F;
    for(uint i = 0; i < 9; ++i)
        F(i/3, i%3) = V(i, 8);

    condition_F(F);
    
    return F;
}

static Matrix3d calc_F_helper(int algorithm,
                              const vector<Vector4d>& corresp,
                              vector<bool>& inliers, 
                              double inlier_err, // 
                              double prob)
{    
    uint n_rows = corresp.size();
    cv::Mat lpts(n_rows, 2, CV_64F);
    cv::Mat rpts(n_rows, 2, CV_64F);
    for(uint r = 0; r < n_rows; ++r) {
        lpts.at<double>(r, 0) = corresp[r](0);
        lpts.at<double>(r, 1) = corresp[r](1);
        rpts.at<double>(r, 0) = corresp[r](2);
        rpts.at<double>(r, 1) = corresp[r](3);
    }

    cv::Mat status(lpts.rows, 1, CV_8U);
    cv::Mat cv_F = findFundamentalMat(lpts, rpts, algorithm, 
                                      inlier_err, prob, status);
    Matrix3d F;
    to_matrix3d(cv_F, F);

    inliers.resize(corresp.size());    
    for(int i = 0; i < status.rows; ++i) 
        inliers[i] = status.at<unsigned char>(i, 0) == 1;

    return F;
}


Matrix3d RANSAC_F(const vector<Vector4d>& corresp,
                  vector<bool>& inliers,    // output vector that gives inliers
                  double inlier_err, // 
                  double prob)
{    
    return calc_F_helper(cv::FM_RANSAC, corresp, inliers, inlier_err, prob);
}

Matrix3d LMEDS_F(const vector<Vector4d>& corresp,
                 vector<bool>& inliers,    // output vector that gives inliers
                 double inlier_err, // 
                 double prob)
{    
    return calc_F_helper(cv::LMEDS, corresp, inliers, inlier_err, prob);
}

Matrix3d fund_from_cameras(const Matrix34d& P1, const Matrix34d& P2)
{
    MatrixXd C1 = null(P1); // Camera centre 1 is null space of P1
    MatrixXd e2 = P2 * C1; // epipole in camera 2

    MatrixXd e2x(3, 3); // skew symmetric matrix from e2
    e2x << 0,    -e2(2,0),  e2(1,0),
        e2(2,0),        0, -e2(0,0),
        -e2(1,0), e2(0,0),        0;

    // Need the right-psuedo inverse of P1;
    MatrixXd P1_pinv = right_pinv(P1);

    // This is F
    MatrixXd F = e2x * P2 * P1_pinv;

    // Ensure that F is rank 2:
    Eigen::JacobiSVD<MatrixXd> svd(F,Eigen::ComputeThinU | Eigen::ComputeThinV);
    MatrixXd U = svd.matrixU();    
    MatrixXd V = svd.matrixV();
    auto s = svd.singularValues();
    MatrixXd D = MatrixXd::Zero(3, 3);
    for(uint i = 0; i < 2; ++i)
        D(i, i) = s(i); // Note the D(2, 2) is zero
    MatrixXd rank_2_F = U * D * V.transpose();
    return Matrix3d(rank_2_F);
}

void condition_F(Matrix3d& F)
{
    Eigen::JacobiSVD<MatrixXd> svd(F,Eigen::ComputeThinU | Eigen::ComputeThinV);
    MatrixXd U = svd.matrixU();    
    MatrixXd V = svd.matrixV();
    auto s = svd.singularValues();
    MatrixXd D = MatrixXd::Zero(3, 3);
    double r = 1.0 / s(0);
    D(0, 0) = r * s(0); // D(0, 0) == 1.0
    D(1, 1) = r * s(1); // D(1, 1) is proportional

    if((U.determinant() > 0) != (V.determinant() > 0)) 
        for(uint r = 0; r < 3; ++r) // Swap sign of last column of U
            U(r, 2) *= -1.0; // so that results below don't have reflections

    F = U * D * V.transpose();
}

// -------------------------------------------------------------------- Epipoles
/**
 * Determine the quadrant for epipole 'e' in image with dimensions 'w'x'h'.
 * The epipole must lie in one of the 9 quadrants below, where region 4 is
 * the image itself.
 *
 * <pre>
 * -------------
 * | 0 | 1 | 2 |
 * |---|---|---|
 * | 3 | 4 | 5 |
 * |---|---|---|
 * | 6 | 7 | 8 |
 * -------------
 * </pre>
 */
uint find_quadrant(Vector3d e, uint w, uint h)
{
    if(e(2) < 0.0) e *= -1.0;     

    bool q036 = e(0) <  0.0;       // quadrant 0, 3, or 6
    bool q258 = e(0) >= w * e(2);  // quadrant 2, 5, or 8
    bool q012 = e(1) <  0.0;
    bool q678 = e(1) >= h * e(2);

    // the corners
    if(q012 && q036) return 0;
    if(q012 && q258) return 2;
    if(q678 && q036) return 6;
    if(q678 && q258) return 8;

    // orthogonal positions
    if(q012) return 1;
    if(q036) return 3;
    if(q258) return 5;
    if(q678) return 7;

    // by elimination
    return 4;
}

// --------------------------------------------------------------- Rectification

// Reimplementation of Andrea Fusiello's Epipolar Rectification Toolkit
static bool find_rectification_helper(const Matrix34d& Po0,const Matrix34d& Po1,
                                      const Vector2d d0, const Vector2d d1,
                                      Matrix34d& Pn0, Matrix34d& Pn1,
                                      Matrix3d& T0, Matrix3d& T1)
{
    bool success = true; // until otherwise noted

    Matrix3d A0, R0;
    Matrix3d A1, R1;
    Vector3d t0, t1;

    if(!factorize_camera_matrix(Po0, A0, R0, t0, true)) success = false;
    if(!factorize_camera_matrix(Po1, A1, R1, t1, true)) success = false;

    // optical centers (unchanged)
    Vector3d c0 = -1.0 * R0.transpose() * A0.inverse() * Po0.block(0, 3, 3, 1);
    Vector3d c1 = -1.0 * R1.transpose() * A1.inverse() * Po1.block(0, 3, 3, 1);
    Vector3d e0 = Po0 * to_hom4(c1);
    Vector3d e1 = Po1 * to_hom4(c0);

    // new x axis (baseline, from c0 to c1)
    Vector3d v1 = (c1 - c0); // Use the epipole image if almost at infinity
    // if(fabs(e0(0)) > 100.0 * e0(2))
    //     v1 = e0; // DANGER

    // new y axes (orthogonal to old z and new x)
    Vector3d v2 = cross(R0.block(2, 0, 1, 3).transpose(), v1);
    // new z axes (no choice, orthogonal to baseline and y)
    Vector3d v3 = cross(v1, v2);

    // new extrinsic (translation unchanged)
    Matrix3d R;
    R.block(0, 0, 1, 3) = v1.transpose() / v1.norm();
    R.block(1, 0, 1, 3) = v2.transpose() / v2.norm();
    R.block(2, 0, 1, 3) = v3.transpose() / v3.norm();

    // new intrinsic (arbitrary) 
    Matrix3d An0 = A1;
    Matrix3d An1 = A1;
    An0(1, 0) = An0(2, 0) = An0(2, 1) = An0(0, 1) = 0.0;
    An1(1, 0) = An1(2, 0) = An1(2, 1) = An1(0, 1) = 0.0;

    // translate image centers 
    An0(0, 2) += d0(0, 0);
    An0(1, 2) += d0(1, 0);
    An1(0, 2) += d1(0, 0);
    An1(1, 2) += d1(1, 0);

    // new projection matrices
    Matrix34d RRc0(3, 4), RRc1(3, 4);
    RRc0.block(0, 0, 3, 3) = R;
    RRc0.block(0, 3, 3, 1) = -1.0 * R * c0;
    RRc1.block(0, 0, 3, 3) = R;
    RRc1.block(0, 3, 3, 1) = -1.0 * R * c1;

    Pn0 = An0 * RRc0;
    Pn1 = An1 * RRc1;
    
    // rectifying image transformation
    T0 = Pn0.block(0, 0, 3, 3) * Po0.block(0, 0, 3, 3).inverse();
    T1 = Pn1.block(0, 0, 3, 3) * Po1.block(0, 0, 3, 3).inverse();

    return success;
}

bool find_rectification(const Matrix34d& Po0, const Matrix34d& Po1,//old cameras
                        double ppt_x, double ppt_y, // principal point
                        Matrix34d& Pn0, Matrix34d& Pn1, // new cameras
                        Matrix3d& T0, Matrix3d& T1) // rect homographies
{
    bool success = true;

    Vector2d d0(0, 0);
    Vector2d d1(0, 0);

    if(!find_rectification_helper(Po0, Po1, d0, d1, Pn0, Pn1, T0, T1))
        success = false;
   
    if(ppt_x > 0 && ppt_y > 0) {
        Vector3d p(ppt_x, ppt_y, 1.0);
        
        Vector3d px0 = T0 * p;
        Vector3d px1 = T1 * p;

        d0(0) = p(0) - px0(0) / px0(2);
        d0(1) = p(1) - px0(1) / px0(2);

        d1(0) = p(0) - px1(0) / px1(2);
        d1(1) = p(1) - px1(1) / px1(2);

        // Force vertical displacement to be the same
        d0(1) = d1(1);

        // Find 'centred' rectification
        if(!find_rectification_helper(Po0, Po1, d0, d1, Pn0, Pn1, T0, T1))
            success = false;
    }   

    return success;
}

bool find_rectification_hz(const Matrix3d& K0, const Matrix3d& K1,
                           const Matrix34d& KP0, const Matrix34d& KP1,
                           const vector<Vector4d>& corresp,
                           Matrix3d& T0, Matrix3d& T1)
{
    bool success = true;

    Vector4d c0 = null(KP0);
    Vector4d c1 = null(KP1);

    // Use normalized co-ordinates, f = 1.0, ppt = 0, 0
    Vector3d e0 = KP0 * c1; //null(F);
    Vector3d e1 = KP1 * c0; //null(F.transpose());
        
    double pptx = K0(0, 2), ppty = K0(1, 2);

    // Homography that translates to ppt
    Matrix3d Tr1 = Matrix3d::Identity();
    Tr1(0, 2) = -pptx;
    Tr1(1, 2) = -ppty;

    Matrix3d R; // Rotate epipole so it lies on x-axis
    double e1x = e1(0) - pptx * e1(2), e1y = e1(1) - ppty * e1(2);
    double theta = -atan2(e1y, e1x);
    R << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;

    Matrix3d G = Matrix3d::Identity(); // Send epipole to infinity
    G(2, 0) = -e1(2) / (cos(theta) * e1x - sin(theta) * e1y);

    Matrix3d Tr2 = Matrix3d::Identity(); // Undo T1 transformation
    Tr2(0, 2) = K0(0, 2);
    Tr2(1, 2) = K0(1, 2);

    T1 = Tr2 * G * R * Tr1;

    // To get T0, we write F = [e1]_x M => M = P1 * P0_pinv
    Matrix3d M = KP1 * pinv(KP0);

    Matrix3d H0 = T1 * M; // for transforming camera0 pts

    // Get points transformed under homographies T1 and H0
    // Now minimize sum(a x0_i + b y0_i + c - x1_i)^2, using least-squares
    MatrixXd A(corresp.size(), 3);
    MatrixXd b(corresp.size(), 1);
    uint row = 0;
    for(auto& xx: corresp) {
        Vector3d x30(xx(0), xx(1), 1.0);
        Vector3d x31(xx(2), xx(3), 1.0);
        Vector2d x0 = from_hom3(H0 * x30);
        Vector2d x1 = from_hom3(T1 * x31);
        A(row, 0) = x0(0);
        A(row, 1) = x0(1);
        A(row, 2) = 1.0;
        b(row, 0) = x1(0);
        row++;
    }

    Vector3d abc = pinv(A) * b;
    Matrix3d HA = Matrix3d::Identity();
    HA(0, 0) = abc(0);
    HA(0, 1) = abc(1);
    HA(0, 2) = abc(2);

    T0 = HA * H0;

    // Sometimes the rectifications can flip the image. If so, this can be
    // detected by examining the ordering of the main diagonal under T0 and T1
    Vector2d p0 = from_hom3(T0 * Vector3d(0, 0, 1));
    Vector2d p1 = from_hom3(T0 * Vector3d(100, 100, 1));
    if(p0(0) > p1(0) && p0(1) > p1(1)) { // rotate 180 degrees
        theta = M_PI;
        R << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;
        Matrix3d rot = Tr2 * R * Tr1;
        T0 = rot * T0;
        T1 = rot * T1;
    }

    return success;
}

// -------------------------------------------------------- Recovering 3d points

Vector3d triangulate_SVD(const Matrix34d& P0, const Matrix34d& P1,
                         Vector2d x0, Vector2d x1)
{
    Matrix4d A(4, 4); // We have Ax = 0

    uint r = 0;
    { // x cross (PX) = 0
        double x = x0(0);
        double y = x0(1);

        for(uint c = 0; c < 4; ++c)
            A(0, c) = x * P0(2, c) - P0(0, c);
        for(uint c = 0; c < 4; ++c)
            A(1, c) = y * P0(2, c) - P0(1, c);
    }
    { // x' cross (P'X) = 0
        double x = x1(0);
        double y = x1(1);

        for(uint c = 0; c < 4; ++c)
            A(2, c) = x * P1(2, c) - P1(0, c);
        for(uint c = 0; c < 4; ++c)
            A(3, c) = y * P1(2, c) - P1(1, c);
    }
    
    // The answer is the "null-space" of A
    MatrixXd V = svd_V(A);
    Vector3d ret(V(0,3)/V(3,3), V(1,3)/V(3,3), V(2,3)/V(3,3));

    return ret;
}

// Finding the null-space directly
Vector3d triangulate(const Matrix34d& P0, const Matrix34d& P1,
                     Vector2d x0, Vector2d x1)
{
    Matrix4d A(4, 4); // We have Ax = 0

    uint r = 0;
    { // x cross (PX) = 0
        double x = x0(0);
        double y = x0(1);

        for(uint c = 0; c < 4; ++c)
            A(0, c) = x * P0(2, c) - P0(0, c);
        for(uint c = 0; c < 4; ++c)
            A(1, c) = y * P0(2, c) - P0(1, c);
    }
    { // x' cross (P'X) = 0
        double x = x1(0);
        double y = x1(1);

        for(uint c = 0; c < 4; ++c)
            A(2, c) = x * P1(2, c) - P1(0, c);
        for(uint c = 0; c < 4; ++c)
            A(3, c) = y * P1(2, c) - P1(1, c);
    }

    // One of the eigenvalues _should_ be close to 0, and its eigenvector
    // is (approx) null(A)... convert to reduced row echelon form, under
    // such error
    
    // The answer is the "null-space" of A
    MatrixXd V = svd_V(A);
    Vector3d svd_ret(V(0,3)/V(3,3), V(1,3)/V(3,3), V(2,3)/V(3,3));

    return svd_ret;
}

// ---------------------------------------------------------------- Homographies

Matrix3d calc_transform_H(uint in_w, uint in_h,
                          const Matrix3d& H1, 
                          const Matrix3d& H2, 
                          uint out_w, uint out_h)
{
    // Find the bounding box for the entire set of images...
    double AABB[4] = { 1e100, 1e100, -1e100, -1e100 };

    auto transform = [] (double x, double y, const Matrix3d& H) {
        return from_hom3(H * Vector3d(x, y, 1.0));
    };

    auto aabb_union = [&] (Vector2d v) {
	if(v(0, 0) < AABB[0]) AABB[0] = v(0, 0);
	if(v(1, 0) < AABB[1]) AABB[1] = v(1, 0);
	if(v(0, 0) > AABB[2]) AABB[2] = v(0, 0);
	if(v(1, 0) > AABB[3]) AABB[3] = v(1, 0);
    };
    aabb_union(transform(0   ,    0, H1));
    aabb_union(transform(in_w,    0, H1));
    aabb_union(transform(in_w, in_h, H1));
    aabb_union(transform(0   , in_h, H1));
    aabb_union(transform(0   ,    0, H2));
    aabb_union(transform(in_w,    0, H2));
    aabb_union(transform(in_w, in_h, H2));
    aabb_union(transform(0   , in_h, H2));

    // Width and height and scale factor
    double w = AABB[2] - AABB[0];
    double h = AABB[3] - AABB[1];

    double scale = std::min(out_w/w, out_h/h);
    double s = scale;

    // printf("[%d %d] => [%d %d]\n", in_w, in_h, out_w, out_h);
    // printf("[%g %g %g %g] -> [%g %g], scale = %g\n",
    //        AABB[0], AABB[1], AABB[2], AABB[3], w, h, s);

    // Translate so that top-left corner is (0, 0)
    Matrix3d T = Matrix3d::Identity(3, 3);
    T<< 1, 0, -AABB[0],
        0, 1, -AABB[1],
        0, 0, 1;

    // Scale so that 
    Matrix3d S = Matrix3d::Identity(3, 3);
    S<< s, 0, 0,
        0, s, 0,
        0, 0, 1;

    return S * T;    
}

cv::Mat straight_warp(const cv::Mat& im, const Matrix3d& H, uint bw, uint bh)
{
    cv::Mat out;
    warpPerspective(im, out, to_mat_64(H), cv::Size(bw, bh));
    return out;
}

// -------------------------------------------------------- Test

bool test_find_E(Vector3d rot_axis, double rot_angle, Vector3d centre)
{
    rot_axis /= rot_axis.norm();
    int nx = 10, ny = 12;
    double square_size = 0.0254; // 1 inch
    vector<Vector2d> world_pts_2d;
    calculate_world_pts(nx, ny, square_size, world_pts_2d);

    vector<Vector4d> world_pts;
    world_pts.reserve(world_pts_2d.size());
    for(const auto& w: world_pts_2d) {
        world_pts.emplace_back(w(0), w(1), 14.0, 1.0);
        world_pts.emplace_back(w(0), w(1), 20.5, 1.0);
    }

    // Camera 0
    Matrix34d P0, P1;
    P0.block(0, 0, 3, 3) = Matrix3d::Identity();    
    P0.block(0, 3, 3, 1) = Vector3d::Zero();

    // Camera 1
    Vector3d c1 = centre;
    Vector2d as = unit_axis_to_spherical(rot_axis);
    double theta = rot_angle;
    Matrix3d R;
    spherical_axis_angle_to_rotation(Vector3d(as(0), as(1), theta), R);       
    P1.block(0, 0, 3, 3) = R;
    P1.block(0, 3, 3, 1) = R * -c1;

    // Create images of world-pts
    vector<Vector2d> im0, im1;
    vector<Vector4d> corresp;
    for(const auto& W: world_pts) {
        Vector2d u = from_hom3(P0 * W);
        Vector2d v = from_hom3(P1 * W);
        im0.push_back(u);
        im1.push_back(v);
        corresp.emplace_back(u(0), u(1), v(0), v(1));
        
        Vector3d X = triangulate(P0, P1, u, v);
        double diff = (from_hom3(from_hom4(W)) - from_hom3(X)).norm();

        if(diff > 1e-9) { // triangulate, does it work?
            cout << "ERROR: W = [" << from_hom4(W).transpose() 
                 << "], X = [" << X.transpose() 
                 << "], diff = " << diff << endl;

            cout << "rot-axis  = " << rot_axis.transpose() << endl;
            cout << "rot-angle = " << rot_angle << endl;
            cout << "centre    = " << centre.transpose() << endl;
            cout << "centre-sp = " << unit_axis_to_spherical(centre).transpose()
                 << endl;

            cout << "P0 = \n" << P0 << endl << endl;
            cout << "P1 = \n" << P1 << endl << endl;

            cout << "u = " << u.transpose() << endl;
            cout << "v = " << v.transpose() << endl;
            cout << "X = " << X.transpose() << endl;
            cout << "W = " << W.transpose() << endl;
        }
    }        
    vector< vector<Vector2d> > camera0_pts, camera1_pts;
    camera0_pts.push_back(im0);
    camera1_pts.push_back(im1);

    // What are the metrics of our setup?
    Vector3d e0 = P0 * to_hom4(c1);
    Vector3d e1 = P1 * Vector4d(0, 0, 0, 1);
    Matrix3d e1x;
    vector3_to_cross_product_matrix(e1, e1x);
    Matrix3d E = e1x * P1 * pinv(P0);
    Matrix3d E8 = calc_E(corresp, 1000);
        
    auto xEx = [&] (const Matrix3d& E, const vector<Vector4d>& corresp) {
        double err = 0.0;

        for(const auto& xx: corresp) {
            Vector3d l(xx(0), xx(1), 1.0);
            Vector3d r(xx(2), xx(3), 1.0);
            err += fabs(r.transpose() * E * l);
        }

        return err / double(corresp.size());
    };

    Matrix34d P1n = P1_from_E(E8, corresp);

    // Did we pass?
    Vector4d aa = rotation_to_axis_angle(P1n.block(0, 0, 3, 3));
    if((aa(3) < 0) != (rot_angle < 0))
        aa *= -1.0;

    double c_err = (from_hom4(null(P1n)) - c1).norm();
    double theta_err = fabs(aa(3) - rot_angle);
    double axis_err = 0.0;
    if(fabs(aa(3)) > 0.01) 
        axis_err = (rot_axis - aa.block(0, 0, 3, 1)).norm();

    bool has_err = c_err + theta_err + axis_err > 1e-3;

    if(has_err) {
        printf("Test ------------------------------------------------------\n");

        printf("c-err     = %g\n", c_err);
        printf("theta-err = %g\n", theta_err);
        printf("axis-err  = %g\n", axis_err);

        cout << " + rot-axis = " << rot_axis.transpose() << ", theta = " 
             << theta << endl;
        cout << " + c1       = " << c1.transpose() <<endl;
        cout << " + t        = " << P1.block(0, 3, 3, 1).transpose() << endl;
        cout << " + plane-nr = " << P1.block(2, 0, 1, 3) << endl;

        cout << " + e0       = " << from_hom3(e0).transpose() << endl;
        cout << " + e1       = " << from_hom3(e1).transpose() << endl;

        cout << " + xE(opt)x = " << xEx(E, corresp) << endl;
        cout << " + xE(est)x = " << xEx(E8, corresp) << endl;
        
        cout << " + null(E)  = " << from_hom3(null(E8)).transpose() << endl;
        cout << " + null(Et) = " << from_hom3(null(E8.transpose())).transpose() 
             << endl;
        cout << " + null(P1) = " << from_hom4(null(P1n)).transpose() <<endl;
        cout << " + P1.t     = " << P1n.block(0, 3, 3, 1).transpose() << endl;
        cout << " + P1.plane = " << P1n.block(2, 0, 1, 3) << endl;
        cout << " + P1.R.aa  = " << aa.transpose() << endl;
        
        printf(" + Done.\n");
        exit(0);
    }

    if(false) {
        double step = 1.0 * M_PI / 180.0; // degrees
        double two_pi = 2.0 * M_PI;
        uint n = two_pi / step;

        auto score_it = [&] (double inc, double azi) {
            Vector3d saa = spherical_to_unit_axis(inc, azi);
            Matrix3d Tx;
            vector3_to_cross_product_matrix(saa, Tx);
            Matrix3d E_new = Tx * R;
            return xEx(E_new, corresp);
        };

        static int counter = 0;
        char buf[1024];
        snprintf(buf, 1024, "/tmp/e_mesh_%02d.m", counter);
        FILE * fp = fopen(buf, "w");

        fprintf(fp, "xx=[");
        for(const auto& xx: corresp)
            fprintf(fp, "%g %g %g %g;", xx(0), xx(1), xx(2), xx(3));
        fprintf(fp, "];\n");

        fprintf(fp, "inc=[");
        for(double inc = 0.0; inc < M_PI; inc += step)
            fprintf(fp, "%g;", inc * 180 / M_PI);
        fprintf(fp, "];\n\n");

        fprintf(fp, "azi=[");
        for(double azi = 0.0; azi < two_pi; azi += step)
            fprintf(fp, "%g;", azi * 180 / M_PI);
        fprintf(fp, "];\n\n");

        fprintf(fp, "scr=[");
        for(double inc = 0.0; inc < M_PI; inc += step) {
            for(double azi = 0.0; azi < two_pi; azi += step)
                fprintf(fp, " %g", score_it(inc, azi));
            fprintf(fp, ";"); // end of 'inc' row
        }
        fprintf(fp, "];\n\n");

        //            fprintf(fp, "mesh(ang, ang, scr);\n\n");
        fprintf(fp, "pkg load image\n");
        fprintf(fp, "figure;\n"
                "surf(azi, inc, log(scr),'EdgeColor', 'None');\n"
                "view(2);\n");
        Vector4d aa = rotation_to_axis_angle(R);
        for(uint i = 0; i < 4; ++i)
            if(fabs(aa(i) < 1e-9))
                aa(i) = 0.0;
        fprintf(fp, "title('xEx for all translations where "
                "axis-angle=[%g %g %g %g]')\n", aa(0), aa(1), aa(2), aa(3));
        fprintf(fp, "xlabel('azimuth of translation vector')\n");
        fprintf(fp, "ylabel('inclination of translation vector')\n");
        fprintf(fp, "axis([0 360 0 180])\n");        
        //fprintf(fp, "print('-djpeg', '/tmp/e-mesh-%02d')\n", counter);
        fclose(fp);

        counter++;
    }

    return !has_err;
}

void test_find_E_comprehensive()
{
    bool success = true;
    double step = 10.0 * M_PI / 180.0; 
    for(double rot = -0.4*M_PI; rot < 0.4*M_PI; rot += step) {
        printf("rot = %g\n", rot);
        for(double tinc = step; tinc < M_PI; tinc += step) {
            printf("   tinc = %g\n", tinc);
            for(double tazz = 0.0; tazz < 2.0*M_PI; tazz += step) {
                Vector3d t = spherical_to_unit_axis(tinc, tazz);
                if(!test_find_E(Vector3d(0, 1, 0), rot, t)) success = false;
                if(!test_find_E(Vector3d(1, 0, 0), rot, t)) success = false;
                if(!test_find_E(Vector3d(0, 0, 1), rot, t)) success = false;
                
                if(!test_find_E(Vector3d(0, 1, 1), rot, t)) success = false;
                if(!test_find_E(Vector3d(1, 0, 1), rot, t)) success = false;
                if(!test_find_E(Vector3d(1, 1, 0), rot, t)) success = false;

                if(!test_find_E(Vector3d(1, 1, 1), rot, t)) success = false;
            }
        }
    }

    if(!success) 
        fprintf(stderr, "At least one testcase failed.\n");
}


