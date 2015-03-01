 
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
#include "params.hpp"
#include "matrix-utils.hpp"

// Matrices used in calibration
class StereoCameraSystem
{
private:    
    // TRUE when the stereo has been loaded from file
    bool is_loaded; 

    // Any 'chessboard' points stored in file
    vector< vector<Vector2d> > corner0_pts, corner1_pts; // intrinsic calib
    vector< vector<Vector2d> > camera0_pts, camera1_pts; // stereo calib

    // The chessboard parameters, if relevant
    int nx, ny;
    double square_size;

    // The image size used in calibration
    int w, h;

    // Intrinsic camera parameters for each camera
    Matrix3d K0, K1;
    MatrixXd D0, D1; // Distortion co-efficients
    Matrix3d U0, U1; // Undistorted intrinsic parameters

    // Instrinsic root-mean-square calibration error
    double rms_KD0, rms_KD1;

    // Corresponding pairs
    vector<Vector4d> all_corresp;  // before RANSAC
    vector<bool> inliers;
    vector<Vector4d> corresp;      // just inlier corresponding points
    vector<Vector4d> norm_corresp; // ibid., normalized

    Matrix3d E; // essential matrix
    Matrix3d F; // fundamental matrix (to-save)

    Vector3d r; // P1's rotation, relative to P0 (spherical-axis-angle) 
    Vector2d s; // P1's translation, relative to P0 (spherical coords)
    
    Matrix34d P0, P1;   // Normalized camera matrices
    Matrix34d KP0, KP1; // Camera matrices
    //Matrix34d R0, R1;   // Rectification camera matrices   
    Matrix3d T0, T1;    // Rectification homographies

    // These derived values are never saved:
    cv::Mat map1l, map2l, map1r, map2r; // undistort maps

    vector<cv::Mat> disparities; // disparity maps

    void init();

public:
    StereoCameraSystem();

    // Undo distortion given a 2d (or 2d-homogeneous) co-ordinate
    Vector2d undistort0(const Vector2d& x) const;
    Vector2d undistort1(const Vector2d& x) const;
    void undistort_im0(const cv::Mat& in, cv::Mat& out) const;
    void undistort_im1(const cv::Mat& in, cv::Mat& out) const;

    // I/O
    bool load(const string& filename);
    bool save(const string& filename) const;

    // Calibration functions
    bool calculate_intrinsic_calibration(const Params& p);
    bool surf_corresponding_points(const Params& p);
    bool calculate_stereo_calibration(const Params& p);
    bool calculate_rectification(const Params& p);
    bool calculate_disparity(const Params& p);
    bool calculate_point_cloud(const Params& p, vector<Vector3d>& pts);

    // Boolean function say if the given parameter has been set
    bool has_corner0_pts() const { return corner0_pts.size() > 0; }
    bool has_corner1_pts() const { return corner1_pts.size() > 0; }
    bool has_camera0_pts() const { return camera0_pts.size() > 0; }
    bool has_camera1_pts() const { return camera1_pts.size() > 0; }
    bool has_nx() const { return nx > 0; }
    bool has_ny() const { return ny > 0; }
    bool has_square_size() const { return square_size > 0.0; }
    bool has_w() const { return w > 0; }
    bool has_h() const { return h > 0; }
    bool has_K0() const { return !has_nan(K0); }
    bool has_K1() const { return !has_nan(K1); }
    bool has_D0() const { return !has_nan(D0); }
    bool has_D1() const { return !has_nan(D1); }
    bool has_U0() const { return !has_nan(U0); }
    bool has_U1() const { return !has_nan(U1); }
    bool has_rms_KD0() const { return rms_KD0 >= 0; }
    bool has_rms_KD1() const { return rms_KD1 >= 0; }
    bool has_E() const { return !has_nan(E); }
    bool has_P0() const { return !has_nan(P0); }
    bool has_P1() const { return !has_nan(P1); }
    // bool has_R0() const { return !has_nan(R0); }
    // bool has_R1() const { return !has_nan(R1); }
    bool has_T0() const { return !has_nan(T0); }
    bool has_T1() const { return !has_nan(T1); }
        
    bool has_checker_pts() const {return has_corner0_pts() &&has_corner1_pts();}
    bool has_camera_pts() const {return has_camera0_pts() &&has_camera1_pts();}
    bool has_intrinsic() const {
        return has_K0()&&has_K1()&&has_D0()&&has_D1()&&has_U0()&&has_U1(); }
    bool has_stereo_calib() const { return has_E()&&has_P0()&&has_P1(); }
    bool has_rectification() const { return has_T0()&&has_T1();}
    bool has_disparity() const { return disparities.size() > 0; }
};

