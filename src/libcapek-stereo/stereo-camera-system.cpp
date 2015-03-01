
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

#include "stereo-camera-system.hpp"

#include "string-utils.hpp"
#include "math-utils.hpp"
#include "matrix-utils.hpp"
#include "camera-matrix-utils.hpp"

#include "intrinsic.hpp"
#include "surf-point-pairs.hpp"
#include "disparity.hpp"

#define This StereoCameraSystem

using namespace std;
using namespace cv;

// ------------------------------------------------------------- Pre-definitions

static double xEx(const Matrix3d& E, const vector<Vector4d>& corresp)
{
    double err = 0.0;
    uint n = 0;

    for(const auto& xx: corresp) {
        Vector3d l(xx(0), xx(1), 1.0);
        Vector3d r(xx(2), xx(3), 1.0);
        err += fabs(r.transpose() * E * l);
        n++;
    }

    return err / double(n);
}

static double rect_mse(const Matrix3d& T0, const Matrix3d& T1,
                       const vector<Vector4d>& pts)
{
    uint n_pairs = pts.size();
    double mse = 0.0;
    for(const auto& pt: pts) {
        // Transform under homographies
        Vector2d xx = from_hom3(T0 * to_hom3(Vector2d(pt(0), pt(1))));
        Vector2d yy = from_hom3(T1 * to_hom3(Vector2d(pt(2), pt(3))));

        // square error in delta-y
        mse += fabs(xx(1)-yy(1));
    }

    return mse / double(n_pairs);
}

// ---------------------------------------------------------------- Construction

This::This()
{
    init();
}

void This::init()
{
    is_loaded = false;
    camera0_pts.clear();
    camera1_pts.clear();
    nx = ny = square_size = w = h = 0;
    K0 = K1 = Matrix3d::Constant(std::nan(""));
    D0 = D1 = Eigen::Matrix<double, 5, 1>::Constant(std::nan(""));
    U0 = U1 = Matrix3d::Constant(std::nan(""));
    rms_KD0 = rms_KD1 = std::nan("");
    E = Matrix3d::Constant(std::nan(""));
    P0 = P1 = Matrix34d::Constant(std::nan(""));
    T0 = T1 = Matrix3d::Constant(std::nan(""));
}

Vector2d This::undistort0(const Vector2d& x) const
{ 
    return undistort(K0, D0, U0, x);
}

Vector2d This::undistort1(const Vector2d& x) const
{ 
    return undistort(K1, D1, U1, x);
}

void This::undistort_im0(const cv::Mat& in, cv::Mat& out) const
{
    remap(in, out, map1l, map2l, INTER_LINEAR);
}

void This::undistort_im1(const cv::Mat& in, cv::Mat& out) const
{
    remap(in, out, map1r, map2r, INTER_LINEAR);
}

// ------------------------------------------------------------------- Load/Save

static const char * key_camera0_pts = "checkerboard_points_camera_0";
static const char * key_camera1_pts = "checkerboard_points_camera_1";
static const char * key_nx = "checkerboard_nx";
static const char * key_ny = "checkerboard_ny";
static const char * key_square_size = "square_size";
static const char * key_w = "image_width";
static const char * key_h = "image_height";
static const char * key_K0 = "K0";
static const char * key_K1 = "K1";
static const char * key_D0 = "D0";
static const char * key_D1 = "D1";
static const char * key_U0 = "U0";
static const char * key_U1 = "U1";
static const char * key_rms_KD0 = "rms_KD0";
static const char * key_rms_KD1 = "rms_KD1";
static const char * key_E = "E";
static const char * key_P0 = "P0";
static const char * key_P1 = "P1";
static const char * key_T0 = "T0";
static const char * key_T1 = "T1";

bool This::load(const string& filename)
{
    init();

    try {
        FileStorage fs(filename, FileStorage::READ);
        if(!fs.isOpened()) return false;

#define HAS(key) (!(fs[key].empty()))

        // Sanity check
        if((HAS(key_camera0_pts) || HAS(key_camera1_pts)) 
            && !(HAS(key_nx) && HAS(key_ny))){
            fprintf(stderr, "Must have 'nx', 'ny' when loading camera pts\n");
            return false;
        }

        Mat cam0_pts, cam1_pts, cv_K0, cv_K1, cv_D0, cv_D1, cv_U0, cv_U1;
        Mat cv_E;
        if(HAS(key_camera0_pts)) fs[key_camera0_pts] >> cam0_pts;
        if(HAS(key_camera1_pts)) fs[key_camera1_pts] >> cam1_pts;
        if(HAS(key_nx)) fs[key_nx] >> nx;
        if(HAS(key_ny)) fs[key_ny] >> ny;
        if(HAS(key_square_size)) fs[key_square_size] >> square_size;
        if(HAS(key_w)) fs[key_w] >> w;
        if(HAS(key_h)) fs[key_h] >> h;
        if(HAS(key_K0)) { fs[key_K0] >> cv_K0; to_matrix3d(cv_K0, K0); }
        if(HAS(key_K1)) { fs[key_K1] >> cv_K1; to_matrix3d(cv_K1, K1); }
        if(HAS(key_D0)) { fs[key_D0] >> cv_D0; D0 = to_matrix(cv_D0); }
        if(HAS(key_D1)) { fs[key_D1] >> cv_D1; D1 = to_matrix(cv_D1); }
        if(HAS(key_U0)) { fs[key_U0] >> cv_U0; to_matrix3d(cv_U0, U0); }
        if(HAS(key_U1)) { fs[key_U1] >> cv_U1; to_matrix3d(cv_U1, U1); }
        if(HAS(key_rms_KD0)) fs[key_rms_KD0] >> rms_KD0;
        if(HAS(key_rms_KD1)) fs[key_rms_KD1] >> rms_KD1;
        if(HAS(key_E)) { fs[key_E] >> cv_E; to_matrix3d(cv_E, E); }

        // Pack camera0_pts/camera1_pts
        auto set_camera_pts = [] (const Mat& m, 
                                  vector< vector<Vector2d> >& pts,
                                  int nx, int ny) {
            pts.clear();
            int n_per_image = nx * ny;
            int n_images = m.rows / n_per_image;
            if(m.rows % n_per_image)
                throw runtime_error("nx,ny does not agree with camerapts size");
            pts.resize(n_images);
            int row = 0;
            for(int z = 0; z < n_images; ++z) {
                pts[z].resize(n_per_image);
                for(int i = 0; i < n_per_image; ++i) {
                    pts[z][i] = Vector2d(m.at<double>(row, 0), 
                                         m.at<double>(row, 1));
                    row++;
                }
            }
        };
        if(HAS(key_camera0_pts)) set_camera_pts(cam0_pts, camera0_pts, nx, ny);
        if(HAS(key_camera1_pts)) set_camera_pts(cam1_pts, camera1_pts, nx, ny);

#undef HAS

    } catch(...) {
        return false;
    }

    is_loaded = true;
    return is_loaded;
}

bool This::save(const string& filename) const
{
     try {
        FileStorage fs(filename, FileStorage::WRITE);
        if(!fs.isOpened()) return false;

        // Sanity check
        if((has_camera0_pts() || has_camera1_pts()) && !(has_nx() && has_ny())){
            fprintf(stderr, "Must have 'nx', 'ny' before saving camera pts\n");
            return false;
        }

        if(has_camera0_pts()) fs << key_camera0_pts << to_mat_64(camera0_pts);
        if(has_camera1_pts()) fs << key_camera1_pts << to_mat_64(camera1_pts);
        if(has_nx()) fs << key_nx << nx;
        if(has_ny()) fs << key_ny << ny;
        if(has_square_size()) fs << key_square_size << square_size;
        if(has_w()) fs << key_w << w;
        if(has_h()) fs << key_h << h;
        if(has_K0()) fs << key_K0 << to_mat_64(K0);
        if(has_K1()) fs << key_K1 << to_mat_64(K1);
        if(has_D0()) fs << key_D0 << to_mat_64(D0);
        if(has_D1()) fs << key_D1 << to_mat_64(D1);
        if(has_U0()) fs << key_U0 << to_mat_64(U0);
        if(has_U1()) fs << key_U1 << to_mat_64(U1);
        if(has_rms_KD0()) fs << key_rms_KD0 << rms_KD0;
        if(has_rms_KD1()) fs << key_rms_KD1 << rms_KD1;
        if(has_E()) fs << key_E << to_mat_64(E);

    } catch(...) {
        return false;
    }

    return true;
}

// ------------------------------------------------------- Intrinsic Calibration

bool This::calculate_intrinsic_calibration(const Params& p)
{
    bool success = true; 

    // -- (*) -- Initialize stereo as best possible
    if(p.input_filename != "") {
        // -- (*) -- Load from file, if relevant
        printf("Loading '%s'\n", p.input_filename.c_str());
        if(!load(p.input_filename)) {
            fprintf(stderr, "Failed to read input file '%s'\n", 
                    p.input_filename.c_str());
            return false;
        }
    } else {
        nx = p.nx;
        ny = p.ny;
        square_size = p.sq_side; 
    }

    // -- (*) -- Set width and height of input images if possible
    if(success && p.filenames.size() > 0) {
        this->w = p.images[0].cols;
        this->h = p.images[0].rows;
    }

    // -- (*) -- Get corners if that's where we're at
    if(success && (!has_intrinsic() || p.recalc_K)) {
        vector< vector<Vector2d> > all_pts;
        printf("Detecting corners...\n");

        // First a sanity check...
        if(p.filenames.size() == 0) {
            fprintf(stderr, "Must supply calibration images.\n");
            return false;
        }

        if(!p.single_camera && p.filenames.size() % 2 != 0) {
            fprintf(stderr, "Must supply even number of calibration images "
                    "when calibrating two cameras. See --help if you wish "
                    "to calibrate a single camera.\n");
            return false;
        }

        corner0_pts.clear();
        corner1_pts.clear();

        uint iw, ih;
        success = find_corners(p.filenames, p.export_corner_detect,
                               p.output_dir, nx, ny, iw, ih, all_pts);
        assert(int(iw) == w && int(ih) == h);

        for(uint i = 0; i < all_pts.size(); ++i) {
            if(i % 2 == 0) // left
                corner0_pts.push_back(all_pts[i]);
            else
                corner1_pts.push_back(all_pts[i]);
        }
    }

    // -- (*) -- Get intrinsic camera parameters
    if(success && (!has_intrinsic() || p.recalc_K)) {
        printf("About to run intrinsic calibration...\n");
        printf(" + Calculating world points, [%dx%d] board with sq-size %g.\n",
               nx, ny, square_size);

        // Sanity check...
        if(!has_square_size()) {
            fprintf(stderr, "yml file must supply square-size\n");
            return false;
        }

        if(!has_checker_pts()) {
            fprintf(stderr, "No checker points available to do calibration. "
                    "You must either supply a 'yml' input file with such, or "
                    "supply a list of image files.\n");
            return false;
        }

        if(!has_w() || !has_h()) {
            fprintf(stderr, "Internal calibration cannot proceed without "
                    "knowing the dimensions of calibration images. Try "
                    "rerunning corner detection by supplying a list of "
                    "image files.\n");
            return false;
        } 

        if(!p.single_camera && corner0_pts.size() !=  corner1_pts.size()) {
            fprintf(stderr, "ERROR: expect even number of input images "
                    "when calibrating two cameras (!). See help to set "
                    "the 'single camera' switch if calibrating only "
                    "one camera\n");
            return false;
        }

        if(p.single_camera) {
            // combine points to make a single camera
            printf(" + Running intrinsic calibration, single-camera...\n");
            vector< vector<Vector2d> > all_pts = corner0_pts;
            all_pts.insert(all_pts.end(),corner1_pts.begin(),corner1_pts.end());
            calibrate_intrinsic(nx, ny, square_size, all_pts, w, h, 
                                K0, D0, rms_KD0);

            K1 = K0;
            D1 = D0;
            rms_KD1 = rms_KD0;

        } else {
            printf(" + Running intrinsic calibration, two-cameras...\n");

            calibrate_intrinsic(nx, ny, square_size, corner0_pts, 
                                w, h, K0, D0, rms_KD0);
            calibrate_intrinsic(nx, ny, square_size, corner1_pts, 
                                w, h, K1, D1, rms_KD1);
        }

       // -- (*) -- Cannot have any 'nans' in the input
        if(!(is_finite(K0) && is_finite(D0) && is_finite(K1) && is_finite(D1))){
            fprintf(stderr, "Error: infinities, or NANs detected in output\n");
            success = false;
        }

        // -- (*) -- Get undistorted camera matrices...
        if(success) {
            printf(" + Calculating optimal undistorted cameras...\n");

            Size sz(w, h);
            Mat cv_K0 = to_mat_64(K0), cv_D0 = to_mat_64(D0);
            Mat cv_K1 = to_mat_64(K1), cv_D1 = to_mat_64(D1);
            
            // Get the optimal camera matrices for undistorted cameras
            Mat cv_U0 = getOptimalNewCameraMatrix(cv_K0, cv_D0, sz, 1, sz, 0);
            Mat cv_U1 = getOptimalNewCameraMatrix(cv_K1, cv_D1, sz, 1, sz, 0);

            to_matrix_T(cv_U0, U0);
            to_matrix_T(cv_U1, U1);

            success = is_finite(U0) && is_finite(U1);

            if(!success) {
                fprintf(stderr, 
                        "Error: infinities, or NANs detected in undistort\n");
            }
        }

        // -- (*) -- Save output file
        if(success && p.output_filename != "") {
            printf(" + Saving output to '%s'\n", p.output_filename.c_str());
            success = save(p.output_filename);
        }
    }

    // -- (*) -- 
    if(success) {
        if(p.single_camera) {
            printf("\n + RMS error: %g.\n", rms_KD0);
            cout << "K = " << endl << K0 << endl << endl;
            cout << "D = " << endl << D0 << endl << endl;
            cout << "U = " << endl << U0 << endl << endl;
        } else {
            printf("\n + RMS error: %g.\n", rms_KD0);
            cout << "K0 = " << endl << K0 << endl << endl;
            cout << "D0 = " << endl << D0 << endl << endl;
            cout << "U0 = " << endl << U0 << endl << endl;

            printf("\n + RMS error: %g.\n", rms_KD1);
            cout << "K1 = " << endl << K1 << endl << endl;
            cout << "D1 = " << endl << D1 << endl << endl;
            cout << "U1 = " << endl << U1 << endl << endl;
        }
    }

    // -- (*) -- Get undistort maps
    if(success) {
        Size sz(w, h);
        Mat cv_K0 = to_mat_64(K0), cv_D0 = to_mat_64(D0), cv_U0 = to_mat_64(U0);
        Mat cv_K1 = to_mat_64(K1), cv_D1 = to_mat_64(D1), cv_U1 = to_mat_64(U1);

        // Get undistortion-rectifcation maps
        initUndistortRectifyMap(cv_K0, cv_D0, Mat(), cv_U0,
                                sz, CV_16SC2, map1l, map2l);
        initUndistortRectifyMap(cv_K1, cv_D1, Mat(), cv_U1,
                                sz, CV_16SC2, map1r, map2r);
    }

    // -- (*) -- Undistort input images if any are supplied
    if(success && p.export_undistorted) {
        printf(" + Output undistorted images.\n");

        Mat rview;
        int counter = 0;
        for(const auto& filename: p.filenames) {
            const Mat& view = p.images[counter];
            int image_number = counter / 2;
            bool is_left = counter % 2 == 0;
            vector<Vector2d> pts;
            if(is_left) {
                this->undistort_im0(view, rview);
                pts = camera0_pts[image_number];
            } else {
                this->undistort_im1(view, rview);
                pts = camera1_pts[image_number];
            }

            // Draw points onto this image
            for(const auto& pt: pts) {
                auto x = is_left ? undistort0(pt) : undistort1(pt);
                circle(rview, Point2f(x(0),x(1)),1,Scalar(0, 0, 255),1);
            }

            string outname = make_outname(filename, p.output_dir,
                                          "undistorted-", ".png");
            printf(" + %s\n", outname.c_str());
            imwrite(outname, rview);            
            counter++;
        }
    }

    return success;
}

// ---------------------------------------------------------- Stereo Calibration

bool This::surf_corresponding_points(const Params& p)
{
    bool success = true; 

    printf("Running SURF to find corresponding pairs, "
           "min-hessian = %g, distance-ratio = %g\n",
           double(p.surf_min_hessian), p.surf_dist_ratio); 

    if(p.filenames.size() % 2 != 0) {
        fprintf(stderr, "ERROR: must supply even number of input files "
                "when 'surfing' corresponding pairs\n");
        return false;
    }

    // How many pairs of images
    uint n_pairs = p.filenames.size() / 2;

    // Clear everything out
    camera0_pts.clear();
    camera1_pts.clear();
    camera0_pts.resize(n_pairs);
    camera1_pts.resize(n_pairs);
    
    for(uint i = 0; i < n_pairs; ++i) {
        string outfile = "";
        if(p.export_surf) 
            outfile = make_outname(p.filenames[i*2+0], p.output_dir, 
                                   "surf-corners-", ".png");
        surf_corresponding_pairs(p.images[i*2+0],
                                 p.images[i*2+1],
                                 camera0_pts[i],
                                 camera1_pts[i],
                                 p.surf_min_hessian, p.surf_dist_ratio,
                                 outfile);
    }

    // Make sure we have the correct width/height
    {
        assert(w != 0 && h != 0);
    }

    return success;
}

// ---------------------------------------------------------- Stereo Calibration

bool This::calculate_stereo_calibration(const Params& p)
{
    bool success = true;

    if(!has_intrinsic())
        return false; // non-starter

    if(p.run_surf) 
        if(!surf_corresponding_points(p))
            return false;

    if(camera0_pts.size() != camera1_pts.size()) {
        fprintf(stderr, "ERROR: stereo calibration can only proceed "
                "with an even number of images\n");
        return false;
    }

    // Populate the corresponding points
    uint n_pts = camera0_pts.size() * camera0_pts[0].size();
    corresp.clear();
    corresp.reserve(n_pts);
    uint counter = 0;
    for(uint z = 0; z < camera0_pts.size(); ++z) {
        for(uint i = 0; i < camera0_pts[z].size(); ++i) {
            Vector2d u(camera0_pts[z][i](0), camera0_pts[z][i](1));
            Vector2d v(camera1_pts[z][i](0), camera1_pts[z][i](1));
            if(p.apply_undistort) {
                u = undistort0(u);
                v = undistort1(v);
            }
            corresp.emplace_back(u(0), u(1), v(0), v(1));
        }
    }
  
    if(success) { // Apply the 8-point algorithm to get an initial estimate of F
        
        inliers.clear(); // true if a corresp is also an inlier

        if(p.surf_lmeds) {
            printf("Finding inliers with LMedS.\n");
            F = LMEDS_F(all_corresp, inliers);
        } else {
            printf("Finding inliers with RANSAC.\n");
            F = RANSAC_F(all_corresp, inliers);
        }

        corresp.reserve(inliers.size());
        for(uint i = 0; i < inliers.size(); ++i)
            if(inliers[i] == true)
                corresp.push_back(all_corresp[i]);

        // Export
        if(p.export_surf) {
            string outfile = make_outname(p.filenames[0], p.output_dir, 
                                          "surf-inliers-", ".png");
            draw_and_save_corresp(p.images[0], p.images[1],
                                  corresp, outfile);
        }

        printf("Estimate F and E.\n");

        // We want to work with normalized points (for E matrix)
        Matrix3d K0_inv = p.apply_undistort ? U0.inverse() : K0.inverse();
        Matrix3d K1_inv = p.apply_undistort ? U1.inverse() : K1.inverse();
        norm_corresp.clear();
        norm_corresp.reserve(corresp.size());
        for(const auto& xx: corresp) {
            Vector3d n0 = K0_inv * Vector3d(xx(0), xx(1), 1.0);
            Vector3d n1 = K1_inv * Vector3d(xx(2), xx(3), 1.0);
            norm_corresp.emplace_back(n0(0)/n0(2), n0(1)/n0(2),
                                      n1(0)/n1(2), n1(1)/n1(2));
        } 

        E = calc_E(norm_corresp);
        if(p.apply_undistort) 
            F = U1.transpose().inverse() * E * U0.inverse();
        else
            F = K1.transpose().inverse() * E * K0.inverse();
        condition_F(F);
    }

    if(success) { // Fill out P0, P1, r, and s
        // Reference camera '0'
        P0.block(0, 0, 3, 3) = Matrix3d::Identity();
        P0.block(0, 3, 3, 1) = Vector3d::Zero();

        // Normalised P1 camera
        P1 =  P1_from_E(E, norm_corresp);

        // Find 'r' and 's' from P1
        Matrix3d R = P1.block(0, 0, 3, 3);
        rotation_to_spherical_axis_angle(R, r);
        s = unit_axis_to_spherical(P1.block(0, 3, 3, 1));        
    }

    if(success) { // Export information about stereo rig
        // Set up un-normalized cameras
        KP0 = (p.apply_undistort ? U0 : K0) * P0;
        KP1 = (p.apply_undistort ? U1 : K1) * P1;

        // Display the calibration results
        cout << "F =\n" << F << endl << endl;
        cout << "E =\n" << E << endl << endl;

        cout << "e0 = " << from_hom3(null(F)).transpose() << endl;
        cout << "e1 = " << from_hom3(null(F.transpose())).transpose() << endl;
        cout << endl;

        cout << "P0 =\n" << P0 << endl << endl;
        cout << "P1 =\n" << P1 << endl << endl;

        cout << "null(P0) = " << from_hom4(null(P0)).transpose() << endl;
        cout << "null(P1) = " << from_hom4(null(P1)).transpose() << endl;
        cout << endl;

        cout << "KP0 =\n" << KP0 << endl << endl;
        cout << "KP1 =\n" << KP1 << endl << endl;

        cout << "rotation = " 
             << spherical_axis_angle_to_axis_angle(r).transpose() << endl;
        cout << "translat = " << spherical_to_unit_axis(s).transpose() << endl;
        cout << endl;

        printf("xFx = %g\n", xEx(F, corresp));
        printf("xEx = %g\n", xEx(E, norm_corresp));
        cout << endl;

        // Make sure no infinities/NANs crept in...
        success = is_finite(E) && is_finite(KP0) && is_finite(KP1)
            && is_finite(F) && is_finite(s) && is_finite(r);

        if(!success) 
            fprintf(stderr, "Infinities/NANs found in stereo calibration\n");
    }

    if(success && p.export_xEx_costfn) { // export code for generating delta-x 
        const double step = 1.0 * M_PI / 180.0; // degrees
        const double two_pi = 2.0 * M_PI;
        Matrix3d R;
        spherical_axis_angle_to_rotation(r, R);

        auto score_it = [&] (double inc, double azi) {
            Vector3d saa = spherical_to_unit_axis(inc, azi);
            Matrix3d Tx;
            vector3_to_cross_product_matrix(saa, Tx);
            Matrix3d E_new = Tx * R;
            return xEx(E_new, corresp);
        };

        FILE * fp = fopen("/tmp/cf.m", "w");
        
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
    }    

    
  

     
    // vector<Vector3d> pts_3d;
    // pts_3d.reserve(pts_3d_deque.size());
    // pts_3d.insert(pts_3d.end(), pts_3d_deque.begin(), pts_3d_deque.end());
    // glut_display_cloud(pts_3d);
             

    return success;
}

bool This::calculate_rectification(const Params& p)
{
    bool success = true;

    if(!has_stereo_calib())
        if(!calculate_stereo_calibration(p))
            return false;

    printf("Find rectification homographies\n");
    if(find_quadrant(null(F), w, h) == 4 || 
       find_quadrant(null(F.transpose()), w, h) == 4) {
        printf(" + ERROR: epipoles detected in image!\n");
        success = false;
    } else if(p.use_fusiello_rect) {
        Matrix34d R0, R1;
        success = find_rectification(KP0, KP1, w, h, R0, R1, T0, T1);
    } else {            
        success = find_rectification_hz((p.apply_undistort ? U0 : K0),
                                        (p.apply_undistort ? U1 : K1),
                                        KP0, KP1, corresp, T0, T1);
    }        

    if(success) 
        printf(" + rect-err = %g\n", rect_mse(T0, T1, corresp));    

    // What do the rectifications look like?
    if(success && p.export_rectified && p.filenames.size() > 0) { 
        printf("Saving %d rectified images:\n", int(p.filenames.size()));
        Mat rview;            
        int counter = 0;
        for(const auto& filename: p.filenames) {
            const Mat& view = p.images[counter];
            bool is_left = counter++ % 2 == 0;

            if(is_left)
                rview = straight_warp(view, T0, w, h);
            else
                rview = straight_warp(view, T1, w, h);
        
            string outname = make_outname(filename, p.output_dir,
                                          "rectified-", ".png");
            printf(" + %s\n", outname.c_str());
            imwrite(outname, rview);
        }
        printf("\n");
    }

    printf("\n");       

    return success;
}

bool This::calculate_disparity(const Params& p)
{
    if(!has_rectification())
        if(!calculate_rectification(p))
            return false;

    bool success = true;
    string method = "";
    switch(p.disp_method) {
    case DISPARITY_METHOD_BM: method = "bm"; break;
    case DISPARITY_METHOD_SGBM: method = "sgbm"; break;
    default:
        fprintf(stderr, "Unknown disparity method '%d'\n", p.disp_method);
        return false;
    }
    printf("Calculating disparity images using '%s'\n", method.c_str()); 

    printf(" + min_disparity    = %d\n", p.disp_min_disparity);
    printf(" + num_disparities  = %d\n", p.disp_num_disparities);
    printf(" + SAD_window_size  = %d\n", p.disp_SAD_window_size);
    printf(" + 12_max_diff      = %d\n", p.disp_12_max_diff);
    printf(" + prefilter_cap    = %d\n", p.disp_prefilter_cap);
    printf(" + prefilter_size   = %d\n", p.disp_prefilter_size);
    printf(" + texture_threshold= %d\n", p.disp_texture_threshold);
    printf(" + uniqueness_ratio = %d\n", p.disp_uniqueness_ratio);
    printf(" + speckle_window_sz= %d\n", p.disp_speckle_window_size);
    printf(" + speckle_range    = %d\n", p.disp_speckle_range);
    printf(" + min_disparity    = %d\n", p.disp_min_disparity);
    printf(" + full_DP          = %d\n", int(p.disp_full_DP));
    printf(" + P1               = %d\n", p.disp_P1);
    printf(" + P2               = %d\n", p.disp_P1);

    uint n_pairs = p.filenames.size() / 2;
    disparities.resize(n_pairs);

    // How many pairs of images
    for(uint i = 0; i < n_pairs; ++i) {
            
        Mat view0 = straight_warp(p.images[i*2+0], T0, w, h);
        Mat view1 = straight_warp(p.images[i*2+1], T1, w, h);
        Mat& disp = disparities[i];

        switch(p.disp_method) {
        case DISPARITY_METHOD_BM:
            disparity_bm(view0, view1, disp, 
                         p.disp_min_disparity,
                         p.disp_num_disparities, 
                         p.disp_SAD_window_size,
                         p.disp_12_max_diff,
                         p.disp_prefilter_cap,
                         p.disp_prefilter_size,
                         p.disp_texture_threshold,
                         p.disp_uniqueness_ratio,
                         p.disp_speckle_window_size,
                         p.disp_speckle_range);
            break;
        case DISPARITY_METHOD_SGBM:
            disparity_sgbm(view0, view1, disp, 
                           p.disp_min_disparity,
                           p.disp_num_disparities, 
                           p.disp_SAD_window_size,
                           p.disp_12_max_diff,
                           p.disp_prefilter_cap,
                           p.disp_uniqueness_ratio,
                           p.disp_speckle_window_size,
                           p.disp_speckle_range,
                           p.disp_full_DP,
                           p.disp_P1,
                           p.disp_P2);
            break;
        }

        // Sanity checks
        if(disp.type() != CV_32F || disp.depth() != CV_32F) {
            fprintf(stderr, "Expected CV_32F disparity type.\n");
            success = false;
        }

        if(disp.rows != h && disp.cols != w) {
            fprintf(stderr, "Disparity image dimensions wrong.\n");
            success = false;
        }

        if(p.export_disparity) {
            Mat disp8;
            string prefix = string("disparity-") + method + "-";
            string outname = make_outname(p.filenames[i*2+0], p.output_dir,
                                          prefix, ".png");
            normalize(disparities[i], disp8, 0, 255, CV_MINMAX, CV_8U);
            printf(" + %s\n", outname.c_str());
            imwrite(outname.c_str(), disp8);
        }
    }

    printf("\n");
    return success;
}

bool This::calculate_point_cloud(const Params& p, vector<Vector3d>& pts)
{
    bool success = true;

    if(!has_disparity())
        if(!calculate_rectification(p))
            return false;

    deque<Vector3d> pts_3d_deque;

    printf("Calculating point clouds.\n");

    // undistort
    Matrix3d T0_inv = T0.inverse();
    Matrix3d T1_inv = T1.inverse();

    // normalize
    Matrix3d K0_inv = p.apply_undistort ? U0.inverse() : K0.inverse();
    Matrix3d K1_inv = p.apply_undistort ? U0.inverse() : K1.inverse();

    // rotation

    // undistort => normalize => rotate => centre
    Matrix3d A0 = K0_inv * T0_inv;
    Matrix3d A1 = K1_inv * T1_inv;

    for(uint i = 0; i < disparities.size(); ++i) {
        const Mat& disp = disparities[i];

        const Mat& grey0 = p.images[i*2+0];
        const Mat& grey1 = p.images[i*2+1];

        string outname = make_outname(p.filenames[i*2+0], p.output_dir,
                                      "points-", ".text");
        printf(" + Saving '%s'\n", outname.c_str());
        FILE * fp = fopen(outname.c_str(), "w");
        if(fp == NULL) {
            fprintf(stderr, "Failed to open '%s' for writing\n",
                    outname.c_str());
            success = false;
            continue;
        }

        const double min_disparity = p.disp_min_disparity;

        for(int y = 0; y < h; ++y) {
            for(int x = 0; x < w; ++x) {
                double dx = disp.at<float>(y, x);
                if(dx < min_disparity) // out of range
                    continue;

                Vector3d x0(x,      y, 1.0); // left-image co-ordinate
                Vector3d x1(x - dx, y, 1.0); // right-image co-ordinate

                Vector3d X = triangulate(KP0, KP1, 
                                         from_hom3(T0_inv * x0),
                                         from_hom3(T1_inv * x1));

                pts_3d_deque.push_back(X);

                fprintf(fp, "%g %g %g\n", X(0), X(1), X(2));
                    
                if(false) {
                    Vector3d X0 = A0 * x0;
                    Vector3d X1 = A1 * x1;

                    Vector3d corner_0006_l(346, 413, 1);
                    Vector3d XX = T0.inverse() * x0;
                    if(((XX/XX(2)) - corner_0006_l).norm() < 1.5) {
                        printf("----\n");
                        printf("dx = %g\n", dx);
                        printf("---------------------\n");
                        cout << "x0 = " << x0.transpose() << endl;
                        cout << "x1 = " << x1.transpose() << endl;
                        cout << endl;
                        cout << "X0 = " << from_hom3(T0.inverse() * x0).transpose() << endl;
                        cout << "X1 = " << from_hom3(T1.inverse() * x1).transpose() << endl;

                        Vector3d X = triangulate(KP0, KP1, 
                                                 from_hom3(T0.inverse() * x0),
                                                 from_hom3(T1.inverse() * x1));
                        Vector3d z0 = KP0 * to_hom4(X);
                        Vector3d z1 = KP1 * to_hom4(X);

                        cout << "X  = " << X.transpose() << endl;
                        cout << "z0 = " << from_hom3(z0).transpose() << endl;
                        cout << "z1 = " << from_hom3(z1).transpose() << endl;

                        cout << endl;
                        cout << "n0 = " << (X0 / X0(2)).transpose() << endl;
                        cout << "n1 = " << (X1 / X1(2)).transpose() << endl;
                        cout << endl;
                        cout << endl;
                        cout << endl;
                    }
                }
            }
        }

        fclose(fp);
    }

    pts.resize(pts_3d_deque.size());
    auto ii = pts.begin();
    for(const auto& x: pts_3d_deque)
        *ii++ = x;

    return success;
}



