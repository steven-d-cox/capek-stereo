
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
#include "intrinsic.hpp"
#include "string-utils.hpp"

#include "matrix-utils.hpp"

#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

bool find_corners(const vector<string>& image_list,
                  bool draw_corners_and_save,
                  const string& out_dir,
                  uint nx, uint ny, 
                  uint& w, uint& h,
                  vector< vector<Vector2d> >& camera_pts)
{
    bool success = true; // until proven otherwise

    camera_pts.resize(image_list.size());

    cv::Size im_size;
    for(uint i = 0; i < image_list.size(); ++i) {
	auto& filename = image_list[i];

        // Read the file
        cv::Mat grey = imread(filename.c_str(), CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat colour = imread(filename.c_str(), CV_LOAD_IMAGE_COLOR);
        vector<Point2f> corners;

        if(grey.empty() || colour.empty()) {
            fprintf(stderr, "Failed to open image file '%s'\n",
                    filename.c_str());
            success = false;
            continue; 
        }            

	// Record (or check) the size of the input image
	if(i == 0) {
	    im_size = grey.size();
            w = im_size.width;
            h = im_size.height;
        }
	
        if(im_size != grey.size() || im_size != colour.size()) {
	    fprintf(stderr, 
		    "All images must be the same size. The first image "
		    "was size (%d %d), but '%s' was of size (%d %d)!\n",
		    im_size.width, im_size.height,
		    filename.c_str(), grey.size().width, grey.size().height);
            success = false;
	}

        { // find the corners
            bool okay = findChessboardCorners(grey, cv::Size(nx, ny), corners);
            if(!okay) {
                fprintf(stderr,
                        "Could not find chessboard for image '%s', aborting",
                        filename.c_str());
                success = false;
            }
	}

        { // Go subpixel on each corner
            int ws = 5.0;
            auto criteria = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 
                                         100, 0.001);
            cornerSubPix(grey, corners, cv::Size(ws, ws), 
                         cv::Size(2, 2), criteria);
        }

        { // Save in output
            camera_pts[i].reserve(corners.size());
            for(const auto& cc: corners)
                camera_pts[i].emplace_back(double(cc.x), double(cc.y));
        }
	
        if(draw_corners_and_save) { // Save output corners for manual inspection
            drawChessboardCorners(colour, cv::Size(nx, ny), cv::Mat(corners), 
                                  true);
            string outname = make_outname(filename, out_dir, "corners-",".png");
            printf(" + saving: %s\n", outname.c_str());
            imwrite(outname.c_str(), colour);
        }
    }

    return success;
}

// -----------------------------------------------------------------------------

void calculate_world_pts(uint nx, uint ny, double sq_size, 
                         vector<Vector2d>& world_pts)
{
    world_pts.clear();
    world_pts.reserve(nx * ny);
    for(uint i = 0; i < ny; ++i)
        for(uint j = 0; j < nx; ++j)
            world_pts.emplace_back(j*sq_size, i*sq_size); 
}

// -----------------------------------------------------------------------------

bool calibrate_intrinsic(uint nx, uint ny, double sq_size,
                         const vector< vector<Vector2d> >& camera_pts,
                         uint w, uint h, 
                         Matrix3d& K, MatrixXd& D, double& error)
{
    vector<Mat> rvecs, tvecs;
    vector<float> reproj_errs;

    // Calculate the world-pts
    vector<Vector2d> world_pts;
    calculate_world_pts(nx, ny, sq_size, world_pts);

    // Convert from Eigen to opencv...
    // OpenCV wants a copy of cv_world_pts for every set of points in camera_pts
    vector< vector<Point3f> > cv_world_pts(camera_pts.size());
    cv_world_pts[0].resize(world_pts.size());
    for(uint i = 0; i < world_pts.size(); ++i)
        cv_world_pts[0][i] =Point3f(world_pts[i](0), world_pts[i](1), 0.0f);
    for(uint z = 1; z < cv_world_pts.size(); ++z) 
        cv_world_pts[z] = cv_world_pts[0];

    vector< vector<Point2f> > cv_pts(camera_pts.size());
    for(uint z = 0; z < cv_pts.size(); ++z) {
        cv_pts[z].resize(camera_pts[z].size());
        for(uint i = 0; i < cv_pts[z].size(); ++i) 
            cv_pts[z][i] = Point2f(camera_pts[z][i](0), camera_pts[z][i](1));
    }

    // The initial calibration and distortion matrices
    Mat cv_K = Mat::eye(3, 3, CV_64F);
    Mat cv_D = Mat::zeros(8, 1, CV_64F);
    int KD_flags = 0; 

    // OpenCV returns the root-mean-square error in camera calibration
    error = calibrateCamera(cv_world_pts, cv_pts, 
                            Size(w, h),
                            cv_K, cv_D,
                            rvecs, tvecs, KD_flags);

    // Checks that there are no 'nans' or infinities in K or D
    bool ok = checkRange(cv_K) && checkRange(cv_D);

    // From opencv to Eigen
    to_matrix3d(cv_K, K); 
    D = to_matrix(cv_D);    

    return ok;
}

// ------------------------------------------------------------------- Undistort

Vector2d undistort(const Matrix3d& K, 
                   const MatrixXd& D,
                   const Matrix3d& U,
                   const Vector2d& X) 
{
    const double cx = K(0, 2), cy = K(1, 2);
    const double fx = K(0, 0), fy = K(1, 1);
    const double ifx = 1.0 / fx, ify = 1.0 / fy;
    const double k1 = D(0, 0), k2 = D(1, 0), k3 = D(4, 0);
    const double p1 = D(2, 0), p2 = D(3, 0);
    const uint n_itrs = 5; // what opencv uses

    const double x0 = (X(0) - cx) * ifx, y0 = (X(1) - cy) * ify;

    // compensate distortion iteratively
    double x = x0, y = y0;
    for(uint i = 0; i < n_itrs; ++i) {
        double r2 = x*x + y*y;
        double icdist = 1.0 / (1 + ((k3*r2 + k2)*r2 + k1)*r2);
        double deltaX = 2*p1*x*y + p2*(r2 + 2*x*x);
        double deltaY = p1*(r2 + 2*y*y) + 2*p2*x*y;
        x = (x0 - deltaX) * icdist;
        y = (y0 - deltaY) * icdist;
    }

    // place within new camera matrix    
    Vector2d res(U(0,0)*x + U(0,2), U(1,1)*y + U(1,2));

    return res;
}


