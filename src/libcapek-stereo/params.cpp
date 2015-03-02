 
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
#include "string-utils.hpp"

using namespace std;

Params::Params() : 
    show_help(false), 
    okay(true),

    input_filename(""),
    output_filename("camera-calib.yml"),

    filenames(),

    recalc_K(false),
    recalc_F(false),

    single_camera(false),
    apply_undistort(false),

    nx(9),
    ny(6), 
    sq_side(0.023571429),

    run_surf(true), // in theory we could get points another way
    surf_min_hessian(100),
    surf_dist_ratio(30),
    surf_lmeds(false),

    use_fusiello_rect(false),

    disp_method(DISPARITY_METHOD_BM),
    disp_min_disparity(-39),
    disp_num_disparities(112),
    disp_SAD_window_size(9), 
    disp_12_max_diff(1),
    disp_prefilter_cap(61), 
    disp_prefilter_size(5), 
    disp_texture_threshold(507),
    disp_uniqueness_ratio(0), 
    disp_speckle_window_size(0), 
    disp_speckle_range(9), 
    disp_full_DP(false),
    disp_P1(0),
    disp_P2(0),

    elas_params(Elas::MIDDLEBURY),

    output_dir("/tmp"),
    export_corner_detect(true), // only default TRUE
    export_undistorted(false),
    export_rectified(false),
    export_surf(false),
    export_xEx_costfn(false),
    export_disparity(false),

    show_point_cloud(false),

    test_find_E(false)
{}

bool Params::operator==(const Params& o) const
{

#define TEST(field) if(field != o.field) return false;
    TEST(filenames);

    TEST(single_camera);
    TEST(apply_undistort);

    TEST(nx);
    TEST(ny);
    TEST(sq_side);

    TEST(run_surf);
    TEST(surf_min_hessian);
    TEST(surf_dist_ratio);
    TEST(surf_lmeds);

    TEST(use_fusiello_rect);

    TEST(disp_method); 
    TEST(disp_min_disparity);
    TEST(disp_num_disparities);
    TEST(disp_SAD_window_size); 
    TEST(disp_12_max_diff);
    TEST(disp_prefilter_cap); 
    TEST(disp_prefilter_size); 
    TEST(disp_texture_threshold);
    TEST(disp_uniqueness_ratio); 
    TEST(disp_speckle_window_size); 
    TEST(disp_speckle_range); 
    TEST(disp_full_DP);
    TEST(disp_P1);
    TEST(disp_P2);

    TEST(output_dir);
    TEST(export_corner_detect);
    TEST(export_undistorted);
    TEST(export_rectified);
    TEST(export_surf);
    TEST(export_xEx_costfn);
    TEST(export_disparity);
#undef TEST

    return true;
}

void print_usage(const char * exec)
{
    Params p;

    printf(R"V0G0N(

   Usage: %s [OPTION]* [image-filename]*

      Options: 

         -h,--help   Show this message.

         // Output options
         -i <string> Input filename (output from a previous step).
         -o <string> Name of output file. Default is '%s'.
         -D <string> Output directory for image exports. Default is '%s'.

         // Intrinsic calibration
         -K          Calculate the intrinsic calibration parameters.
         -nx <int>   Number of squares in x-direction in calibration image.
         -ny <int>   Number of squares in y-direction in calibration image.
         -sz <float> Width/height of each square in world-coordinates.
         --one-cam   Assume a single camera.

         // Stereo calibration
         -F            Calculate the fundamental matrix.
         --s-hessian <int> Min hessian value used in SURF. Default is '%d'.
         --s-ratio <float> Distance ratio filters SURF matches. Default is '%g'.
         --s-lmeds     Use LMedS (instead of RANSAC) when running surf.

         --fusiello-rect  Use alternate rectification algorithm: Andrea Fusiello

         // Disparity method parameters
         --d-method       Algorithm to use. Default '%s'.
         --d-min          Minimum possible disparity value, default '%d'.
         --d-num          Maximum disparity minus minimum disparity, must be 
                          divisible by 16. Default '%d'.
         --d-SAD          Matched block size (odd number >=1), default '%d'.
         --d-max-diff     Maximum allowed difference (in integer pixel units) in
                          the left-right disparity check. Default '%d'.
         --d-pf-cap       Truncation for prefiltered image pixels, default '%d'.
         --d-pf-size      ... TBA, default is '%d'.
         --d-tex-thres    ... TBA, default is '%d'.
         --d-uniq-ratio   Margin in percentage by which the best (minimum) 
                          computed cost function value should “win” the second
                          best value to consider the found match correct. 
                          Normally, avalue within the 5-15 range is good enough.
                          Default is '%d'.
         --d-spek-window  Maximum size of smooth disparity regions to consider
                          their noise speckles and invalidate. Default is '%d'.
         --d-spek-range   Maximum disparity variation within each connected
                          component. Default is '%d'.
         --d-full-DP      SGBM only, see opencv docs, default is '%d'.
         --d-P1           SGBM only, see opencv docs, default is '%d'.
         --d-P2           SGBM only, see opencv docs, default is '%d'.

         // Export options
         --x-undist  Export undistorted images.
         --x-surf    Export surf corner detection images.
         --x-costfn  Export matlab code to draw xEx cost-function. (/tmp/cf.m)
         --x-rect    Export rectified images.    
         --x-disp    Export disparity images.

         // GUI options
         --show-point-cloud   A crude point-cloud viewer in glut.

         // Test options
         --test-E    Test finding 'E'. 

   About:

      This program has two basic functions. (1) perform intrinsic calibration
      from a set of checkboard images. (2) perform (on-the-fly) stereo 
      calibration, find a disparity map, and generate a point cloud.

      Input and output files are used to save results between runs. The 
      intention is that you perform intrinsic calibration on your cameras:

         > %s -K --corners -nx 7 -ny 9 -sz 0.0254 -o k.yml /path/to/images*.png

      and later create a point cloud:

         > %s -i k.yml -F --x-rect --x-disp left.png right.png

      If the program outputs files (other than the explicit output file 'k.yml'
      in the example above), then they are written to the configurable default
      directory '%s'. Note that on Ubuntu, anything saved to '/dev/shm' is 
      stored in memory (i.e., not the hard disk), and this allows for very fast
      file IO and crude inter-process communication.

   Intrinsic calibration:

      The images must be a sequence of one or more left-right pairs. If the 
      stereo system results from images taken from a single camera, then pass
      the '--one-cam' switch. Otherwise calibration will proceed under the 
      assumption that all the 'left' images are taken with one camera, and
      all the 'right' images with another.

      The calibration points themselves come from corner detection. Remember 
      that nx and ny must count the 'inside-corners', so, a normal chessboard
      will be 7x7. Ignore corners around edge of checker pattern.

   Disparity methods:

      Currently two methods are supported. You can read about 'bm' and 'sgbm' 
      here: http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

   Point cloud:

      The intention is to detect corners with SURF and then apply RANSAC to find
      a consistent stereo system. Rectifications are calculated (can can be
      exported), and then some software is run to generate a disparity map. 
      Finally the progam uses triangulation to calculate a point cloud which is
      saved to the configurable export direction.

)V0G0N",
           exec, 
           p.output_filename.c_str(),
           p.output_dir.c_str(), 
           p.surf_min_hessian, 
           p.surf_dist_ratio,
           p.disp_method_name(p.disp_method).c_str(),
           p.disp_min_disparity,
           p.disp_num_disparities,
           p.disp_SAD_window_size, 
           p.disp_12_max_diff,
           p.disp_prefilter_cap, 
           p.disp_prefilter_size, 
           p.disp_texture_threshold,
           p.disp_uniqueness_ratio, 
           p.disp_speckle_window_size, 
           p.disp_speckle_range, 
           int(p.disp_full_DP),
           p.disp_P1,
           p.disp_P2,
           exec, 
           exec,
           p.output_dir.c_str());
}

Params parse_cmd_args(int argc, char * * argv)
{
    Params p;

    bool first_image = false;
    bool arg_error = false;
    p.filenames.reserve(argc);

    auto safe_num = [&] (int& i) -> double {
	if(++i >= argc) {
	    arg_error = true;
	    return 0;
	}
	char * endptr;
	double v = strtod(argv[i], &endptr);
	if(v == 0) {
	    fprintf(stderr, 
		    "Error in argument '%s': cannot be zero!\n",
		    argv[i]);
	    arg_error = true;
	    return 0;
	}
	return v;
    };

    auto safe_cstr = [&] (int& i) -> const char * {
        if(++i >= argc) {
            arg_error = true;
            return "";
        }
        return argv[i];
    };
    
    // Parse command-line arguments
    for(int i = 1; i < argc && !arg_error; ++i) {
	string arg = argv[i];
        if(!first_image) {
            if(arg == "--help") { p.show_help = true; break; }
            if(arg == "-h")  { p.show_help = true; break; }
            
            if(arg == "-i")  { p.input_filename = safe_cstr(i); continue; }
            if(arg == "-o")  { p.output_filename = safe_cstr(i); continue; }
            if(arg == "-D")  { p.output_dir = safe_cstr(i); continue; }

            if(arg == "-K")  { p.recalc_K = true; continue; }
            if(arg == "-nx") { p.nx = safe_num(i); continue; }
            if(arg == "-ny") { p.ny = safe_num(i); continue; }
            if(arg == "-sz") { p.sq_side = safe_num(i); continue;}
            if(arg == "--one-cam") { p.single_camera = true; continue; }

            if(arg == "-F")  { p.recalc_F = true; continue; }
            if(arg == "--s-hessian") 
                { p.surf_min_hessian = safe_num(i); continue; }
            if(arg == "--s-ratio") 
                { p.surf_dist_ratio = safe_num(i); continue; }
            if(arg == "--s-lmeds") { p.surf_lmeds = true; continue; }

            if(arg == "--fusiello-rect") { p.use_fusiello_rect = true;continue;}

            if(arg == "--d-method") { 
                p.disp_method = -1;
                string method = safe_cstr(i);
                for(uint i = 0; i <= DISPARITY_METHOD_MAX; ++i) 
                    if(method == p.disp_method_name(i)) 
                        p.disp_method = i;
                if(p.disp_method == -1) {
                    arg_error = true;
                    fprintf(stderr, "Unknown stereo-corresp method '%s'\n", 
                            method.c_str());
                }
                continue;
            }
            if(arg == "--d-min") { p.disp_min_disparity = safe_num(i); continue; }
            if(arg == "--d-num") { p.disp_num_disparities = safe_num(i); continue; }
            if(arg == "--d-SAD") { p.disp_SAD_window_size = safe_num(i); continue; } 
            if(arg == "--d-max-diff") { p.disp_12_max_diff = safe_num(i); continue; }
            if(arg == "--d-pf-cap") { p.disp_prefilter_cap = safe_num(i); continue; } 
            if(arg == "--d-pf-size") { p.disp_prefilter_size = safe_num(i); continue; } 
            if(arg == "--d-tex-threshold") { p.disp_texture_threshold = safe_num(i); continue; }
            if(arg == "--d-uniqueness-ratio") { p.disp_uniqueness_ratio = safe_num(i); continue; } 
            if(arg == "--d-speckle-window-sz") { p.disp_speckle_window_size = safe_num(i); continue; } 
            if(arg == "--d-speckle-range") { p.disp_speckle_range = safe_num(i); continue; } 
            if(arg == "--d-full-DP") { p.disp_full_DP = safe_num(i); continue; }
            if(arg == "--d-P1") { p.disp_P1 = safe_num(i); continue; }
            if(arg == "--d-P2") { p.disp_P2 = safe_num(i); continue; }

            if(arg == "--show-point-cloud") 
                { p.show_point_cloud = true; continue; }

            if(arg == "--x-undist") { p.export_undistorted = true; continue; }
            if(arg == "--x-surf")   { p.export_surf = true; continue; }
            if(arg == "--x-costfn") { p.export_xEx_costfn = true; continue; }
            if(arg == "--x-rect")   { p.export_rectified = true; continue; }
            if(arg == "--x-disp")   { p.export_disparity = true; continue; }

            if(arg == "--test-E") { p.test_find_E = true; continue; }
        }
        if(!p.show_help) {
            if(!file_readable(argv[i])) {
                fprintf(stderr, "Could not open file for reading: '%s'\n", 
                        argv[i]);
                arg_error = true;
            } else {
                p.filenames.push_back(argv[i]);
            }
        }
    }

    if(p.nx <= 0 || p.ny <= 0) {
	arg_error = true;
	fprintf(stderr, "Chessboard dimensions (%d x %d) imply %d squares!\n\n",
		p.nx, p.ny, p.nx*p.ny);
    }

    if(p.sq_side * p.sq_side <= 0) {
	arg_error = true;
	fprintf(stderr, "Size of chessboard squares must be larger than zero.\n"
		"Dimensions of (%g x %g) implies size of %g.\n\n",
		p.sq_side, p.sq_side, p.sq_side * p.sq_side);
    }       

    p.okay = !arg_error;

    return p;
}

string Params::disp_method_name(int method_id) const
{
    if(method_id == -1)
        method_id = this->disp_method;

    switch(method_id) {
    case DISPARITY_METHOD_BM: return "bm";
    case DISPARITY_METHOD_SGBM: return "sgbm";
    case DISPARITY_METHOD_ELAS: return "elas";
#if USE_TRICLOPS == 1
    case DISPARITY_METHOD_TRICLOPS: return "triclops";
#endif
    default:
        fprintf(stderr, "FATAL ERROR, unknown disparity-method-id '%d'\n", 
                method_id);
        exit(1);
    }
}


