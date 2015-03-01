
#include <time.h>

#include "stdinc.hpp"
#include "params.hpp"
#include "string-utils.hpp"
#include "stereo-camera-system.hpp"
#include "camera-matrix-utils.hpp"
#include "viewer.hpp"

using namespace cv;
using namespace std;
using namespace Eigen;

int main(int argc, char * * argv)
{
    bool success = true; // until proven otherwise
    srand(time(NULL));

    StereoCameraSystem stereo;
    vector<Vector3d> pts_3d;

    auto params = parse_cmd_args(argc, argv);
    if(!params.okay) {
        fprintf(stderr,"Error passing command line, pass '--help' for help.\n");
        success = false;
    } else if(params.show_help) {
        print_usage(basename(argv[0]));
    } else if(params.test_find_E) {
        test_find_E_comprehensive();
    } else if(params.recalc_K) {
        success = stereo.calculate_intrinsic_calibration(params);
    } else {
        // Load in the intrinsic calibration...
        if(success) success = stereo.calculate_intrinsic_calibration(params);
        if(success) success = stereo.calculate_stereo_calibration(params);
        if(success) success = stereo.calculate_rectification(params);
        if(success) success = stereo.calculate_disparity(params);
        if(success) success = stereo.calculate_stereo_calibration(params);

        for(uint i = 0; i < stereo.disparity_images.size() && success; ++i) {
            pts_3d.clear();
            success = stereo.calculate_point_cloud(params, 
                                                   stereo.disparity_images[i],
                                                   pts_3d);
            if(success) {
                string outname = make_outname(params.filenames[i*2+0], 
                                              params.output_dir,
                                              "points-", ".text");
                printf(" + Saving '%s'\n", outname.c_str());
                FILE * fp = fopen(outname.c_str(), "w");
                if(fp == NULL) {
                    fprintf(stderr, "Failed to open '%s' for writing\n",
                            outname.c_str());
                    success = false;
                    continue;
                }
                for(const auto& X: pts_3d)                
                    fprintf(fp, "%g %g %g\n", X(0), X(1), X(2));
                fclose(fp);                    
            }
        }
        
        if(success && params.show_point_cloud) {
            glut_display_cloud(pts_3d);
        }

        printf("\nFinishing with success = %d\n", int(success));
    }

    return success ? EXIT_SUCCESS : EXIT_FAILURE;
}


