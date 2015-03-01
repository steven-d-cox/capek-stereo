
#include <time.h>

#include "stdinc.hpp"
#include "params.hpp"
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

        if(success && params.show_point_cloud) {
            vector<Vector3d> pts_3d;
            success = stereo.calculate_point_cloud(params, pts_3d);
            glut_display_cloud(pts_3d);
        }

        printf("\nFinishing with success = %d\n", int(success));
    }

    return success ? EXIT_SUCCESS : EXIT_FAILURE;
}


