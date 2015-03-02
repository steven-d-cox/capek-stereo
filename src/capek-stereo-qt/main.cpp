
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

#include <time.h>

#include "stdinc.hpp"

#include "shared-data.hpp"
#include "main-viewer.hpp"

#include <QApplication>
#include <QPushButton>

#include <chrono>
#include <thread>

using namespace cv;
using namespace std;
using namespace Eigen;

int main(int argc, char * * argv)
{
    bool success = true; // until proven otherwise
    srand(time(NULL));

    SharedData shared;
    shared.params = parse_cmd_args(argc, argv);

    QApplication app(argc, argv);
    MainViewer viewer(shared);

    auto fun = [&] () {
        bool first = true;
        Params params;

        while(!shared.quit) {
            bool run = false;

            { // copy out shared_data.params
                std::lock_guard<std::mutex> lock(shared.padlock);
                if(first || !(params == shared.params)) {
                    params = shared.params;
                    run = true;
                    first = false;
                }
            }



            this_thread::sleep_for(chrono::milliseconds(50));
        }
    };

    thread worker(fun);

    // Execute gui
    viewer.show();
    int ret = app.exec();

    // Wait for worker thread
    worker.join();

    return ret;
}

// int zmain(int argc, char * * argv)
// {
//     bool success = true; // until proven otherwise
//     srand(time(NULL));

//     StereoCameraSystem stereo;
//     vector<Vector3d> pts_3d;

//     auto params = parse_cmd_args(argc, argv);
//     if(!params.okay) {
//         fprintf(stderr,"Error passing command line, pass '--help' for help.\n");
//         success = false;
//     } else if(params.show_help) {
//         print_usage(basename(argv[0]));
//     } else if(params.test_find_E) {
//         test_find_E_comprehensive();
//     } else if(params.recalc_K) {
//         success = stereo.calculate_intrinsic_calibration(params);
//     } else {
//         // Load in the intrinsic calibration...
//         if(success) success = stereo.calculate_intrinsic_calibration(params);
//         if(success) success = stereo.calculate_stereo_calibration(params);
//         if(success) success = stereo.calculate_rectification(params);
//         if(success) success = stereo.calculate_disparity(params);
//         if(success) success = stereo.calculate_stereo_calibration(params);

//         for(uint i = 0; i < stereo.disparity_images.size() && success; ++i) {
//             pts_3d.clear();
//             success = stereo.calculate_point_cloud(params, 
//                                                    stereo.disparity_images[i],
//                                                    pts_3d);
//             if(success) {
//                 string outname = make_outname(params.filenames[i*2+0], 
//                                               params.output_dir,
//                                               "points-", ".text");
//                 printf(" + Saving '%s'\n", outname.c_str());
//                 FILE * fp = fopen(outname.c_str(), "w");
//                 if(fp == NULL) {
//                     fprintf(stderr, "Failed to open '%s' for writing\n",
//                             outname.c_str());
//                     success = false;
//                     continue;
//                 }
//                 for(const auto& X: pts_3d)                
//                     fprintf(fp, "%g %g %g\n", X(0), X(1), X(2));
//                 fclose(fp);                    
//             }
//         }
        
//         printf("\nFinishing with success = %d\n", int(success));
//     }

//     return success ? EXIT_SUCCESS : EXIT_FAILURE;
// }


