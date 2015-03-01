
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

/**
 * + min_disparity     – Minimum possible disparity value.
 * + num_disparities   – Maximum disparity minus minimum disparity. 
 *                       This parameter must be divisible by 16.
 * + SAD_window_size   – Matched block size. It must be an odd number >=1 .
 * + disp_12_max_diff  – Maximum allowed difference (in integer pixel units) in 
 *                       the left-right disparity check.
 * + prefilter_cap     – Truncation value for the prefiltered image pixels.

//...

 * + uniqueness_ratio  – Margin in percentage by which the best (minimum) 
 *                       computed cost function value should “win” the second
 *                       best value to consider the found match correct. 
 *                       Normally, avalue within the 5-15 range is good enough.
 * + speckle_window_size – Maximum size of smooth disparity regions to consider
 *                       their noise speckles and invalidate.
 * + speckle_range     – Maximum disparity variation within each connected
 *                       component.
 */
void disparity_bm(const cv::Mat& grey0, const cv::Mat& grey1, 
                  cv::Mat& disp,
                  int min_disparity = -39, 
                  int num_disparities = 112,
                  int SAD_window_size = 9, 
                  int disp_12_max_diff = 1,
                  int prefilter_cap = 61, 
                  int prefilter_size = 5,
                  int texture_threshold = 507,
                  int uniqueness_ratio = 0, 
                  int speckle_window_size = 0, 
                  int speckle_range = 9);
 
// @see http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html?highlight=disparity#StereoSGBM::StereoSGBM%28int%20minDisparity,%20int%20numDisparities,%20int%20SADWindowSize,%20int%20P1,%20int%20P2,%20int%20disp12MaxDiff,%20int%20preFilterCap,%20int%20uniquenessRatio,%20int%20speckleWindowSize,%20int%20speckleRange,%20bool%20fullDP%29
void disparity_sgbm(const cv::Mat& grey0, const cv::Mat& grey1, cv::Mat& disp,
                    int min_disparity = -64, 
                    int num_disparities = 192,
                    int SAD_window_size = 5, 
                    int disp_12_max_diff = 0,
                    int prefilter_cap = 0, 
                    int uniqueness_ratio = 0, 
                    int speckle_window_size = 0, 
                    int speckle_range = 0,
                    bool full_DP = false,
                    int P1 = 0,
                    int P2 = 0);


