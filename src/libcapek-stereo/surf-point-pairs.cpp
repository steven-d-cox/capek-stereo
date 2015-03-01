
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

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/flann/flann.hpp>

#include "surf-point-pairs.hpp"

using namespace cv;
using namespace std;

void surf_corresponding_pairs(const Mat& mat0, const Mat& mat1,
                              vector<Vector2d>& pts0, 
                              vector<Vector2d>& pts1,
                              int min_hessian, double dist_ratio,
                              Mat& out)
{
    SurfFeatureDetector detector(min_hessian);
    SurfDescriptorExtractor extractor;
    FlannBasedMatcher matcher;

    vector<KeyPoint> keypoints0, keypoints1;
    Mat descriptors0; // surf descritors
    Mat descriptors1; 
    vector<DMatch> matches; // matches between two images
    vector<DMatch> good_matches; // good matches
    
    // Run SURF on each image
    detector.detect(mat0, keypoints0); // run surf
    detector.detect(mat1, keypoints1); 
    extractor.compute(mat0, keypoints0, descriptors0); 
    extractor.compute(mat1, keypoints1, descriptors1); 

    // Matching descriptor vectors using FLANN matcher
    matcher.match(descriptors0, descriptors1, matches);

    // Get min/max distances
    double max_dist = numeric_limits<double>::min(); 
    double min_dist = numeric_limits<double>::max();                
    for(int i = 0; i < descriptors0.rows; ++i) { 
        double dist = matches[i].distance;
        if(dist < min_dist) min_dist = dist;
        if(dist > max_dist) max_dist = dist;
    }

    // -- Want only "good" matches (distance is less than dist_ratio*min_dist)
    for(int i = 0; i < descriptors0.rows; ++i)
        if(matches[i].distance < dist_ratio * min_dist)
            good_matches.push_back(matches[i]); 

    // -- Get the keypoints from the good matches
    pts0.reserve(good_matches.size());
    pts1.reserve(good_matches.size());
    for(uint i = 0; i < good_matches.size(); i++)  {      
        Point2d p0 = keypoints0[good_matches[i].queryIdx].pt;
        Point2d p1 = keypoints1[good_matches[i].trainIdx].pt;
        pts0.push_back(Vector2d(p0.x, p0.y));
        pts1.push_back(Vector2d(p1.x, p1.y));
    }

    // Export
    {
        vector<Vector4d> corresp;
        corresp.resize(pts0.size());
        for(uint i = 0; i < corresp.size(); ++i)
            corresp[i] = Vector4d(pts0[i](0), pts0[i](1),
                                  pts1[i](0), pts1[i](1));
        draw_and_save_corresp(mat0, mat1, corresp, out);
    }
}

void draw_and_save_corresp(const Mat& mat0, const Mat& mat1,
                           const vector<Vector4d>& corresp,
                           Mat& out)
{
    vector<KeyPoint> keypoints0, keypoints1;
    vector<DMatch> good_matches; // good matches

    // Build the keypoints and 'good-matches'
    keypoints0.resize(corresp.size());
    keypoints1.resize(corresp.size());
    good_matches.resize(corresp.size());
    for(uint i = 0; i < corresp.size(); ++i) {
        const auto& xx = corresp[i];
        keypoints0[i] = KeyPoint(xx(0), xx(1), 30.0);
        keypoints1[i] = KeyPoint(xx(2), xx(3), 30.0);
        good_matches[i] = DMatch(i, i, 0);
    }    

    drawMatches(mat0, keypoints0, 
                mat1, keypoints1,
                good_matches, 
                out, Scalar::all(-1), Scalar::all(-1),
                vector<char>(), 
                DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);    
}

 
