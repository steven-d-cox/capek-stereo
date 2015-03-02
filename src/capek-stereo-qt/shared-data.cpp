
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
#include "shared-data.hpp"

#include <chrono>
#include <thread>
#include <QImage>

using namespace std;

#define This SharedData

This::SharedData() 
    : QObject(nullptr),
      padlock(), 
      quit(false), 
      intrinsics_dirty(true), 
      surf_dirty(true), 
      stereo_calibration_dirty(true),
      rectification_dirty(true), 
      disparity_dirty(true),
      params(),      
      im0(new QImage(0, 0, QImage::Format_ARGB32)), 
      im1(new QImage(0, 0, QImage::Format_ARGB32)), 
      undistort0(new QImage(0, 0, QImage::Format_ARGB32)), 
      undistort1(new QImage(0, 0, QImage::Format_ARGB32)),
      surf(new QImage(0, 0, QImage::Format_ARGB32)), 
      inliers(new QImage(0, 0, QImage::Format_ARGB32)), 
      rect0(new QImage(0, 0, QImage::Format_ARGB32)), 
      rect1(new QImage(0, 0, QImage::Format_ARGB32)),
      disparity(new QImage(0, 0, QImage::Format_ARGB32))
{ }

This::~SharedData()
{
    delete im0; 
    delete im1;
    delete undistort0;
    delete undistort1;
    delete surf;
    delete inliers;
    delete rect0;
    delete rect1;
    delete disparity;
}

static void update_qimage(const cv::Mat& im, QImage * * qim)
{
    if(im.rows == 0 && im.cols == 0) return;
    
    // Destroy whatever is at *qim
    delete *qim;
    *qim = nullptr;

    uint w = im.cols, h = im.rows;

    // Copy pixels across
    if(im.type() == CV_8UC1) { // greyscale
        (*qim) = new QImage(w, h, QImage::Format_Indexed8);

        // Set the colour tables
        QVector<QRgb> colour_table(256);
        for(int i=0; i < 256; ++i)
            colour_table[i] = qRgb(i, i, i);
        (*qim)->setColorTable(colour_table);

        // Copy across one row of pixels
        for(uint row = 0; row < h; ++row) {
            const uint8 * ptr = im.ptr(row);
            const uint8 * end = ptr + w;
            memcpy((*qim)->scanLine(row), ptr, w);
        }
        
    } else if(im.type() == CV_8UC3) { // RGB
        (*qim) = new QImage(w, h, QImage::Format_ARGB32);
        
        for(uint row = 0; row < h; ++row) {
            uint32 * pos = (uint32 *) (*qim)->scanLine(row);
            const uint8 * ptr = im.ptr(row);
            const uint8 * end = ptr + 3 * w;
            while(ptr != end) {
                uint8 r = *ptr++;
                uint8 g = *ptr++;
                uint8 b = *ptr++;
                *pos++ = (0xff << 24) | (r << 16) | (g << 8) | (b << 0);
            }
        }
    } else if(im.type() == CV_32FC1) {
        (*qim) = new QImage(w, h, QImage::Format_Indexed8);

        // Set the colour tables
        QVector<QRgb> colour_table(256);
        for(int i=0; i < 256; ++i)
            colour_table[i] = qRgb(i, i, i);
        (*qim)->setColorTable(colour_table);

        // Copy across one row of pixels
        for(uint row = 0; row < h; ++row) {
            uint8 * dest = (*qim)->scanLine(row);
            uint8 * end = dest + w;
            const float * ptr = (const float *) im.ptr(row);
            while(dest != end) 
                *dest++ = (*ptr++) * 255.0f;
        }

    } else {        
        printf("Unsupported cv matrix type: %d\n", im.type());
        exit(1);
    }    
}

void This::set_intrinsic_results(const StereoCameraSystem& stereo)
{
    std::lock_guard<std::mutex> lock(padlock);
    if(stereo.images.size() >= 2) {
        update_qimage(stereo.images[0], &im0);
        update_qimage(stereo.images[1], &im1);
    }
    if(stereo.undistort_images.size() >= 2) {
        update_qimage(stereo.undistort_images[0], &undistort0);
        update_qimage(stereo.undistort_images[1], &undistort1);
    }
    emit intrinsics_updated();
}

void This::set_surf_results(const StereoCameraSystem& stereo)
{
    std::lock_guard<std::mutex> lock(padlock);
    if(stereo.surf_images.size() >= 1) 
        update_qimage(stereo.surf_images[0], &surf);
    emit surf_updated();
}

void This::set_stereo_calibration_results(const StereoCameraSystem& stereo)
{
    std::lock_guard<std::mutex> lock(padlock);
    update_qimage(stereo.inlier_image, &inliers);
    emit stereo_calibration_updated();
}

void This::set_rectification_results(const StereoCameraSystem& stereo)
{
    std::lock_guard<std::mutex> lock(padlock);
    if(stereo.rect_images.size() >= 2) {
        update_qimage(stereo.rect_images[0], &rect0);
        update_qimage(stereo.rect_images[1], &rect1);
    }
    emit rectification_updated();
}

void This::set_disparity_results(const StereoCameraSystem& stereo)
{
    std::lock_guard<std::mutex> lock(padlock);
    if(stereo.disparity_images.size() > 0) 
        update_qimage(stereo.disparity_images[0], &disparity);
    emit disparity_updated();
}

void This::set_point_cloud_results(const vector<Vector3d>& pts)
{
    std::lock_guard<std::mutex> lock(padlock);
    pts_3d = pts;
    emit point_cloud_updated();
}

void This::worker_thread() 
{        
    Params local; // local (in-thread) copy of parameters
    StereoCameraSystem stereo;
    vector<Vector3d> pts_3d;

    auto step = [&] () {
        bool success = true;

        {
            std::lock_guard<std::mutex> lock(padlock);
            local = params;
            local.run_surf = false; // we do this manually
        }

        if(intrinsics_dirty) {
            StereoCameraSystem new_stereo;
            stereo = new_stereo; // clear everything out
            surf_dirty = true;
            if(stereo.calculate_intrinsic_calibration(local)) 
                set_intrinsic_results(stereo);
            intrinsics_dirty = false;
        }

        if(!intrinsics_dirty && surf_dirty) {
            stereo_calibration_dirty = true;
            if(stereo.surf_corresponding_points(local)) 
                set_surf_results(stereo);
            surf_dirty = false;
        }

        if(!intrinsics_dirty && !surf_dirty && stereo_calibration_dirty) {
            rectification_dirty = true;
            if(stereo.calculate_stereo_calibration(local)) 
                set_stereo_calibration_results(stereo);
            stereo_calibration_dirty = false;
        }

        if(!intrinsics_dirty && !surf_dirty && !stereo_calibration_dirty
           && rectification_dirty) {
            disparity_dirty = true;
            if(stereo.calculate_rectification(local)) 
                set_rectification_results(stereo);
            rectification_dirty = false;
        }
          
        if(!intrinsics_dirty && !surf_dirty && !stereo_calibration_dirty
           && !rectification_dirty && disparity_dirty) {
            if(stereo.calculate_disparity(local)) {
                set_disparity_results(stereo);

                printf("\n\nFull run complete.\n");
            }
            disparity_dirty = false;
        }  

        // Get 3d points
        // if(stereo.disparity_images.size() > 0) {
        //     pts_3d.clear();
        //     if(!stereo.calculate_point_cloud(local, 
        //                                      stereo.disparity_images[0],
        //                                      pts_3d))
        //         return false;
        //     set_point_cloud_results(pts_3d);
        // }


        return true;
    };

    while(!quit) {         
        step();
        this_thread::sleep_for(chrono::milliseconds(50));
    }
};

