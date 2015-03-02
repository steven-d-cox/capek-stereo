
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
#include "qt-includes.hpp"
#include "params.hpp"
#include "stereo-camera-system.hpp"

#include <mutex>
#include <atomic>

class QImage;

class SharedData : public QObject
{
    Q_OBJECT
    ;

public:
    std::mutex padlock;        // Read/write access to all members should be
                               // guarded by this mutex. Make sure you use
                               // a lockguard (because of RAII)    
    // e.g., std::lock_guard<std::mutex> lock(padlock);

    std::atomic<bool> quit;    // tells worker thread to end when set to TRUE

    std::atomic<bool> intrinsics_dirty, 
        surf_dirty, 
        stereo_calibration_dirty,
        rectification_dirty, 
        disparity_dirty;

    Params params;             // parameter block used by StereoCameraSystem

    // Calulated data...
    QImage * im0, * im1;
    QImage * undistort0, * undistort1;
    QImage * surf, * inliers;
    QImage * rect0, * rect1;
    QImage * disparity;

    vector<Vector3d> pts_3d;

    // Constructors
    SharedData();
    ~SharedData();

    // No 'accidents'
    SharedData(const SharedData&) = delete; 
    SharedData(const SharedData&&) = delete; 
    void operator=(const SharedData&) = delete;
    void operator=(const SharedData&&) = delete;

    void worker_thread();

private:
    // These 'set' methods _must_ be called from the worker thread _only_
    void set_intrinsic_results(const StereoCameraSystem& stereo);
    void set_surf_results(const StereoCameraSystem& stereo);
    void set_stereo_calibration_results(const StereoCameraSystem& stereo);
    void set_rectification_results(const StereoCameraSystem& stereo);
    void set_disparity_results(const StereoCameraSystem& stereo);
    void set_point_cloud_results(const vector<Vector3d>& pts);

signals:
    void intrinsics_updated();
    void surf_updated();
    void stereo_calibration_updated();
    void rectification_updated();
    void disparity_updated();
    void point_cloud_updated();
};

