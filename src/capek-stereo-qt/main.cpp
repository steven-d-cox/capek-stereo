
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

    thread worker([&] () { shared.worker_thread(); });

    // Execute gui
    viewer.show();
    int ret = app.exec();

    // Wait for worker thread
    worker.join();

    return ret;
}


