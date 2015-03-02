
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
#include "main-viewer.hpp"
#include "gl-visualizer.hpp"
#include "image-viewer.hpp"

#include <QShortcut>
#include <QApplication>
#include <QSettings>
#include <QGridLayout>

#define This MainViewer

class This::Pimpl
{
public:
    Pimpl(MainViewer * viewer_, SharedData& shared_data_)
        : viewer(viewer_), shared_data(shared_data_)
    {}

    MainViewer * viewer;
    SharedData& shared_data;

    // Important widgets
    QTabWidget * tab_widget;
    QWidget * cpanel;

    // Viewers
    ImageViewer * imv_im0, * imv_im1;
    ImageViewer * imv_undistort0, * imv_undistort1;
    ImageViewer * imv_surf, * imv_inliers;
    ImageViewer * imv_rect0, * imv_rect1;
    ImageViewer * imv_disparity;

    // Data
    vector<Vector3d> cloud;
    Vector3d cloud_centre;

    
    void make_ui();
};

// ---------------------------------------------------------------- Construction

This::MainViewer(SharedData& shared_data, QWidget * parent)
    : QMainWindow(parent), _pimpl(new Pimpl(this, shared_data))
{
    _pimpl->make_ui();

    // Wire shortcuts
    {
	auto shortcut = new QShortcut(QKeySequence("Alt+Left"), this);
	shortcut->setContext(Qt::ApplicationShortcut);
	connect(shortcut, SIGNAL(activated()), this, SLOT(prev_tab()));	
    }

    {
	auto shortcut = new QShortcut(QKeySequence("Alt+Right"), this);
	shortcut->setContext(Qt::ApplicationShortcut);
	connect(shortcut, SIGNAL(activated()), this, SLOT(next_tab()));	
    }

    // Wire save-state
    {
        connect(qApp, SIGNAL(aboutToQuit()), this, SLOT(save_state()));
    }

    // Finally, restore-state and GO!
    restore_state();
}

This::~MainViewer()
{
    delete _pimpl;
}

// ----------------------------------------------------------------------- Slots
// Tab-control
void This::next_tab()
{
    int idx = _pimpl->tab_widget->currentIndex() + 1;
    _pimpl->tab_widget->setCurrentIndex(idx % _pimpl->tab_widget->count());
}

void This::prev_tab()
{
    int idx = _pimpl->tab_widget->currentIndex() - 1;
    if(idx < 0)
	idx = _pimpl->tab_widget->count() - 1;
    _pimpl->tab_widget->setCurrentIndex(idx);
}

// Application-state
void This::save_state()
{
    _pimpl->shared_data.quit = true;
    QSettings settings;
    settings.setValue("CS_VIEWER_POS_main_viewer", saveGeometry());
    settings.setValue("CS_VIEWER_STATE_main_viewer", saveState());
}

void This::restore_state()
{
    QSettings settings;
    restoreGeometry(settings.value("CS_VIEWER_POS_main_viewer").toByteArray());
    restoreState(settings.value("CS_VIEWER_STATE_main_viewer").toByteArray());
}

void This::update_gui()
{
    std::lock_guard<std::mutex> lock(_pimpl->shared_data.padlock);
    // P.imv_right->setImage(*image_to_QImage(make_im(raw_r, hull_r, ppt)));
}

// ---------------------------------------------------------------------- render

void This::render()
{
    printf("render not yet implemented\n");
}

// --------------------------------------------------------------------- make_ui

void This::Pimpl::make_ui()
{   
    // Main window is a control-panel and tab-widget side by side

    { // -- (*) -- control panel
        cpanel = new QWidget;
        //cpanel->setLayout(layout);
        cpanel->setFixedWidth(300);
        cpanel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
    }

    { // -- (*) -- tab-widget
        tab_widget = new QTabWidget;
        tab_widget->setMovable(true);    
    }

    { // -- (*) -- create important widgets
        auto make_image_viewer = [&] (ImageViewer * * im) {
            *im = new ImageViewer();
            // (*im)->setSizePolicy(QSizePolicy::Expanding, 
            //                      QSizePolicy::Expanding);
        };

        make_image_viewer(&imv_im0);
        make_image_viewer(&imv_im1);
        make_image_viewer(&imv_undistort0);
        make_image_viewer(&imv_undistort1);
        make_image_viewer(&imv_surf);
        make_image_viewer(&imv_inliers);
        make_image_viewer(&imv_rect0);
        make_image_viewer(&imv_rect1);
        make_image_viewer(&imv_disparity);        
    }

    { // -- (*) -- im0, im1, distort0, distort1
        auto * layout = new QGridLayout;
        layout->addWidget(imv_im0, 0, 0);
        layout->addWidget(imv_im1, 0, 1);
        layout->addWidget(imv_undistort0, 1, 0);
        layout->addWidget(imv_undistort1, 1, 1);

        auto * wgt = new QWidget;        
        wgt->setLayout(layout);
        tab_widget->addTab(wgt, "Input Images");
    }

    { // -- (*) -- point-cloud
	auto gl_vis = new GLVisualizer;
	gl_vis->onDrawThunk = [&] () { viewer->render(); };	
        tab_widget->addTab(gl_vis, "Point Cloud");
    }

    { // -- (*) -- pull everything together in main widget
	auto * layout = new QHBoxLayout;
	layout->addWidget(cpanel);
	layout->addWidget(tab_widget);
        auto * wgt = new QWidget;        
        wgt->setLayout(layout);
        viewer->setCentralWidget(wgt);
    }
}


