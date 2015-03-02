
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
#include "push-button-c11.hpp"

#include <QShortcut>
#include <QApplication>
#include <QSettings>
#include <QCheckBox>
#include <QSlider>
#include <QFormLayout>
#include <QPushButton>
#include <QLineEdit>
#include <QComboBox>

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

    // Controls
    QLineEdit * le_im0, * le_im1;

    QCheckBox * cb_apply_undistort;
    QLineEdit * le_surf_min_hessian, * le_surf_ratio;
    QCheckBox * cb_surf_lmeds;
    QCheckBox * cb_fusiello_rect;

    // Disparity
    QComboBox * combo_method;
    QLineEdit * le_min_disparity;
    QLineEdit * le_num_disparities;
    QLineEdit * le_SAD_window_size, * le_12_max_diff;
    QLineEdit * le_prefilter_cap, * le_prefilter_size;
    QLineEdit * le_texture_threshold, * le_uniqueness_ratio;
    QLineEdit * le_speckle_window_size, * le_speckle_range;
    QCheckBox * cb_full_DP;
    QLineEdit * le_P1, * le_P2;

    // Disparity Elas
    QLineEdit * le_elas_disp_min;               
    QLineEdit * le_elas_disp_max;               
    QLineEdit * le_elas_support_threshold;      
    QLineEdit * le_elas_support_texture;        
    QLineEdit * le_elas_candidate_stepsize;     
    QLineEdit * le_elas_incon_window_size;      
    QLineEdit * le_elas_incon_threshold;        
    QLineEdit * le_elas_incon_min_support;      
    QCheckBox * cb_elas_add_corners;
    QLineEdit * le_elas_grid_size;             
    QLineEdit * le_elas_beta;                  
    QLineEdit * le_elas_gamma;                 
    QLineEdit * le_elas_sigma;                 
    QLineEdit * le_elas_sradius;               
    QLineEdit * le_elas_match_texture;         
    QLineEdit * le_elas_lr_threshold;          
    QLineEdit * le_elas_speckle_sim_threshold; 
    QLineEdit * le_elas_speckle_size;          
    QLineEdit * le_elas_ipol_gap_width;        
    QCheckBox * cb_elas_filter_median;        
    QCheckBox * cb_elas_filter_adaptive_mean;
    QCheckBox * cb_elas_postprocess_only_left; 
    QCheckBox * cb_elas_subsampling;    

    // Viewers
    ImageViewer * imv_im0, * imv_im1;
    ImageViewer * imv_undistort0, * imv_undistort1;
    ImageViewer * imv_surf, * imv_inliers;
    ImageViewer * imv_rect0, * imv_rect1;
    ImageViewer * imv_disparity;

    // Data
    vector<Vector3d> cloud;
    Vector3d cloud_centre;

    // Making the user interface
    void make_ui();
};

// ------------------------------------------------------------------ Qt Helpers

static QWidget * newSpacer()
{
    QWidget * widget = new QWidget;
    widget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    widget->setMinimumSize(0, 0);
    return widget;
}

// Create a widget at the specified address, returning *ptr
template<typename T, typename... Args> 
T * newWidget(T * * ptr, Args&... args)
{
    T * res = new T(args...);
    if(ptr)
	*ptr = res;
    return res;
}

static std::string str(const QString& qstr) { return qstr.toUtf8().data(); }

static QString qstr(const char * s) { return QString(s); }

static QString qstr(const std::string& s) { return QString(s.c_str()); }

static QString qstr(uint x)
{
    char buf[250];
    sprintf(buf, "%d", x);
    return qstr(buf);
}

static Qt::CheckState qchecked(bool v) 
{ 
    return v ? Qt::Checked : Qt::Unchecked; 
};

static QWidget * newCb(QCheckBox * * cb_ptr) 
{
    auto res = new QWidget;
    auto layout = new QHBoxLayout;
    *cb_ptr = new QCheckBox;
    layout->addWidget(*cb_ptr);
    layout->addWidget(newSpacer());
    res->setLayout(layout);
    return res;
};

static QSlider * newSlider(QSlider * * slider_ptr) 
{
    auto slider = new QSlider;
    slider->setTickInterval(20);
    slider->setTickPosition(QSlider::TicksBelow);
    slider->setRange(1, 100);
    slider->setOrientation(Qt::Horizontal);
    *slider_ptr = slider;
    return slider;
};

static QFrame * newHLine() 
{
    QFrame* frame = new QFrame();
    frame->setFrameShape(QFrame::HLine);
    return frame;
};

static QWidget * newFileSelect(QLineEdit * * lineEdit_ptr, 
			       bool dir_only = false)
{
    auto * res = new QWidget;
    auto * layout = new QHBoxLayout;
    auto * lineEdit = new QLineEdit;
    *lineEdit_ptr = lineEdit;
    lineEdit->setFixedWidth(140);

    auto * button = new QPushButtonC11("...", [=] (bool) {
	    auto opts = dir_only ? QFileDialog::ShowDirsOnly : 0;
	    QString filename;
	    if(dir_only) {
		filename = QFileDialog::getExistingDirectory(res, 
							     res->tr("Open Directory"),
							     lineEdit->text(),
							     QFileDialog::ShowDirsOnly
							     | QFileDialog::DontResolveSymlinks);
	    } else {
		filename = QFileDialog::getOpenFileName(res, 
							res->tr("Open File"),
							lineEdit->text(),
							res->tr("Files (*.*)"));

	    }
	    lineEdit->setText(filename); 
	});
    button->setMaximumWidth(20);

    layout->addWidget(lineEdit);
    layout->addWidget(button);
    layout->addWidget(newSpacer());
    layout->setContentsMargins(0, 0, 0, 0);    
    res->setLayout(layout);
    
    return res;
}

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

    // Wire gui updates
    connect(&shared_data, SIGNAL(intrinsics_updated()), 
            this, SLOT(update_intrinsics()), Qt::QueuedConnection);
    connect(&shared_data, SIGNAL(surf_updated()), 
            this, SLOT(update_surf()), Qt::QueuedConnection);
    connect(&shared_data, SIGNAL(stereo_calibration_updated()), 
            this, SLOT(update_stereo_calibration()), Qt::QueuedConnection);
    connect(&shared_data, SIGNAL(rectification_updated()), 
            this, SLOT(update_rectification()), Qt::QueuedConnection);
    connect(&shared_data, SIGNAL(disparity_updated()), 
            this, SLOT(update_disparity()), Qt::QueuedConnection);
    connect(&shared_data, SIGNAL(point_cloud_updated()), 
            this, SLOT(update_point_cloud()), Qt::QueuedConnection);

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

void This::intrinsics_dirty()
{
    printf("intrinsics dirty\n");
    
    Params& p = _pimpl->shared_data.params;
    std::lock_guard<std::mutex> lock(_pimpl->shared_data.padlock);
    if(p.filenames.size() < 2)
        p.filenames.resize(2);
    p.filenames[0] = qPrintable(_pimpl->le_im0->text());
    p.filenames[1] = qPrintable(_pimpl->le_im1->text());
    p.apply_undistort = _pimpl->cb_apply_undistort->isChecked();

    printf("apply-undistort = %d\n", (int) p.apply_undistort);
    
    _pimpl->shared_data.intrinsics_dirty = true;
}

void This::surf_dirty()
{
    printf("surf dirty\n");
    
    
    Params& p = _pimpl->shared_data.params;
    std::lock_guard<std::mutex> lock(_pimpl->shared_data.padlock);
    
    p.surf_min_hessian = atoi(qPrintable(_pimpl->le_surf_min_hessian->text()));
    p.surf_dist_ratio = atof(qPrintable(_pimpl->le_surf_ratio->text()));

    _pimpl->shared_data.surf_dirty = true;
}

void This::stereo_calibration_dirty()
{
    printf("stereo calibration dirty\n");

    Params& p = _pimpl->shared_data.params;
    std::lock_guard<std::mutex> lock(_pimpl->shared_data.padlock);

    p.surf_lmeds = _pimpl->cb_surf_lmeds->isChecked();

    _pimpl->shared_data.stereo_calibration_dirty = true;
}

void This::rectification_dirty()
{
    printf("rectification dirty\n");

    Params& p = _pimpl->shared_data.params;
    std::lock_guard<std::mutex> lock(_pimpl->shared_data.padlock);

    p.use_fusiello_rect = _pimpl->cb_fusiello_rect->isChecked();

    _pimpl->shared_data.rectification_dirty = true;
}

void This::disparity_dirty()
{
    printf("disparity dirty\n");

    Params& p = _pimpl->shared_data.params;
    auto& elas = p.elas_params;
    std::lock_guard<std::mutex> lock(_pimpl->shared_data.padlock);

    p.disp_method = _pimpl->combo_method->currentIndex();

#define SETINT(dst, src) { dst = atoi(qPrintable(_pimpl->src->text())); }
#define SETNUM(dst, src) { dst = atof(qPrintable(_pimpl->src->text())); }
#define SETCB(dst, src) { dst = _pimpl->src->isChecked(); }

    SETINT(p.disp_min_disparity,       le_min_disparity);
    SETINT(p.disp_num_disparities,     le_num_disparities);
    SETINT(p.disp_SAD_window_size,     le_SAD_window_size);
    SETINT(p.disp_12_max_diff,         le_12_max_diff);
    SETINT(p.disp_prefilter_cap,       le_prefilter_cap);
    SETINT(p.disp_prefilter_size,      le_prefilter_size);
    SETINT(p.disp_texture_threshold,   le_texture_threshold);
    SETINT(p.disp_uniqueness_ratio,    le_uniqueness_ratio);
    SETINT(p.disp_speckle_window_size, le_speckle_window_size);
    SETINT(p.disp_speckle_range,       le_speckle_range);
    SETCB (p.disp_full_DP,             cb_full_DP);
    SETINT(p.disp_P1,                  le_P1);
    SETINT(p.disp_P2,                  le_P2);

    SETINT(elas.disp_min,              le_elas_disp_min);               
    SETINT(elas.disp_max,              le_elas_disp_max);               
    SETNUM(elas.support_threshold,     le_elas_support_threshold);      
    SETINT(elas.support_texture,       le_elas_support_texture);        
    SETINT(elas.candidate_stepsize,    le_elas_candidate_stepsize);     
    SETINT(elas.incon_window_size,     le_elas_incon_window_size);      
    SETINT(elas.incon_threshold,       le_elas_incon_threshold);        
    SETINT(elas.incon_min_support,     le_elas_incon_min_support);      
    SETCB (elas.add_corners,           cb_elas_add_corners);
    SETINT(elas.grid_size,             le_elas_grid_size);             
    SETNUM(elas.beta,                  le_elas_beta);                  
    SETNUM(elas.gamma,                 le_elas_gamma);                 
    SETNUM(elas.sigma,                 le_elas_sigma);                 
    SETNUM(elas.sradius,               le_elas_sradius);               
    SETINT(elas.match_texture,         le_elas_match_texture);         
    SETINT(elas.lr_threshold,          le_elas_lr_threshold);          
    SETNUM(elas.speckle_sim_threshold, le_elas_speckle_sim_threshold); 
    SETINT(elas.speckle_size,          le_elas_speckle_size);          
    SETINT(elas.ipol_gap_width,        le_elas_ipol_gap_width);        
    SETCB (elas.filter_median,         cb_elas_filter_median);        
    SETCB (elas.filter_adaptive_mean,  cb_elas_filter_adaptive_mean);
    SETCB (elas.postprocess_only_left, cb_elas_postprocess_only_left); 
    SETCB (elas.subsampling,           cb_elas_subsampling); 

#undef SETINT
#undef SETNUM
#undef SETCB

    _pimpl->shared_data.disparity_dirty = true;
}

void This::update_intrinsics()
{
    SharedData& shared = _pimpl->shared_data;
    std::lock_guard<std::mutex> lock(shared.padlock);
    _pimpl->imv_im0->setImage(*shared.im0);
    _pimpl->imv_im1->setImage(*shared.im1);
    _pimpl->imv_undistort0->setImage(*shared.undistort0);
    _pimpl->imv_undistort1->setImage(*shared.undistort1);
}

void This::update_surf()
{
    SharedData& shared = _pimpl->shared_data;
    std::lock_guard<std::mutex> lock(shared.padlock);
    _pimpl->imv_surf->setImage(*shared.surf);
}

void This::update_stereo_calibration()
{
    SharedData& shared = _pimpl->shared_data;
    std::lock_guard<std::mutex> lock(shared.padlock);
    _pimpl->imv_inliers->setImage(*shared.inliers);
}

void This::update_rectification()
{
    SharedData& shared = _pimpl->shared_data;
    std::lock_guard<std::mutex> lock(shared.padlock);
    _pimpl->imv_rect0->setImage(*shared.rect0);
    _pimpl->imv_rect1->setImage(*shared.rect1);
}

void This::update_disparity()
{
    SharedData& shared = _pimpl->shared_data;
    std::lock_guard<std::mutex> lock(shared.padlock);
    _pimpl->imv_disparity->setImage(*shared.disparity);
}

void This::update_point_cloud()
{
    {
        SharedData& shared = _pimpl->shared_data;
        std::lock_guard<std::mutex> lock(shared.padlock);
        _pimpl->cloud = shared.pts_3d;
    }    
    _pimpl->cloud_centre = Vector3d(0, 0, 0);
    for(const auto& X: _pimpl->cloud)
        _pimpl->cloud_centre += X;
    _pimpl->cloud_centre /= double(_pimpl->cloud.size());
}

// ---------------------------------------------------------------------- render

void This::render()
{
    const Vector3d centre = _pimpl->cloud_centre;

    auto vertex = [&] (const Vector3d& X) {
        Vector3d p = X - centre;
        glVertex3d(p(0), p(1), p(2));
    };

    glColor3d(1.0, 0.0, 0.0);
    glPushMatrix();
    glBegin(GL_POINTS);
    for(const auto& X: _pimpl->cloud)
        vertex(X);
    glEnd();
    glPopMatrix();
}

// --------------------------------------------------------------------- make_ui

void This::Pimpl::make_ui()
{   
    // Main window is a control-panel and tab-widget side by side

    // Helper functions
    


    { // -- (*) -- control panel
        auto form = new QWidget;
        {
            auto layout = new QFormLayout;
            layout->addRow("Image 0:", newFileSelect(&le_im0));
            layout->addRow("Image 1:", newFileSelect(&le_im1));
            layout->addRow("", newHLine());
            layout->addRow("apply undistort:", newCb(&cb_apply_undistort));
            layout->addRow("fusiello rect:", newCb(&cb_fusiello_rect));
            layout->addRow("", newHLine());
            layout->addRow("surf min hessian:",newWidget(&le_surf_min_hessian));
            layout->addRow("surf ratio:",newWidget(&le_surf_ratio));
            layout->addRow("surf lmeds:", newCb(&cb_surf_lmeds));
            layout->addRow("", newHLine());
            
            layout->addRow("method:", newWidget(&combo_method));
            layout->addRow("", newHLine());
            layout->addRow("bm/sgbm parameters", new QWidget());
            layout->addRow("min-disparity:", newWidget(&le_min_disparity));
            layout->addRow("num-disparities:", newWidget(&le_num_disparities));
            layout->addRow("SAD-window-size:", newWidget(&le_SAD_window_size));
            layout->addRow("12-max-diff:", newWidget(&le_12_max_diff));
            layout->addRow("prefilter-cap:", newWidget(&le_prefilter_cap));
            layout->addRow("prefilter-size:", newWidget(&le_prefilter_size));
            layout->addRow("tex-threshold:", newWidget(&le_texture_threshold));
            layout->addRow("uniq-ratio:", newWidget(&le_uniqueness_ratio));
            layout->addRow("speckle-window-size:", 
                           newWidget(&le_speckle_window_size));
            layout->addRow("speckle-range:", newWidget(&le_speckle_range));
            layout->addRow("stereo-bm full-DP:", newCb(&cb_full_DP));
            layout->addRow("stereo-bm P1:", newWidget(&le_P1));
            layout->addRow("stereo-bm P2:", newWidget(&le_P2));

            layout->addRow("", newHLine());
            layout->addRow("elas parameters", new QWidget());
            layout->addRow("disp-min", newWidget(&le_elas_disp_min));
            layout->addRow("disp-max", newWidget(&le_elas_disp_max)); 
            layout->addRow("support-threshold", 
                           newWidget(&le_elas_support_threshold));      
            layout->addRow("support-texture", 
                           newWidget(&le_elas_support_texture));        
            layout->addRow("candidate-stepsize", 
                           newWidget(&le_elas_candidate_stepsize));     
            layout->addRow("incon-window-size", 
                           newWidget(&le_elas_incon_window_size));      
            layout->addRow("incon-threshold", 
                           newWidget(&le_elas_incon_threshold));        
            layout->addRow("incon-min-support", 
                           newWidget(&le_elas_incon_min_support));      
            layout->addRow("add-corners", newCb(&cb_elas_add_corners));
            layout->addRow("grid-size", newWidget(&le_elas_grid_size));
            layout->addRow("beta", newWidget(&le_elas_beta));                  
            layout->addRow("gamma", newWidget(&le_elas_gamma));                 
            layout->addRow("sigma", newWidget(&le_elas_sigma));                 
            layout->addRow("sradius", newWidget(&le_elas_sradius));
            layout->addRow("match-texture", newWidget(&le_elas_match_texture));
            layout->addRow("lr-threshold", newWidget(&le_elas_lr_threshold)); 
            layout->addRow("speckle-sim-threshold", 
                           newWidget(&le_elas_speckle_sim_threshold)); 
            layout->addRow("speckle-size", newWidget(&le_elas_speckle_size)); 
            layout->addRow("ipol-gap-width",newWidget(&le_elas_ipol_gap_width));
            layout->addRow("filter-median", newCb(&cb_elas_filter_median)); 
            layout->addRow("filter-adaptive-mean", 
                           newCb(&cb_elas_filter_adaptive_mean));
            layout->addRow("postprocess-only-left", 
                           newCb(&cb_elas_postprocess_only_left)); 
            layout->addRow("subsampling", newCb(&cb_elas_subsampling));

            form->setLayout(layout);
        }

        {
            auto * layout = new QVBoxLayout;
	    layout->addWidget(form);
	    layout->addWidget(newSpacer());

            cpanel = new QWidget;
            cpanel->setLayout(layout);
            cpanel->setFixedWidth(300);
            cpanel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Expanding);
        }
    }

    { // -- (*) -- tab-widget
        tab_widget = new QTabWidget;
        tab_widget->setMovable(true);    
    }

    { // -- (*) -- create important widgets
        auto make_image_viewer = [&] (ImageViewer * * im) {
            *im = new ImageViewer();
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

    { // -- (*) -- im0, im1
        auto * layout = new QVBoxLayout;
        layout->addWidget(imv_im0);
        layout->addWidget(imv_im1);

        auto * wgt = new QWidget;        
        wgt->setLayout(layout);
        tab_widget->addTab(wgt, "Input Images");
    }

    { // -- (*) -- distort0, distort1
        auto * layout = new QVBoxLayout;
        layout->addWidget(imv_undistort0);
        layout->addWidget(imv_undistort1);

        auto * wgt = new QWidget;        
        wgt->setLayout(layout);
        tab_widget->addTab(wgt, "Undistorted");
    }

    { // -- (*) -- surf, and inliers
        auto * layout = new QVBoxLayout;
        layout->addWidget(imv_surf);
        layout->addWidget(imv_inliers);

        auto * wgt = new QWidget;        
        wgt->setLayout(layout);
        tab_widget->addTab(wgt, "Surf & Inliers");
    }

    { // -- (*) -- rectifications
        auto * layout = new QVBoxLayout;
        layout->addWidget(imv_rect0);
        layout->addWidget(imv_rect1);

        auto * wgt = new QWidget;        
        wgt->setLayout(layout);
        tab_widget->addTab(wgt, "Rectifications");
    }

    { // -- (*) -- disparity
        auto * layout = new QVBoxLayout;
        layout->addWidget(imv_disparity);

        auto * wgt = new QWidget;        
        wgt->setLayout(layout);
        tab_widget->addTab(wgt, "Disparity");
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

    // -- (*) -- Initialize 
    {
        const Params& p = shared_data.params;
        std::lock_guard<std::mutex> lock(shared_data.padlock);
        
        // Intrinsics
        if(p.filenames.size() > 0) {
            le_im0->setText(qstr(p.filenames[0]));
            le_im1->setText(qstr(p.filenames[1]));
        }
        cb_apply_undistort->setChecked(p.apply_undistort);

        // Surf
        le_surf_min_hessian->setText(qstr(p.surf_min_hessian));
        le_surf_ratio->setText(qstr(p.surf_dist_ratio));

        // Stereo calib
        cb_surf_lmeds->setChecked(p.surf_lmeds);

        // Rect
        cb_fusiello_rect->setChecked(p.use_fusiello_rect);

        { // Disparity method
            const auto& p = shared_data.params;
            for(uint i = 0; i < DISPARITY_METHOD_MAX; ++i)
                combo_method->addItem(qstr(p.disp_method_name(i)));  
            combo_method->setCurrentIndex(p.disp_method);
        }

        // Disparity
        le_min_disparity->setText(qstr(p.disp_min_disparity));
        le_num_disparities->setText(qstr(p.disp_num_disparities));
        le_SAD_window_size->setText(qstr(p.disp_SAD_window_size));
        le_12_max_diff->setText(qstr(p.disp_12_max_diff));
        le_prefilter_cap->setText(qstr(p.disp_prefilter_cap));
        le_prefilter_size->setText(qstr(p.disp_prefilter_size));
        le_texture_threshold->setText(qstr(p.disp_texture_threshold));
        le_uniqueness_ratio->setText(qstr(p.disp_uniqueness_ratio));
        le_speckle_window_size->setText(qstr(p.disp_speckle_window_size));
        le_speckle_range->setText(qstr(p.disp_speckle_range));
        cb_full_DP->setChecked(p.disp_full_DP);
        le_P1->setText(qstr(p.disp_P1));
        le_P2->setText(qstr(p.disp_P2));

        // Disparity Elas
        const Elas::parameters& elas = p.elas_params;
        le_elas_disp_min->setText(qstr(elas.disp_min));               
        le_elas_disp_max->setText(qstr(elas.disp_max));               
        le_elas_support_threshold->setText(qstr(elas.support_threshold));      
        le_elas_support_texture->setText(qstr(elas.support_texture));        
        le_elas_candidate_stepsize->setText(qstr(elas.candidate_stepsize));
        le_elas_incon_window_size->setText(qstr(elas.incon_window_size));      
        le_elas_incon_threshold->setText(qstr(elas.incon_threshold));        
        le_elas_incon_min_support->setText(qstr(elas.incon_min_support));      
        cb_elas_add_corners->setChecked(elas.add_corners);
        le_elas_grid_size->setText(qstr(elas.grid_size));             
        le_elas_beta->setText(qstr(elas.beta));                  
        le_elas_gamma->setText(qstr(elas.gamma));                 
        le_elas_sigma->setText(qstr(elas.sigma));                 
        le_elas_sradius->setText(qstr(elas.sradius));               
        le_elas_match_texture->setText(qstr(elas.match_texture));         
        le_elas_lr_threshold->setText(qstr(elas.lr_threshold));          
        le_elas_speckle_sim_threshold
            ->setText(qstr(elas.speckle_sim_threshold)); 
        le_elas_speckle_size->setText(qstr(elas.speckle_size));          
        le_elas_ipol_gap_width->setText(qstr(elas.ipol_gap_width));        
        cb_elas_filter_median->setChecked(elas.filter_median);        
        cb_elas_filter_adaptive_mean->setChecked(elas.filter_adaptive_mean);
        cb_elas_postprocess_only_left->setChecked(elas.postprocess_only_left);
        cb_elas_subsampling->setChecked(elas.subsampling);
    }


    // -- (*) -- Wiring
#define CONNECT_TEXT(source, slot)                      \
    { connect(source, SIGNAL(textChanged(QString)),     \
              viewer, SLOT(slot()));  }

#define CONNECT_CB(source, slot)                        \
    { connect(source, SIGNAL(stateChanged(int)),        \
              viewer, SLOT(slot())); }

    // Intrinsics
    CONNECT_TEXT(le_im0, intrinsics_dirty);
    CONNECT_TEXT(le_im1, intrinsics_dirty);
    CONNECT_CB  (cb_apply_undistort, intrinsics_dirty);

    // Surf
    CONNECT_TEXT(le_surf_min_hessian, surf_dirty);
    CONNECT_TEXT(le_surf_ratio, surf_dirty);

    // Stereo calib
    CONNECT_CB  (cb_surf_lmeds, stereo_calibration_dirty);

    // Rect
    CONNECT_CB  (cb_fusiello_rect, rectification_dirty);

    // Disparity
    QObject::connect(combo_method, SIGNAL(activated(int)), 
                     viewer, SLOT(disparity_dirty()));
    CONNECT_TEXT(le_min_disparity, disparity_dirty);
    CONNECT_TEXT(le_num_disparities, disparity_dirty);
    CONNECT_TEXT(le_SAD_window_size, disparity_dirty);
    CONNECT_TEXT(le_12_max_diff, disparity_dirty);
    CONNECT_TEXT(le_prefilter_cap, disparity_dirty);
    CONNECT_TEXT(le_prefilter_size, disparity_dirty);
    CONNECT_TEXT(le_texture_threshold, disparity_dirty);
    CONNECT_TEXT(le_uniqueness_ratio, disparity_dirty);
    CONNECT_TEXT(le_speckle_window_size, disparity_dirty);
    CONNECT_TEXT(le_speckle_range, disparity_dirty);
    CONNECT_CB  (cb_full_DP, disparity_dirty);
    CONNECT_TEXT(le_P1, disparity_dirty);
    CONNECT_TEXT(le_P2, disparity_dirty);

    // Disparity Elas
    CONNECT_TEXT(le_elas_disp_min, disparity_dirty);               
    CONNECT_TEXT(le_elas_disp_max, disparity_dirty);               
    CONNECT_TEXT(le_elas_support_threshold, disparity_dirty);      
    CONNECT_TEXT(le_elas_support_texture, disparity_dirty);        
    CONNECT_TEXT(le_elas_candidate_stepsize, disparity_dirty);     
    CONNECT_TEXT(le_elas_incon_window_size, disparity_dirty);      
    CONNECT_TEXT(le_elas_incon_threshold, disparity_dirty);        
    CONNECT_TEXT(le_elas_incon_min_support, disparity_dirty);      
    CONNECT_CB  (cb_elas_add_corners, disparity_dirty);
    CONNECT_TEXT(le_elas_grid_size, disparity_dirty);             
    CONNECT_TEXT(le_elas_beta, disparity_dirty);                  
    CONNECT_TEXT(le_elas_gamma, disparity_dirty);                 
    CONNECT_TEXT(le_elas_sigma, disparity_dirty);                 
    CONNECT_TEXT(le_elas_sradius, disparity_dirty);               
    CONNECT_TEXT(le_elas_match_texture, disparity_dirty);         
    CONNECT_TEXT(le_elas_lr_threshold, disparity_dirty);          
    CONNECT_TEXT(le_elas_speckle_sim_threshold, disparity_dirty); 
    CONNECT_TEXT(le_elas_speckle_size, disparity_dirty);          
    CONNECT_TEXT(le_elas_ipol_gap_width, disparity_dirty);        
    CONNECT_CB  (cb_elas_filter_median, disparity_dirty);        
    CONNECT_CB  (cb_elas_filter_adaptive_mean, disparity_dirty);
    CONNECT_CB  (cb_elas_postprocess_only_left, disparity_dirty); 
    CONNECT_CB  (cb_elas_subsampling, disparity_dirty);

#undef CONNECT_TEXT
}
