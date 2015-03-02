
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
#include "gl-visualizer.hpp"

#include <sys/time.h>

#include <QOpenGLPaintDevice>
#include <QPixmap>

#include <GL/gl.h>	
#include <GL/glu.h>
  
#define This GLVisualizer

struct This::Pimpl 
{
    Pimpl(This * parent_) : parent(parent_),
			    capture_counter(0),
			    animationTimer(new QTimer),
			    xRot(0), yRot(0), zRot(0),
			    lastPos(),
			    translate(0, 0, 0), 
			    scale(1, 1, 1),
			    fov(25.0),
			    znear(0.5), zfar(500.0),
			    eye(0, 0, 2), centre(0, 0, 0), up(0, 1, 0),
			    delta(0.0), 
			    camera_speed(16.0),
			    time0(0), last_delta(0)
    {}

    This * parent;

    uint capture_counter; 

    QTimer * animationTimer;

    // Mouse control changes viewing angle
    int xRot;
    int yRot;
    int zRot;
    QPoint lastPos;

    // Translate and scale object
    Vector3d translate;
    Vector3d scale; // in the x, y, and z dimensions

    double fov; // options().info.fov() * 180.0 / M_PI;
    double znear;
    double zfar;

    Vector3d eye, centre, up;

    double delta;
    double camera_speed; // 1/Hz... i.e, num of seconds per complete revolution
    int64 time0;
    int64 last_delta;

    double radius() const { return (eye - centre).norm(); }
    void set_radius(double r) 
    { 
        Vector3d n = eye - centre;
        eye = centre + r * n / n.norm(); 
    }
};

int64 milli_clock()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return int64(1000) * int64(tv.tv_sec) + int64(tv.tv_usec) / 1000;
}

void glVertex3(const Vector3d& X)
{
    glVertex3d(X(0), X(1), X(2));
}

void glDrawAxis(double line_len = 2.0, double line_width = 4.0)
{
    glPushMatrix();
    glScaled(line_len, line_len, line_len);
    Vector3d axis_origin(0.0, 0.0, 0.0);
    glLineWidth(line_width);
    glBegin(GL_LINES);
    glColor3d(1, 0, 0);
    glVertex3(axis_origin);
    glVertex3(axis_origin + Vector3d(1.0, 0.0, 0.0));
    glColor3d(0, 1, 0);
    glVertex3(axis_origin);
    glVertex3(axis_origin + Vector3d(0.0, 1.0, 0.0));
    glColor3d(0, 0, 1);
    glVertex3(axis_origin);
    glVertex3(axis_origin + Vector3d(0.0, 0.0, 1.0));
    glEnd();
    glLineWidth(1.0);
    glPopMatrix();
}

void glDrawNiceCube()
{
    glBegin( GL_QUADS );                 // Draw A Quad                      
    glColor3f(   0.0f,  1.0f,  0.0f ); // Set The Color To Green           
    glVertex3f(  1.0f,  1.0f, -1.0f ); // Top Right Of The Quad (Top)      
    glVertex3f( -1.0f,  1.0f, -1.0f ); // Top Left Of The Quad (Top)       
    glVertex3f( -1.0f,  1.0f,  1.0f ); // Bottom Left Of The Quad (Top)    
    glVertex3f(  1.0f,  1.0f,  1.0f ); // Bottom Right Of The Quad (Top)   

    glColor3f(   1.0f,  0.5f,  0.0f ); // Set The Color To Orange          
    glVertex3f(  1.0f, -1.0f,  1.0f ); // Top Right Of The Quad (Botm)     
    glVertex3f( -1.0f, -1.0f,  1.0f ); // Top Left Of The Quad (Botm)      
    glVertex3f( -1.0f, -1.0f, -1.0f ); // Bottom Left Of The Quad (Botm)   
    glVertex3f(  1.0f, -1.0f, -1.0f ); // Bottom Right Of The Quad (Botm)  

    glColor3f(   1.0f,  0.0f,  0.0f ); // Set The Color To Red             
    glVertex3f(  1.0f,  1.0f,  1.0f ); // Top Right Of The Quad (Front)    
    glVertex3f( -1.0f,  1.0f,  1.0f ); // Top Left Of The Quad (Front)     
    glVertex3f( -1.0f, -1.0f,  1.0f ); // Bottom Left Of The Quad (Front)  
    glVertex3f(  1.0f, -1.0f,  1.0f ); // Bottom Right Of The Quad (Front) 

    glColor3f(   1.0f,  1.0f,  0.0f ); // Set The Color To Yellow          
    glVertex3f(  1.0f, -1.0f, -1.0f ); // Bottom Left Of The Quad (Back)   
    glVertex3f( -1.0f, -1.0f, -1.0f ); // Bottom Right Of The Quad (Back)  
    glVertex3f( -1.0f,  1.0f, -1.0f ); // Top Right Of The Quad (Back)     
    glVertex3f(  1.0f,  1.0f, -1.0f ); // Top Left Of The Quad (Back)      

    glColor3f(   0.0f,  0.0f,  1.0f ); // Set The Color To Blue            
    glVertex3f( -1.0f,  1.0f,  1.0f ); // Top Right Of The Quad (Left)     
    glVertex3f( -1.0f,  1.0f, -1.0f ); // Top Left Of The Quad (Left)      
    glVertex3f( -1.0f, -1.0f, -1.0f ); // Bottom Left Of The Quad (Left)   
    glVertex3f( -1.0f, -1.0f,  1.0f ); // Bottom Right Of The Quad (Left)  

    glColor3f(   1.0f,  0.0f,  1.0f ); // Set The Color To Violet          
    glVertex3f(  1.0f,  1.0f, -1.0f ); // Top Right Of The Quad (Right)    
    glVertex3f(  1.0f,  1.0f,  1.0f ); // Top Left Of The Quad (Right)     
    glVertex3f(  1.0f, -1.0f,  1.0f ); // Bottom Left Of The Quad (Right)  
    glVertex3f(  1.0f, -1.0f, -1.0f ); // Bottom Right Of The Quad (Right) 
    glEnd( );                            // Done Drawing The Quad  
}

This::This(QWidget * parent) : QGLWidget(parent)
{
    onDrawThunk = [] () { };
    draw_gl_axis = false;
    draw_3d_cube = false;
    capture_frames = false;
    rotating = false;

    pimpl = new This::Pimpl(this);
    make_ui();

    auto animationTimer = pimpl->animationTimer;
    animationTimer->setSingleShot(false);
    connect(animationTimer, SIGNAL(timeout()), this, SLOT(updateGL()));
    animationTimer->start(1000.0 / 33.0); // i.e. 33 frames per second
    setRotating(false);
}

This::~This()
{
    delete pimpl;
}

QSize This::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize This::sizeHint() const
{
    return QSize(400, 400);
}

void This::make_ui()
{

}

static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360 * 16)
        angle -= 360 * 16;
}

void This::setXRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != pimpl->xRot) {
        pimpl->xRot = angle;
        emit xRotationChanged(angle);
        updateGL();
    }
}

void This::setYRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != pimpl->yRot) {
        pimpl->yRot = angle;
        emit yRotationChanged(angle);
        updateGL();
    }
}

void This::setZRotation(int angle)
{
    qNormalizeAngle(angle);
    if (angle != pimpl->zRot) {
        pimpl->zRot = angle;
        emit zRotationChanged(angle);
        updateGL();
    }
}

void This::setZoom(double scalev)
{
    const double min_scale = 0.001;
    const double max_scale = 1000.0;
    if(scalev < min_scale) {
        printf("WARNING: scale (%g) less than floor (%g), using floor\n",
               scalev, min_scale);
	scalev = min_scale;
    } 
    if(scalev > max_scale) {
	printf("WARNING: scale (%g) greater than ceil (%g), using ceil\n",
               scalev, max_scale);
	scalev = max_scale;
    } 
    pimpl->scale = (pimpl->scale / pimpl->scale.norm()) * scalev;

    emit(zoomChanged(scalev));
    updateGL();
}

void This::setCapture(bool capture)
{
    if(capture && !capture_frames) 
	pimpl->capture_counter = 0;       
    capture_frames = capture;
}

void This::setDrawAxis(bool draw)
{
    draw_gl_axis = draw;
}

void This::setDraw3dCube(bool draw)
{
    draw_3d_cube = draw;
}

void This::setRotating(bool rotate)
{
    if(rotate && !rotating) 
	pimpl->time0 = milli_clock();
    rotating = rotate;
}

void This::setRotateDelta(double delta)
{
    pimpl->delta = delta;
}

void This::zoomIn()
{
    setZoom(pimpl->scale.norm() * 1.1);
}

void This::zoomOut()
{
    setZoom(pimpl->scale.norm() * 0.9);
}

void This::initializeGL()
{
    glShadeModel(GL_SMOOTH);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_CULL_FACE);
}

void This::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

    { // Locate the current eye position...
	Pimpl& P = *pimpl;
	const double radius = (P.centre - P.eye).norm() * P.scale.norm();
	Vector3d centre = P.centre;
	Vector3d at = (P.centre - P.eye);
        at /= at.norm();
	Vector3d up = P.up;
        up /= up.norm();
	Vector3d right = at.cross(up);

	auto time_now = milli_clock();

	if(rotating) {
	    P.last_delta = time_now - P.time0;
	    const double ellapsed_ms = P.last_delta;
	    const double speed = P.camera_speed; // 4s for one revolution
	    P.delta = fmod(ellapsed_ms*0.001, speed) / speed;
	}
	assert(P.delta >= 0.0 && P.delta <= 1.0);
	const double theta = 2.0 * (P.delta-0.5) * M_PI; // 2pi radians  
	Vector3d eye = centre + at*cos(theta)*radius + right*sin(theta)*radius;
	 
	gluLookAt(0.0, 0.0, 0.0,
		  centre(0) - eye(0), centre(1) - eye(1), centre(2) - eye(2),
		  up(0), up(1), up(2));

	glTranslated(-eye(0), -eye(1), -eye(2));

	glRotated(double(pimpl->xRot) / 16.0, 1.0, 0.0, 0.0);
	glRotated(double(pimpl->yRot) / 16.0, 0.0, 1.0, 0.0);
	glRotated(double(pimpl->zRot) / 16.0, 0.0, 0.0, 1.0);	
    }

    // -- Draw the axis
    if(draw_gl_axis)
	glDrawAxis();
    
    // Draw a quad -- testing
    if(draw_3d_cube)
	glDrawNiceCube();

    if(true)
	onDrawThunk();

    // Save a frame to disk (if requested)
    if(capture_frames) { 
        const uint buflen = 2048;

	static uint capture_counter = 0;
	static int t0 = 0;
	static int frame_counter = 0;
        static char buffer[buflen];
        static uint8 * pixels = nullptr;
        static uint pixels_sz = 0;

	capture_counter = (pimpl->capture_counter)++;
	printf("capture frame = %d\n", capture_counter);
	// convert -delay 10 -loop 0 inputfiles*.png animaion.gif

	string outdir = "/tmp/gl-capture";
        {
	    // Process a screen capture
	    auto sz = size();
	    auto w = sz.width();
	    auto h = sz.height();
            uint n_pixels = 4 * w * h;
            if(pixels_sz < n_pixels) {
                pixels_sz = n_pixels;
                delete[] pixels;
                pixels = new uint8[pixels_sz];
            }

	    glReadPixels(0, 0, w, h, GL_BGRA, GL_UNSIGNED_BYTE, pixels);

            snprintf(buffer, buflen, "/tmp/gl-capture-%06d.png", 
                     capture_counter);

	    // Save the image
            QImage im(pixels, w, h, QImage::Format_ARGB32);
            im.save(buffer);
	}
    }
}

void This::resizeGL(int width, int height)
{
    auto fov = pimpl->fov;
    auto znear = pimpl->znear;
    auto zfar = pimpl->zfar;

    // Protect against a divide by zero 
    if(height == 0) height = 1;
 
    // Setup our viewport. 
    glViewport(0, 0, (GLint) width, (GLint) height);

    // change to the projection matrix and set our viewing volume. 
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fov, float(width) / float(height), znear, zfar);

    // Make sure we're chaning the model view and not the projection 
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void This::mousePressEvent(QMouseEvent *event)
{
    pimpl->lastPos = event->pos();
}

void This::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - pimpl->lastPos.x();
    int dy = event->y() - pimpl->lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(pimpl->xRot + 8 * dy);
        setYRotation(pimpl->yRot + 8 * dx);
    } else if (event->buttons() & Qt::RightButton) {
        setXRotation(pimpl->xRot + 8 * dy);
        setZRotation(pimpl->zRot + 8 * dx);
    }
    pimpl->lastPos = event->pos();
}

void This::keyPressEvent(QKeyEvent * event)
{
  
}

void This::wheelEvent(QWheelEvent * event)
{
    auto d_angle = event->angleDelta();
    if(d_angle.y() < 0)
	zoomIn();
    if(d_angle.y() > 0)
	zoomOut();
}

