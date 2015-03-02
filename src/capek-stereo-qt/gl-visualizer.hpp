
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
#include <QGLWidget>
#include <QPoint>
#include <QColor>

class GLVisualizer : public QGLWidget
{
    Q_OBJECT
    ;
private:

    void make_ui();

    GLVisualizer(const GLVisualizer&) = delete;
    void operator=(const GLVisualizer&) = delete;

public:

    GLVisualizer(QWidget * parent = nullptr);
    virtual ~GLVisualizer();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;
			  
    std::function<void ()> onDrawThunk;
    bool draw_gl_axis;
    bool draw_3d_cube;
    bool capture_frames;
    bool rotating;

public slots:

    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);
    void setZoom(double scale);

    void zoomIn();
    void zoomOut();

    void setCapture(bool capture);
    void setDrawAxis(bool draw);
    void setDraw3dCube(bool draw);
    void setRotating(bool rotate);
    void setRotateDelta(double delta);

signals:

    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);
    void zoomChanged(double scale);

protected:

    void initializeGL();
    void paintGL();
    void resizeGL(int width, int height);

    virtual void mousePressEvent(QMouseEvent * event);
    virtual void mouseMoveEvent(QMouseEvent * event);
    virtual void keyPressEvent(QKeyEvent * event);
    virtual void wheelEvent(QWheelEvent * event);

public:
    class Pimpl;
private:    
    Pimpl * pimpl;  

};

