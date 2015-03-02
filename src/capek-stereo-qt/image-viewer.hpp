
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

class ImageViewer : public QMainWindow
{
    Q_OBJECT
    ;

public:
    ImageViewer(QString title = "Image Viewer", QWidget * parent = nullptr);
							
public slots:
    void openFile(QString filename, bool feedback = false);
    void setImage(const QImage& image);
    void open();
    void zoomIn();
    void zoomOut();
    void normalSize();
    void fitToWindow();

    bool onKeyPress(QKeyEvent * event);

    // Click-coords ==> image-coords
    QPoint translate(QPoint pos) const;

signals:
    void sendMousePress(QMouseEvent * event);

protected:
    void mousePressEvent(QMouseEvent * event);
    void mouseMoveEvent(QMouseEvent * event);

private:
    void pageImage(int deltaPages, bool direction, bool byPage);
    void scaleImage(double factor);
    void adjustScrollBar(QScrollBar * scrollBar, double factor);
    bool okayToDisplay() const;

    QLabel * imageLabel;
    QScrollArea * scrollArea;
    double scaleFactor;

    QImage image;

    QPoint dragOffset;
};

