
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
#include "image-viewer.hpp"

#include "generic-q-event-filter.hpp"

ImageViewer::ImageViewer(QString title, QWidget * parent) : QMainWindow(parent)
{
    imageLabel = new QLabel;
    imageLabel->setBackgroundRole(QPalette::Base);
    imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    imageLabel->setScaledContents(false);
    imageLabel->setAlignment(Qt::AlignCenter);

    scrollArea = new QScrollArea;
    scrollArea->setWidget(imageLabel);
    scrollArea->setAlignment(Qt::AlignCenter);
    setCentralWidget(scrollArea);

    setWindowTitle(title);
    setObjectName(title);

    auto me = this;
    auto filter = new GenericQEventFilter(this, 
					  [=] (QObject *obj, QEvent *event) {
	    if(event->type() == QEvent::KeyPress) 
		return me->onKeyPress(dynamic_cast<QKeyEvent*>(event));
	    return false;
	});
    this->installEventFilter(filter);
}

void ImageViewer::open()
{
    QString fileName = QFileDialog::getOpenFileName(this,
						    tr("Open File"), 
						    QDir::currentPath());
    openFile(fileName, true);
}

					   
void ImageViewer::openFile(QString fileName, bool feedback)
{
    if (!fileName.isEmpty()) {
	QImage image(fileName);
	if (image.isNull()) {
	    if(feedback)
		QMessageBox::information(this, 
					 tr("Image Viewer"),
					 tr("Cannot load %1.").arg(fileName));
	    return;
	}
	setImage(image);
    }
}

void ImageViewer::setImage(const QImage& image)
{
    bool preserve = (image.size() == this->image.size());
    this->image = image;
    if(preserve) 
	scaleImage(1.0);
    else 
	normalSize();
}

void ImageViewer::zoomIn()
{
    scaleImage(1.25);
}

void ImageViewer::zoomOut()
{
    scaleImage(0.8);
}

void ImageViewer::normalSize()
{
    scaleFactor = 1.0;
    scaleImage(1.0);
}

bool ImageViewer::okayToDisplay() const
{
    auto sz = size();
    return !image.isNull() && sz.width() > 0 && sz.height() > 0;
}

void ImageViewer::fitToWindow()
{
    if(!okayToDisplay())
	return;

    auto sz = size();
    
    double scale_w = double(sz.width()) / double(image.width());
    double scale_h = double(sz.height()) / double(image.height());
    
    scaleFactor = std::min(scale_w, scale_h);
    scaleImage(1.0);
}


void ImageViewer::scaleImage(double factor)
{
    if(!okayToDisplay())
	return;

    scaleFactor *= factor;

    uint new_w = std::max(int(double(image.width() * scaleFactor)), 128);
    uint new_h = std::max(int(double(image.height() * scaleFactor)), 128);    
    auto pixmap = QPixmap::fromImage(image);

    imageLabel->setPixmap(pixmap.scaled(new_w, new_h, Qt::IgnoreAspectRatio, 
					Qt::FastTransformation));
    imageLabel->adjustSize();

    adjustScrollBar(scrollArea->horizontalScrollBar(), factor);
    adjustScrollBar(scrollArea->verticalScrollBar(), factor);
}
 
void ImageViewer::adjustScrollBar(QScrollBar * scrollBar, double factor)
{
    scrollBar->setValue(int(factor * scrollBar->value() 
			    + ((factor - 1) * scrollBar->pageStep()/2)));
}

void ImageViewer::pageImage(int deltaPages, bool direction, bool byPage)
{
    auto scrollBar = direction ? scrollArea->horizontalScrollBar() 
	: scrollArea->verticalScrollBar();
    auto step = byPage ? scrollBar->pageStep() : 1;
    scrollBar->setValue(scrollBar->value() + deltaPages * step);
}

bool ImageViewer::onKeyPress(QKeyEvent * event)
{
    bool handled = true;
    switch(event->key()) {
    case Qt::Key_Plus: zoomIn(); break;
    case Qt::Key_Minus: zoomOut(); break;
    case Qt::Key_Space: normalSize(); break;
    case Qt::Key_F: fitToWindow(); break;
    case Qt::Key_PageUp: pageImage(1, true, true); break;
    case Qt::Key_PageDown: pageImage(-1, true, true); break;
    default: handled = false;
    }
    if(handled) 
	event->ignore();
    return handled;
}

void ImageViewer::mousePressEvent(QMouseEvent *event)
{
    dragOffset = event->pos();
    emit(sendMousePress(event));
}

void ImageViewer::mouseMoveEvent(QMouseEvent *event)
{
    if(event->buttons() & Qt::LeftButton) {
	auto delta = event->pos() - dragOffset;
    }
}

QPoint ImageViewer::translate(QPoint pos) const
{
    const QPoint topLeft = imageLabel->pos();
    const double scale = scaleFactor;
    
    int hvalue = 0.49 + (pos.x() - topLeft.x()) / scale;
    int vvalue = 0.49 + (pos.y() - topLeft.y()) / scale;

    return QPoint(hvalue, vvalue);
}

