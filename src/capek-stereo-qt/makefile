
# ------- Depends on...
# eigen3, opencv (with non-free)
# edit 'config.h' to switch on/off triclops

VERSION ?= debug
CC:=colorgcc

SRCS:=main.cpp shared-data.cpp
QTSR=generic-q-event-filter.cpp image-viewer.cpp gl-visualizer.cpp \
	main-viewer.cpp

CVD:=/usr/local
OPENCV_LIBS:=-L$(CVD)/lib \
	-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d \
	-lopencv_nonfree -lopencv_flann -lopencv_features2d
GLUT_LIBS:=-lGL -lGLU -lglut

DEP_INCLUDES:=-I$(CURDIR)/../libcapek-stereo -I$(CURDIR)/../libelas
DEP_LIBS:=-lcapek-stereo -lelas

USE_TRICLOPS=$(shell cat ../config.h | grep USE_TRICLOPS | awk '{ print $$3 }')
ifeq "$(USE_TRICLOPS)" "1"
POINTGREY_INC=-I/usr/include/triclops -isystem/usr/include/flycapture
POINTGREY_LIB=-ltriclops -lpnmutils -lflycapture -lflycapture2bridge
else
POINTGREY_INC=
POINTGREY_LIB=
endif

QT_INCLUDES=-I$(QTDIR)/include -I$(QTDIR)/include/QtCore -I$(QTDIR)/include/QtWidgets -I$(QTDIR)/include/QtGui -I$(QTDIR)/include/QtOpenGL -I$(QTDIR)/include/QtPrintSupport

QT_LIBS=-Wl,-rpath,$(QTDIR)/lib -L$(QTDIR)/lib -lQt5Core -lQt5Widgets -lQt5Gui -lQt5OpenGL -lQt5PrintSupport

# ----------------------------- 
CPP_SANITIZE:=
CPP_COMP_FLAGS:=-m64 -fPIC -Wall -Wno-unused -fmax-errors=4 -msse2 -mssse3
CPP_DEFINES:=
CPP_INCLUDES:=-I$(CURDIR) $(DEP_INCLUDES) $(QT_INCLUDES) \
	-I/usr/include/eigen3 -I$(CVD)/include $(POINTGREY_INC)

# ----------------------------- build
BUILD_EXE:=1
TARGET:=capek-stereo-qt
CPP_FLAGS:=$(CPP_COMP_FLAGS) $(CPP_DEFINES) $(CPP_INCLUDES)
LINK_FLAGS:= $(DEP_LIBS) -lm -lstdc++ -lpthread \
	$(OPENCV_LIBS) $(GLUT_LIBS) $(POINTGREY_LIB) $(QT_LIBS)

# ----------------------------- options
PCH:=
ADDRESS_SANITIZE:=0

# ----------------------------- dependent libraries
DEPENDS_ON:=../libcapek-stereo

# ----------------------------- 

-include ../master-makefile

