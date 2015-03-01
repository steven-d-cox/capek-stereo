
# ------- Depends on...
# eigen3, opencv (with non-free)
# edit 'config.h' to switch on/off triclops

VERSION ?= debug
CC:=colorgcc

SRCS:=main.cpp viewer.cpp

CVD:=/usr/local
OPENCV_LIBS:=-L$(CVD)/lib \
	-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d \
	-lopencv_nonfree -lopencv_flann -lopencv_features2d
GLUT_LIBS:=-lGL -lGLU -lglut

DEP_INCLUDES:=-I$(CURDIR)/../libcapek-stereo
DEP_LIBS:=-lcapek-stereo

USE_TRICLOPS=$(shell cat ../libcapek-stereo/config.h | grep USE_TRICLOPS | awk '{ print $$3 }')
ifeq "$(USE_TRICLOPS)" "1"
POINTGREY_INC=-I/usr/include/triclops -isystem/usr/include/flycapture
POINTGREY_LIB=-ltriclops -lpnmutils -lflycapture -lflycapture2bridge
else
POINTGREY_INC=
POINTGREY_LIB=
endif

# ----------------------------- 
CPP_SANITIZE:=
CPP_COMP_FLAGS:=-m64 -fPIC -Wall -Wno-unused -fmax-errors=4 -msse2 -mssse3
CPP_DEFINES:=
CPP_INCLUDES:=-I$(CURDIR) $(DEP_INCLUDES) $(BOOST_INCLUDES) \
	-I/usr/include/eigen3 -I$(CVD)/include $(POINTGREY_INC)

# ----------------------------- build
BUILD_EXE:=1
TARGET:=capek-stereo
CPP_FLAGS:=$(CPP_COMP_FLAGS) $(CPP_DEFINES) $(CPP_INCLUDES)
LINK_FLAGS:=-lm -lstdc++ -lpthread $(DEP_LIBS) $(BOOST_LIBS) \
	$(OPENCV_LIBS) $(GLUT_LIBS) $(POINTGREY_LIB)

# ----------------------------- options
PCH:=stdinc.hpp
MOCD:=
ADDRESS_SANITIZE:=0

# ----------------------------- dependent libraries
DEPENDS_ON:=../libcapek-stereo

# ----------------------------- 

-include ../master-makefile

