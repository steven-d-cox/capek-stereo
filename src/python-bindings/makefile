

# ------- Depends on...
# eigen3, opencv (with non-free)
# edit 'config.h' to switch on/off triclops

VERSION ?= debug
CC:=colorgcc

SRCS:=params.cpp stereo-camera-system.cpp \
	math-utils.cpp string-utils.cpp \
	matrix-utils.cpp camera-matrix-utils.cpp \
	intrinsic.cpp surf-point-pairs.cpp disparity.cpp \
	nelder-mead.cpp 

CVD:=/usr/local

DEP_INCLUDES:=-I$(CURDIR)/../libelas

USE_TRICLOPS=$(shell cat ../config.h | grep USE_TRICLOPS | awk '{ print $$3 }')
ifeq "$(USE_TRICLOPS)" "1"
POINTGREY_INC=-I/usr/include/triclops -isystem/usr/include/flycapture
else
POINTGREY_INC=
endif

# ----------------------------- 
CPP_SANITIZE:=
CPP_COMP_FLAGS:=-m64 -fPIC -Wall -Wno-unused -fmax-errors=4 -msse2 -mssse3
CPP_DEFINES:=
CPP_INCLUDES:=-I$(CURDIR) $(DEP_INCLUDES) \
	-I/usr/include/eigen3 -I$(CVD)/include $(POINTGREY_INC)

# ----------------------------- build 
BUILD_A:=1
TARGET:=capek-stereo
CPP_FLAGS:=$(CPP_COMP_FLAGS) $(CPP_DEFINES) $(CPP_INCLUDES)
LINK_FLAGS:=

# ----------------------------- options
PCH:=stdinc.hpp
MOCD:=
ADDRESS_SANITIZE:=0

# ----------------------------- dependent libraries
DEPENDS_ON:=../libelas

# ----------------------------- 

-include ../master-makefile

