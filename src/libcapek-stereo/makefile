
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

# ----------------------------- 
CPP_SANITIZE:=
CPP_COMP_FLAGS:=-m64 -fPIC -Wall -Wno-unused -fmax-errors=4 -msse2 -mssse3
CPP_DEFINES:=
CPP_INCLUDES:=-I$(CURDIR) $(BOOST_INCLUDES) -I/usr/include/eigen3 -I$(CVD)/include

# ----------------------------- build 
BUILD_A:=1
TARGET:=capek-stereo
CPP_FLAGS:=$(CPP_COMP_FLAGS) $(CPP_DEFINES) $(CPP_INCLUDES)
LINK_FLAGS:=

# ----------------------------- options
PCH:=stdinc.hpp
MOCD:=
ADDRESS_SANITIZE:=0

# ----------------------------- 

-include ../master-makefile

