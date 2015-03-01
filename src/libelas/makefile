
# ------- Depends on...
# eigen3, opencv (with non-free)
# edit 'config.h' to switch on/off triclops

VERSION ?= debug
CC:=colorgcc

SRCS:=descriptor.cpp  elas.cpp  filter.cpp  main.cpp  matrix.cpp  triangle.cpp

# ----------------------------- 
CPP_SANITIZE:=
CPP_COMP_FLAGS:=-m64 -fPIC -Wall -Wno-unused -fmax-errors=4 -msse2 -mssse3
CPP_DEFINES:=
CPP_INCLUDES:=-I$(CURDIR)

# ----------------------------- build 
BUILD_A:=1
TARGET:=elas
CPP_FLAGS:=$(CPP_COMP_FLAGS) $(CPP_DEFINES) $(CPP_INCLUDES)
LINK_FLAGS:=

# ----------------------------- options
PCH:=
MOCD:=
ADDRESS_SANITIZE:=0

# ----------------------------- 

-include ../master-makefile

