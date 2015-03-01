# capek-stereo



To build:

  cd src/capek-stereo
  make -j4 VERSION=release
  
Dependencies
  
  Must install eigen3 and opencv (with non-free libaraies).
  triclops can be used to find the disparity map, but must be installed as well.
  Once triclops is installed, edit src/config.h to enable
  
