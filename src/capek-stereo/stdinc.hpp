 
#ifndef CALIBRATE_STDINC_HPP
#define CALIBRATE_STDINC_HPP

#include "config.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdint.h>

#include <cctype>
#include <deque>
#include <vector>
#include <iostream>
#include <exception>
#include <stdexcept>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>

typedef int8_t  int8;
typedef int16_t int16;
typedef int32_t int32;
typedef int64_t int64;

typedef uint8_t  uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef uint64_t uint64;

typedef uint32_t uint;

using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

typedef Eigen::Matrix<double, 3, 3> Matrix3d;
typedef Eigen::Matrix<double, 4, 4> Matrix4d;
typedef Eigen::Matrix<double, 3, 4> Matrix34d; 
typedef Eigen::Matrix<double, 8, 1> Vector8d;

using std::string;
using std::vector;
using std::deque;

#endif

