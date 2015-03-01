
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

// -------------------------------------------- To/from homogeneous co-ordinates

inline Vector2d from_hom3(const Vector3d& X);
inline Vector3d from_hom4(const Vector4d& X);
inline Vector3d to_hom3(const Vector2d& X);
inline Vector4d to_hom4(const Vector3d& X);

inline void normalize_hom3_point(Vector3d& X);
inline void normalize_hom3_line(Vector3d& X);

// --------------------------------------------------------- Working with angles

inline double normalise_angle(double theta);

// ---------------------------------------------- Closest point between 3d lines

Vector3d intersect_rays(const Vector3d& u0, const Vector3d& u1,
                        const Vector3d& v0, const Vector3d& v1);
















// ------------------------------------------------------------- Implementations

inline Vector2d from_hom3(const Vector3d& X) 
{ 
    return Vector2d(X(0)/X(2),X(1)/X(2)); 
}

inline Vector3d from_hom4(const Vector4d& X)
{ 
    return Vector3d(X(0)/X(3),X(1)/X(3),X(2)/X(3)); 
}

inline Vector3d to_hom3(const Vector2d& X) 
{ 
    return Vector3d(X(0),X(1),1.0); 
}

inline Vector4d to_hom4(const Vector3d& X)
{
    return Vector4d(X(0),X(1),X(2),1.0);
}

inline double normalise_angle(double theta)
{
    if(theta >= M_PI)
	return fmod(theta + M_PI, 2.0*M_PI) - M_PI;
    if(theta < -M_PI)
	return fmod(theta - M_PI, 2.0*M_PI) + M_PI;
    return theta;
}

inline void normalize_hom3_point(Vector3d& X)
{
    X /= X(2);
}

inline void normalize_hom3_line(Vector3d& X)
{
    X /= sqrt(X(0)*X(0) + X(1)*X(1));
}


