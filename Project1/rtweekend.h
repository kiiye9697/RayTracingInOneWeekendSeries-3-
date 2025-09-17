#ifndef RTWEEKEND_H
#define RTWEEKEND_H
//COMMENT：
//为了让包含关系不过于复杂，以及一些数学定义并未使用
//我们设计了一个类来存放一些包含文件和常见的数学类。
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <random>

// C++ Std Usings

using std::make_shared;
using std::shared_ptr;

// Constants

const double infinity = std::numeric_limits<double>::infinity();
const double pi = 3.1415926535897932385;

// Utility Functions

inline double degrees_to_radians(double degrees) {
    return degrees * pi / 180.0;
}
inline double random_double() {
    // Returns a random real in [0,1).
    return std::rand() / (RAND_MAX + 1.0);
}

inline double random_double(double min, double max) {
    // Returns a random real in [min,max).
    return min + (max - min) * random_double();
}
inline int random_int(int min, int max) {
    // Returns a random integer in [min,max].
    return int(random_double(min, max + 1));
}

// Common Headers

#include "color.h"
#include "ray.h"
#include "vec3.h"
#include "interval.h"

#endif
