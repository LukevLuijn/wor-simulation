//
// Created by luke on 25-10-22.
//

#include "math_utils.h"

namespace Utils
{
    /*static*/ double MathUtils::toDegrees(double radians) {
        return radians * (180.0 / M_PI);
    }
    /*static*/ double MathUtils::toRadians(double degrees) {
        return degrees * (M_PI / 180.0);
    }
    /*static*/ double MathUtils::map(double value, double inMin, double inMax, double outMin, double outMax) {
        return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
}

