//
// Created by luke on 25-10-22.
//

#ifndef WOR_SIMULATION_MATH_UTILS_H
#define WOR_SIMULATION_MATH_UTILS_H

#include <cmath>

namespace Utils {
    class MathUtils {
    public:
        static double toDegrees(double radians);

        static double toRadians(double degrees);

        static double map(double value, double inMin, double inMax, double outMin, double outMax);
    };
}


#endif //WOR_SIMULATION_MATH_UTILS_H
