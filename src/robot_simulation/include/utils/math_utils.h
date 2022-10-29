//
// Created by luke on 25-10-22.
//

#ifndef WOR_SIMULATION_MATH_UTILS_H
#define WOR_SIMULATION_MATH_UTILS_H

#include <cmath>

namespace Utils {
    class MathUtils {
    public:
        /**
         * @brief convert degrees to radians
         *
         * @param radians
         *
         * @return degrees
         */
        static double toDegrees(double radians);
        /**
         * @brief convert radians to degrees
         *
         * @param degrees
         *
         * @return radians
         */
        static double toRadians(double degrees);
        /**
         * @brief maps a value from range to range.
         *
         * @param value to be mapped value.
         * @param inMin from-range minimum value.
         * @param inMax from-range maximum value.
         * @param outMin to-range minimum value.
         * @param outMax to-range maximum value.
         *
         * @return mapped value
         */
        static double map(double value, double inMin, double inMax, double outMin, double outMax);
        /**
         * @brief checks if value is between to values.
         *
         * @tparam T typename
         * @param min minimal value.
         * @param max maximum value.
         * @param value value to be checked.
         *
         * @return true if value is between min and max, false if not.
         */
        template<typename T>
        static bool between(T min, T max, T value) {
            return ((value - max) * (value - min) <= 0);
        }
    };
}


#endif //WOR_SIMULATION_MATH_UTILS_H
