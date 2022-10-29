//
// Created by luke on 25-10-22.
//

#ifndef WOR_SIMULATION_STRING_UTILS_H
#define WOR_SIMULATION_STRING_UTILS_H

#include <string>
#include <vector>

namespace Utils {
    class StringUtils {

    public:
        /**
         * @brief divide string into segments based on target char.
         *
         * @param str string to be divided.
         * @param target target char to divide string with.
         *
         * @return vector containing all substrings.
         */
        static std::vector<std::string> divide(const std::string &str, char target);
        /**
         * @brief divide string into segments based on start and stop char.
         *
         * @param str string to be divided.
         * @param start start char
         * @param stop stop char
         *
         * @return vector containing all substrings.
         */
        static std::vector<std::string> divide(const std::string &str, char start, char stop);
        /**
         * @brief remove all instances of target chars from string.
         *
         * @param str reference to the string.
         * @param targets vector containing all to be removed chars.
         */
        static void remove(std::string &str, const std::vector<char> &targets);
        /**
         * @brief remove all instances of target char from string.
         *
         * @param str reference to the string.
         * @param target to be removed char.
         */
        static void remove(std::string &str, char target);
    };
}

#endif //WOR_SIMULATION_STRING_UTILS_H
