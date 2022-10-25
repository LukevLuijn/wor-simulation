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
        static std::vector<std::string> divide(const std::string &str, char target);

        static std::vector<std::string> divide(const std::string &str, char start, char stop);

        static void remove(std::string &str, const std::vector<char> &targets);

        static void remove(std::string &str, char target);
    };
}

#endif //WOR_SIMULATION_STRING_UTILS_H
