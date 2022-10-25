//
// Created by luke on 25-10-22.
//

#include "string_utils.h"

#include <algorithm>
#include <sstream>

#include "iostream"

namespace Utils {
    /*static*/ std::vector<std::string> StringUtils::divide(const std::string &str, char target) {

        std::stringstream ss(str);
        std::string token;
        std::vector<std::string> words;

        while (std::getline(ss, token, target)) {
            words.emplace_back(token);
        }
        return words;
    }

    /*static*/ std::vector<std::string> StringUtils::divide(const std::string &str, char start, char stop) {
        std::vector<std::string> words = divide(str, stop);
        for (std::string &word: words) {
            remove(word, start);
        }
        return words;
    }

    /*static*/ void StringUtils::remove(std::string &str, const std::vector<char> &targets) {
        for (char target: targets) {
            str.erase(std::remove(str.begin(), str.end(), target), str.end());
        }
    }

    /*static*/ void StringUtils::remove(std::string &str, char target) {
        str.erase(std::remove(str.begin(), str.end(), target), str.end());
    }
}