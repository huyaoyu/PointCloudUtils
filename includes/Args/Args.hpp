//
// Created by yaoyu on 3/16/20.
//

#ifndef POINTCLOUDUTILS_ARGS_HPP
#define POINTCLOUDUTILS_ARGS_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include "Exception/Common.hpp"

struct args_validation_failed : virtual exception_common_base {};

#define EXCEPTION_INVALID_ARGUMENTS(args) \
    {\
        std::stringstream args##_ss;\
        args##_ss << "Arguments validation failed. " << std::endl \
                  << args << std::endl;\
        BOOST_THROW_EXCEPTION( args_validation_failed() << ExceptionInfoString(args##_ss.str()) );\
    }

#define EXCEPTION_INVALID_ARGUMENTS_IN_CLASS() \
    {\
        std::stringstream ss; \
        ss << "Arguments validation failed. \n" << *this << "\n"; \
        BOOST_THROW_EXCEPTION( args_validation_failed() << ExceptionInfoString(ss.str()) );\
    }

#define MAIN_COMMON_LINES(argc, argv, args) \
    Args args; \
    parse_args(argc, argv, args); \
    std::cout << "args: " << std::endl; \
    std::cout << args << std::endl;

#define MAIN_COMMON_LINES_ONE_CLASS(argc, argv, args) \
    Args args; \
    args.parse_args(argc, argv); \
    std::cout << "args: \n" << args << std::endl;

/**
 * This function is copied from
 * https://www.boost.org/doc/libs/1_60_0/libs/program_options/example/options_description.cpp
 *
 * @tparam T
 * @param os
 * @param v
 * @return
 */
template<class T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v)
{
    copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " "));
    return os;
}

template <typename T>
void show_numeric_vector(const std::vector<T>& v, const std::string& delimiter=",") {
    for ( T c : v ) {
        std::cout << c << delimiter;
    }
}

template <typename T>
void show_vector_as_list(const std::vector<T>& v,
        const std::string& delimiter="\n", const std::string& prefix="", const std::string& suffix="") {
    for ( T c : v ) {
        std::cout << prefix << c << suffix << delimiter;
    }
}

template<typename T>
std::vector<T> extract_number_from_string(const std::string& s, const int expected, const std::string& delimiter=",") {

    if ( expected <= 0 ) {
        std::stringstream ss;
        ss << "Exprected number must be positive. expected = " << expected << ". ";
        throw std::runtime_error(ss.str());
    }

    // Split the string.
    std::vector<std::string> splitString;
    boost::split(splitString, s, boost::is_any_of(delimiter));

    if ( splitString.size() != expected ) {
        std::stringstream ss;
        ss << "Wrong number of split strings (" << splitString.size() << "). "
           << "Expecting " << expected << ". ";
        throw std::runtime_error(ss.str());
    }

    // Convert the strings into numbers.
    T number;
    std::stringstream ss;
    std::vector<T> numbers;

    for ( auto& fs : splitString ) {
        ss.str(""); ss.clear(); ss << fs;
        ss >> number;
        numbers.push_back(number);
    }

    return numbers;
}

bool validate_string_with_trimming(const std::string& s, const std::vector<std::string>& validList) {
    // Make a copy of the trimmed s.
    auto ts = boost::algorithm::trim_copy(s);

    bool flagFound = false;

    for ( std::string v : validList ) {
        if ( ts == boost::algorithm::trim_copy(v) ) {
            flagFound = true;
            break;
        }
    }

    return flagFound;
}

#endif //POINTCLOUDUTILS_ARGS_HPP
