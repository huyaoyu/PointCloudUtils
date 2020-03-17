//
// Created by yaoyu on 3/16/20.
//

#ifndef POINTCLOUDUTILS_FILESYSTEM_HPP
#define POINTCLOUDUTILS_FILESYSTEM_HPP

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>

std::vector<std::string> read_file_list(const std::string& fn)
{
    std::vector<std::string> fList;
    std::string line;

    std::ifstream ifs(fn);

    if ( ifs.is_open() ) {
        std::size_t count = 0;

        while( std::getline(ifs, line) ) {
            count++;

            boost::algorithm::trim(line);

            if ( !line.empty() ) {
                fList.push_back(boost::algorithm::trim_copy(line));
            } else {
                std::cout << "Empty line at Line " << count << std::endl;
            }
        }
    } else {
        std::stringstream ss;
        ss << "File " << fn << " not opened.";
        throw(std::runtime_error(ss.str()));
    }

    return fList;
}

#endif //POINTCLOUDUTILS_FILESYSTEM_HPP
