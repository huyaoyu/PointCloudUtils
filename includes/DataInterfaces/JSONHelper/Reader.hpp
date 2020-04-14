//
// Created by yaoyu on 4/13/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_DATAINTERFACES_JSONHELPER_READER_HPP
#define POINTCLOUDUTILS_INCLUDES_DATAINTERFACES_JSONHELPER_READER_HPP

#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "DataInterfaces/JSON/single_include/nlohmann/json.hpp"
#include "Exception/Common.hpp"

using JSON = nlohmann::json;

static std::shared_ptr<JSON> read_json( const std::string &fn ) {
    std::shared_ptr<JSON> pJson ( new JSON );

    std::ifstream ifs(fn);

    if ( !ifs.good() ) {
        EXCEPTION_FILE_NOT_GOOD(fn)
    }

    ifs >> *pJson;

    return pJson;
}

#endif //POINTCLOUDUTILS_INCLUDES_DATAINTERFACES_JSONHELPER_READER_HPP
