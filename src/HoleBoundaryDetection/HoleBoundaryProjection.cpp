//
// Created by yaoyu on 3/30/20.
//

#include <iostream>
#include <string>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

// Namespaces.
namespace bpo = boost::program_options;

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

class Args
{
public:
    Args() = default;

    ~Args() = default;

    bool validate() {
        bool flag = true;

        return flag;
    }

    friend std::ostream& operator<<(std::ostream& out, const Args& args) {
        out << Args::AS_IN_CLOUD << ": " << args.inFile << std::endl;
        out << Args::AS_IN_SETS_JSON << ": " << args.injson << std::endl;
        out << Args::AS_OUT_DIR << ": " << args.outDir << std::endl;

        return out;
    }

public:
    static const std::string AS_IN_CLOUD; // AS stands for argument string
    static const std::string AS_IN_SETS_JSON;
    static const std::string AS_OUT_DIR;

public:
    std::string inFile; // The input point cloud file.
    std::string injson;
    std::string outDir; // The output directory.
};

const std::string Args::AS_IN_CLOUD = "incloud";
const std::string Args::AS_OUT_DIR = "outdir";
const std::string Args::AS_IN_SETS_JSON = "injson";

static void parse_args(int argc, char* argv[], Args& args) {
    try
    {
        bpo::options_description optDesc("Find the hole boundary points of a point cloud.");

        optDesc.add_options()
                ("help", "Produce help message.")
                (Args::AS_IN_CLOUD.c_str(), bpo::value< std::string >(&(args.inFile))->required(), "Input point cloud.")
                (Args::AS_IN_SETS_JSON.c_str(), bpo::value< std::string >(&args.injson)->required(), "The JSON file stores the disjoint sets.")
                (Args::AS_OUT_DIR.c_str(), bpo::value< std::string >(&(args.outDir))->required(), "Output directory.");

        bpo::positional_options_description posOptDesc;
        posOptDesc.add(
                Args::AS_IN_CLOUD.c_str(), 1
                ).add(Args::AS_IN_SETS_JSON.c_str(), 1
                ).add(Args::AS_OUT_DIR.c_str(), 1);

        bpo::variables_map optVM;
        bpo::store(bpo::command_line_parser(argc, argv).
                options(optDesc).positional(posOptDesc).run(), optVM);
        bpo::notify(optVM);
    }
    catch(std::exception& e)
    {
        std::cout << e.what() << std::endl;
        throw(e);
    }
}

int main(int argc, char* argv[]) {
    std::cout << "Hello, HoleBoundaryProjection! " << std::endl;

    // Handle the command line.
    Args args;
    parse_args(argc, argv, args);

    std::cout << "args: " << std::endl;
    std::cout << args << std::endl;

    return 0;
}