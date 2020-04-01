//
// Created by yaoyu on 3/30/20.
//

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <pcl/common/projection_matrix.h>
#include <pcl/point_types.h>

#include "DataInterfaces/JSON/single_include/nlohmann/json.hpp"
#include "HoleBoundaryDetection/HoleBoundaryDetector.hpp"

using namespace std;

using json = nlohmann::json;

namespace ns {
    struct Hole {
        vector<float> centroid;
        vector<float> normal;
        vector<int> indices;
    };
}

int main(int argc, char* argv[]) {
    cout << "Hello, TryJSON! " << endl;

    vector<ns::Hole> disjointSets;

    ns::Hole h0, h1;
    h0.centroid = { 1.0, 2.0, 3.0 };
    h0.normal   = { 4.0, 5.0, 6.0 };
    h0.indices  = { 7, 8, 9 };

    h1.centroid = { 11.0, 12.0, 13.0 };
    h1.normal   = { 14.0, 15.0, 16.0 };
    h1.indices  = { 17, 18, 19 };

    disjointSets.push_back(h0);
    disjointSets.push_back(h1);

    json jNode;
    jNode["disjointSets"] = json::array();
    jNode["disjointSets"].push_back( {
        {"centroid", h0.centroid},
        {"normal", h0.normal},
        {"indices", h0.indices}
    } );

    jNode["disjointSets"].push_back( {
                                         {"centroid", h1.centroid},
                                         {"normal", h1.normal},
                                         {"indices", h1.indices}
                                     } );

    cout << jNode.dump(2) << endl;

    if ( 1 == argc ) {
        return 0;
    }

    // Load a JSON file.
    std::ifstream ifs(argv[1]);
    json jExt;
    ifs >> jExt;

    const auto nSets = jExt["disjointSets"].size();

    cout << "jExt[\"disjointSets\"].size() = " << nSets << endl;

    cout << "Size of centroid: " << jExt["disjointSets"][nSets-1]["centroid"].size() << endl;
    cout << "Size of normal: " << jExt["disjointSets"][nSets-1]["normal"].size() << endl;
    cout << "Size of indices: " << jExt["disjointSets"][nSets-1]["indices"].size() << endl;

    // Test hole boundary detector.
    pcl::PointCloud<pcl::PointNormal>::Ptr pEquivalentNormal ( new pcl::PointCloud<pcl::PointNormal> );
    vector< vector<int> > sets;
    pcu::read_equivalent_normal_from_json(argv[1], pEquivalentNormal, sets);

    cout << "pEquivalentNormal->size() = " << pEquivalentNormal->size() << endl;
    cout << "pEquivalentNormal->at(0) = " << pEquivalentNormal->at(0) << endl;
    cout << "pEquivalentNormal->at(pEquivalentNormal->size()-1) = " << pEquivalentNormal->at(pEquivalentNormal->size()-1) << endl;
    cout << "sets.size() = " << sets.size() << endl;
    cout << "sets[sets.size()-1] = " << endl;

    for ( const auto& c : sets[ sets.size() -1 ] ) {
        cout << c << ", ";
    }

    cout << endl;

    return 0;
}