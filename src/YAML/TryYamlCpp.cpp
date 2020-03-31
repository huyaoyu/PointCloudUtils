//
// Created by yaoyu on 3/30/20.
//

#include <fstream>
#include <iostream>
#include <vector>

#include <yaml-cpp/yaml.h>

using namespace std;

int main(int argc, char* argv[]) {
    cout << "Hello, TryYamlCpp! " << endl;

    YAML::Node root;

    root["node0"] = "node0_value";
    root["node1"].push_back(1.0f);
    root["node1"].push_back(2.0f);

    vector<int> indices = { 1, 2, 3, 4, 5 };

    root["node2"] = indices;

    root["node3"]["key"] = 1;
    root["node3"]["value"] = indices;

    YAML::Emitter em;
    em << root;

    cout << em.c_str() << endl;

    ofstream ofs("TestYamlCppOutput.yaml");
    ofs << em.c_str();

    return 0;
}
