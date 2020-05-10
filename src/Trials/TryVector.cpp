//
// Created by yaoyu on 5/6/20.
//

#include <iostream>
#include <string>
#include <vector>

#define SHOW_VECTOR(v) \
    show_vector(v, #v);

template < typename T >
static void show_vector( const std::vector<T> &v, const std::string &name ) {
    std::cout << name << ": " << std::endl;

    for ( const auto& i : v ) {
        std::cout << i << ", ";
    }
    std::cout << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "Hello, TryVector!" << std::endl;

    // Create a vector.
    std::vector<int> v0 { 0, 1, 2 };
    SHOW_VECTOR(v0)

    // Create a longer vector.
    std::vector<int> v1 { 3, 4, 5, 6 };
    SHOW_VECTOR(v1)

    // Assignment.
    v1 = v0;
    SHOW_VECTOR(v1)

    std::cout << "v1.size() = " << v1.size() << std::endl;

    // Change value in v0.
    v0[0] = -1;
    std::cout << "After v0[0] = -1" << std::endl;
    SHOW_VECTOR(v0)
    SHOW_VECTOR(v1)

    return 0;
}