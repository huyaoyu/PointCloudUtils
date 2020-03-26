//
// Created by yaoyu on 3/25/20.
//

#ifndef POINTCLOUDUTILS_EDGE_HPP
#define POINTCLOUDUTILS_EDGE_HPP

#include <iostream>

namespace pcu 
{
    
template <typename vertexIndexT, typename weightT>
class Edge
{
public:
    // Default constructor.
    Edge() = default;

    // Constructor with vertex indices.
    Edge(const vertexIndexT& v0, const vertexIndexT& v1, const weightT& w) {
        this->v0 = v0;
        this->v1 = v1;
        this->w  = w;
    }

    // Copy constructor.
    Edge(const Edge<vertexIndexT, weightT>& other) {
        this->v0 = other.v0;
        this->v1 = other.v1;
        this->w  = other.w;
    }

    ~Edge() = default;

    // Overload operator =.
    Edge<vertexIndexT, weightT>& operator = (const Edge<vertexIndexT, weightT>& other) {
        if ( this != &other ) {
            this->v0 = other.v0;
            this->v1 = other.v1;
            this->w  = other.w;
        }

        return *this;
    }

    // Overload the stream operator <<.
    friend std::ostream& operator << (std::ostream& out, const Edge<vertexIndexT, weightT>& e) {
        out << "Edge( "
            << e.w << ", "
            << e.v0 << ", "
            << e.v1 << " )";

        return out;
    }

    // Overload the operator < and operator >.
    bool operator < ( const Edge& other ) const {
        return ( this->w < other.w );
    }

    bool operator > ( const Edge& other ) const {
        return ( this->w > other.w );
    }

public:
    vertexIndexT v0;
    vertexIndexT v1;

    weightT w;
};

} // namespace pcu

#endif //POINTCLOUDUTILS_EDGE_HPP
