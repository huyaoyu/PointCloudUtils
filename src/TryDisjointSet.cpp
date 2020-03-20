//
// Created by yaoyu on 3/19/20.
//

#include <map>
#include <iostream>
#include <vector>

#include <boost/pending/disjoint_sets.hpp>

template <typename vertexIndexT, typename weightT>
class Edge
{
public:
    // Default constructor.
    Edge() = default;

    // Constructor with vertex indices.
    Edge(const vertexIndexT& v0, const vertexIndexT& v1, const weightT& w) {
        mV0 = v0;
        mV1 = v1;
        mW  = w;
    }

    // Copy constructor.
    Edge(const Edge<vertexIndexT, weightT>& other) {
        this->mV0 = other.mV0;
        this->mV1 = other.mV1;
        this->mW  = other.mW;
    }

    ~Edge() = default;

    // Overload operator =.
    Edge<vertexIndexT, weightT>& operator = (const Edge<vertexIndexT, weightT>& other) {
        if ( this != &other ) {
            this->mV0 = other.mV0;
            this->mV1 = other.mV1;
            this->mW  = other.mW;
        }

        return *this;
    }

    // Overload the stream operator <<.
    friend std::ostream& operator << (std::ostream& out, const Edge<vertexIndexT, weightT>& e) {
        out << "Edge( "
            << e.mW << ", "
            << e.mV0 << ", "
            << e.mV1 << " )";

        return out;
    }

public:
    vertexIndexT mV0;
    vertexIndexT mV1;

    weightT mW;
};

/**
 * This function makes a graph that is described at
 * https://www.geeksforgeeks.org/kruskals-minimum-spanning-tree-algorithm-greedy-algo-2/
 *
 * @tparam vT The type of the vertex index.
 * @tparam wT The type of the edge weight.
 * @param v The vertices.
 * @param e The edges.
 */
template < typename vT, typename wT >
static void make_default_graph( std::vector<vT>& v, std::vector<Edge<vT, wT>>& e ) {
    // Create the vertices.
    for (int i = 0; i < 9; i++) {
        v.push_back(static_cast<vT>(i));
    }

    // Create the edges.
    e.push_back( Edge<vT, wT>( static_cast<vT>(7), static_cast<vT>(6), static_cast<wT>(1.0) ) );
    e.push_back( Edge<vT, wT>( static_cast<vT>(8), static_cast<vT>(2), static_cast<wT>(2.0) ) );
    e.push_back( Edge<vT, wT>( static_cast<vT>(6), static_cast<vT>(5), static_cast<wT>(2.0) ) );
    e.push_back( Edge<vT, wT>( static_cast<vT>(0), static_cast<vT>(1), static_cast<wT>(4.0) ) );
    e.push_back( Edge<vT, wT>( static_cast<vT>(2), static_cast<vT>(5), static_cast<wT>(4.0) ) );
    e.push_back( Edge<vT, wT>( static_cast<vT>(8), static_cast<vT>(6), static_cast<wT>(6.0) ) );
    e.push_back( Edge<vT, wT>( static_cast<vT>(2), static_cast<vT>(3), static_cast<wT>(7.0) ) );
    e.push_back( Edge<vT, wT>( static_cast<vT>(7), static_cast<vT>(8), static_cast<wT>(7.0) ) );
    e.push_back( Edge<vT, wT>( static_cast<vT>(0), static_cast<vT>(7), static_cast<wT>(8.0) ) );
    e.push_back( Edge<vT, wT>( static_cast<vT>(1), static_cast<vT>(2), static_cast<wT>(8.0) ) );
    e.push_back( Edge<vT, wT>( static_cast<vT>(3), static_cast<vT>(4), static_cast<wT>(9.0) ) );
    e.push_back( Edge<vT, wT>( static_cast<vT>(5), static_cast<vT>(4), static_cast<wT>(10.0) ) );
    e.push_back( Edge<vT, wT>( static_cast<vT>(1), static_cast<vT>(7), static_cast<wT>(11.0) ) );
    e.push_back( Edge<vT, wT>( static_cast<vT>(3), static_cast<vT>(5), static_cast<wT>(14.0) ) );
}

template <typename vT, typename wT>
static void display_graph(const std::vector<vT>& vertices, const std::vector<Edge<vT, wT>>& edges) {
    // List the vertices.
    std::cout << "Vertices: " << std::endl;
    for ( const vT& v : vertices ) {
        std::cout << v << ", ";
    }
    std::cout << std::endl << std::endl;

    // List the edges.
    std::cout << "Edges: " << std::endl;
    for ( const Edge<vT, wT>& e : edges ) {
        std::cout << e << std::endl;
    }
}

int main(int argc, char* argv[])
{
    std::cout << "Hello, TryDisjointSet! " << std::endl;

    // The original vertices and edges.
    std::vector<int> vertices;
    typedef Edge<int, double> Edge_t;
    std::vector<Edge_t> edgesOri;

    // The default
    make_default_graph(vertices, edgesOri);
    std::cout << "========== The initial graph. ========== " << std::endl;
    display_graph(vertices, edgesOri);

    // The maps.
    typedef std::map<int, std::size_t> Rank_t;
    typedef std::map<int, int> Parent_t;
    typedef boost::associative_property_map<Rank_t> PropMapRank_t;
    typedef boost::associative_property_map<Parent_t> PropMapParent_t;

    Rank_t          mapRank;
    Parent_t        mapParent;
    PropMapRank_t   propMapRank(mapRank);
    PropMapParent_t propMapParent(mapParent);

    // The disjoint set.
    boost::disjoint_sets<PropMapRank_t, PropMapParent_t> djs( propMapRank, propMapParent );

    // Make a disjoint set with all the vertices as sub-set containing single element.
    for ( const int& v : vertices ) {
        djs.make_set( v );
    }

    std::cout << "djs.count_sets() = " << djs.count_sets( vertices.begin(), vertices.end() ) << std::endl;
    std::cout << std::endl;

    // Make the MST.
    std::cout << "========== Start making the MST. ==========" << std::endl;
    size_t nDJSEdges = 0;
    std::vector<Edge_t> edgesMST; // Stores the edges of the MST.

    for ( const Edge_t& e : edgesOri ) {
        if ( nDJSEdges == vertices.size() - 1 ) {
            std::cout << "Number of edges in the MST reaches the maximum." << std::endl;
            break;
        }

        std::cout << "Process " << e << ": " << std::endl;

        // Get the sub-sets contain the two vertices of the current edge.
        auto u = djs.find_set(e.mV0);
        auto v = djs.find_set(e.mV1);

        // Check if it makes a circle in the MST.
        if ( u != v) {
            // Not a circle.
            djs.link(u, v);
            nDJSEdges++;
            edgesMST.push_back(e);
            std::cout << "Add new sub set." << std::endl;
        } else {
            // A circle will be made if we contain this edge into the MST.
            std::cout << "The two vertices of the edge are in the same sub-set " << u << ". " << std::endl;
        }
    }

    std::cout << "djs.count_sets() = " << djs.count_sets( vertices.begin(), vertices.end() ) << std::endl;
    std::cout << std::endl;

    std::cout << "========== The MST. ==========" << std::endl;

    display_graph(vertices, edgesMST);

    return 0;
}