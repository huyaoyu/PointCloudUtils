//
// Created by yaoyu on 3/21/20.
//

#ifndef POINTCLOUDUTILS_PROXIMITYGRAPH_HPP
#define POINTCLOUDUTILS_PROXIMITYGRAPH_HPP

#include <iostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "Profiling/SimpleTime.hpp"

namespace pcu
{

template <typename pcT>
class ProximityGraph {
public:
//    typedef std::size_t Index_t;
    typedef int Index_t;
    typedef std::set<Index_t> Neighbors_t;

public:
    ProximityGraph(): processed(false) {}
    ~ProximityGraph() = default;

    void clear() {
        neighborsTable.clear();

        processed = false;
    }

    void allocate() {
        if ( processed ) {
            std::stringstream ss;
            ss << "Cannot allocate memory after the graph have been processed.";
            throw( std::runtime_error(ss.str()) );
        }

        // Update the memory of the vertices and neighbors table.
        neighborsTable.resize(ppc->size());
    }

    void set_point_cloud(typename pcl::PointCloud<pcT>::Ptr& p) {
        ppc = p;

        // Clear.
        if ( processed ) {
            clear();
        }

        // Allocate.
        allocate();
    }

    Neighbors_t& get_neighbors(int index) {
        return neighborsTable[index];
    }

    void process(int k, double radius, int showDetailBase=100000);

protected:
    void add_neighbors_2_single_table_entry( const std::vector<Index_t>& neighbors,
            Neighbors_t& entry, int selfIndex ) {
        for ( auto idx : neighbors ) {
            if ( idx != selfIndex ) {
                entry.insert(idx);
            }
        }
    }

    void symmetric_update( const Index_t from ) {
        for ( auto ni : neighborsTable[from] ) {
            neighborsTable[ni].insert(from);
        }
    }

protected:
    typename pcl::PointCloud<pcT>::Ptr ppc;
    std::vector< Neighbors_t > neighborsTable;
    bool processed;
};

template <typename pcT>
void ProximityGraph<pcT>::process(int k, double radius, int showDetailBase) {
    // Check if already processed.
    if ( processed ) {
        clear();
        allocate();
    }

    // Create a KD-Tree.
    typename pcl::KdTreeFLANN<pcT>::Ptr tree ( new pcl::KdTreeFLANN<pcT> );
    tree->setInputCloud(ppc);

    // Loop over all the points of the input point cloud.
    std::vector<int> indexKNN(k);
    std::vector<float> squaredDistance(k);
    std::vector<int> indexR;
    std::vector<float> squaredDistanceR;

    QUICK_TIME_START(te)

    std::cout << "Start processing " << ppc->size() << " points." << std::endl;

    for ( std::size_t i = 0; i < ppc->size(); ++i) {
        indexKNN.clear();
        squaredDistance.clear();
        indexR.clear();
        squaredDistanceR.clear();

        // Search the k-nearest neighbors.
        tree->nearestKSearch( *ppc, i, k+1, indexKNN, squaredDistance );

        // Update the neighbor set of the current vertex.
        add_neighbors_2_single_table_entry( indexKNN, neighborsTable[i], i );

        // Search the neighbors inside the radius.
        tree->radiusSearch( *ppc, i, radius, indexR, squaredDistanceR );

        // Update the neighbor set of the current vertex.
        add_neighbors_2_single_table_entry( indexR, neighborsTable[i], i );

        // Update the neighbor sets of all the k-nearest neighbors.
        symmetric_update(i);

        // Show details.
        if ( showDetailBase > 0 && i % showDetailBase == 0 ) {
            std::cout << "Proximity graph " << static_cast<double>(i) / ppc->size() * 100 << "%..." << std::endl;
        }
    }

    QUICK_TIME_END(te)

    std::cout << "Build proximity graph in " << te << "ms." << std::endl;
}

} // Namespace pcu.

#endif //POINTCLOUDUTILS_PROXIMITYGRAPH_HPP
