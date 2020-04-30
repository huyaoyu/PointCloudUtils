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
    virtual ~ProximityGraph() = default;

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

    std::size_t size() const {
        return neighborsTable.size();
    }

    void set_point_cloud(typename pcl::PointCloud<pcT>::Ptr& p) {
        ppc = p;

        // Clear.
        if ( processed ) {
            this->clear();
        }

        // Allocate.
        this->allocate();
    }

    Neighbors_t& get_neighbors(int index) {
        return neighborsTable[index];
    }

    const Neighbors_t& get_neighbors(int index) const {
        return neighborsTable[index];
    }

    virtual void process(int showDetailBase) {
        throw(std::runtime_error("virtual base function of ProximityGraph::process() is called."));
    }

    ProximityGraph<pcT>& operator = ( const ProximityGraph<pcT>& other ) {
        if ( this == &other ) {
            return *this;
        }

        this->clear();
        this->ppc = other.ppc;
        this->allocate();

        for ( std::size_t i = 0; i < other.neighborsTable.size(); ++i ) {
            this->neighborsTable[i] = other.neighborsTable[i];
        }

        return *this;
    }

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

template < typename pT >
class KRProximityGraph : public ProximityGraph<pT> {
public:
    KRProximityGraph() : k(10), radius(0.05), ProximityGraph<pT>() { }
    virtual ~KRProximityGraph() = default;

    void set_k_r(int valK, double valR) {
        assert( valK > 0 );
        assert( valR > 0 );

        this->k = valK;
        this->radius = valR;
    }

    void process(int showDetailBase=100000);

protected:
    int k;
    double radius;
};

template <typename pcT>
void KRProximityGraph<pcT>::process(int showDetailBase) {
    // Check if already processed.
    if ( this->processed ) {
        this->clear();
        this->allocate();
    }

    // Create a KD-Tree.
    typename pcl::KdTreeFLANN<pcT>::Ptr tree ( new pcl::KdTreeFLANN<pcT> );
    tree->setInputCloud(this->ppc);

    // Loop over all the points of the input point cloud.
    std::vector<int> indexKNN;
    std::vector<float> squaredDistance;
    std::vector<int> indexR;
    std::vector<float> squaredDistanceR;

    QUICK_TIME_START(te)

    std::cout << "Start processing " << this->ppc->size() << " points." << std::endl;

    for ( std::size_t i = 0; i < this->ppc->size(); ++i) {
        indexKNN.clear();
        squaredDistance.clear();
        indexR.clear();
        squaredDistanceR.clear();

        // Search the k-nearest neighbors.
        tree->nearestKSearch( *(this->ppc), i, k+1, indexKNN, squaredDistance );

        // Update the neighbor set of the current vertex.
        this->add_neighbors_2_single_table_entry( indexKNN, this->neighborsTable[i], i );

        // Search the neighbors inside the radius.
        tree->radiusSearch( *(this->ppc), i, radius, indexR, squaredDistanceR );

        // Update the neighbor set of the current vertex.
        this->add_neighbors_2_single_table_entry( indexR, this->neighborsTable[i], i );

        // Update the neighbor sets of all the k-nearest neighbors.
        this->symmetric_update(i);

        // Show details.
        if ( showDetailBase > 0 && i % showDetailBase == 0 ) {
            std::cout << "Proximity graph " << static_cast<double>(i) / this->ppc->size() * 100 << "%..." << std::endl;
        }
    }

    QUICK_TIME_END(te)

    std::cout << "Build proximity graph in " << te << "ms." << std::endl;
}

template < typename pT >
class RProximityGraph : public ProximityGraph<pT> {
    using ProximityGraph<pT>::ppc;
    using ProximityGraph<pT>::neighborsTable;
    using ProximityGraph<pT>::processed;
public:
    RProximityGraph() : radius(0.05), ProximityGraph<pT>() { }
    virtual ~RProximityGraph() = default;

    void set_r(double valR) {
        assert( valR > 0 );

        this->radius = valR;
    }

    void process(int showDetailBase=100000);

protected:
    double radius;
};

template <typename pcT>
void RProximityGraph<pcT>::process(int showDetailBase) {
    // Check if already processed.
    if ( processed ) {
        this->clear();
        this->allocate();
    }

    // Create a KD-Tree.
    typename pcl::KdTreeFLANN<pcT>::Ptr tree ( new pcl::KdTreeFLANN<pcT> );
    tree->setInputCloud(ppc);

    // Loop over all the points of the input point cloud.
    std::vector<int> indexR;
    std::vector<float> squaredDistanceR;

    QUICK_TIME_START(te)

    std::cout << "Start processing " << ppc->size() << " points." << std::endl;

    for ( std::size_t i = 0; i < ppc->size(); ++i) {
        indexR.clear();
        squaredDistanceR.clear();

        // Search the neighbors inside the radius.
        tree->radiusSearch( *(ppc), i, radius, indexR, squaredDistanceR );

        // Update the neighbor set of the current vertex.
        this->add_neighbors_2_single_table_entry( indexR, neighborsTable[i], i );

        // Update the neighbor sets of all the k-nearest neighbors.
        this->symmetric_update(i);

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
