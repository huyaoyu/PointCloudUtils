//
// Created by yaoyu on 5/25/20.
//

#ifndef POINTCLOUDUTILS_INCLUDES_OCCUPANCYMAP_FRONTIERMAP_HPP
#define POINTCLOUDUTILS_INCLUDES_OCCUPANCYMAP_FRONTIERMAP_HPP

#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

namespace f_map
{

typedef bool Frontier_t;
static const Frontier_t FRONTIER     = true;
static const Frontier_t NON_FRONTIER = false;

using namespace octomap;

class FrontierOcTreeNode : public OcTreeNode {
public:
    FrontierOcTreeNode()
        : OcTreeNode(),
          fv(NON_FRONTIER)
    {}

    FrontierOcTreeNode(Frontier_t f)
    : OcTreeNode(),
      fv(f)
    {}

    FrontierOcTreeNode( const FrontierOcTreeNode &other )
    : OcTreeNode(other),
      fv(other.fv)
      {}

    bool operator == ( const FrontierOcTreeNode& other ) const {
        return ( other.value == value && other.fv == fv );
    }

    void copyData( const FrontierOcTreeNode &other ) {
        OcTreeNode::copyData(other);
        this->fv = other.is_frontier() ? FRONTIER : NON_FRONTIER;
    }

    bool is_frontier() const { return fv; }
    void set_frontier() { fv = FRONTIER; }
    void clear_frontier() { fv = NON_FRONTIER; }

    std::istream& readData( std::istream &ins ) {
        ins.read((char*) &value, sizeof(value)); // Occupancy.
        ins.read((char*) &fv, sizeof(Frontier_t));  // Frontier.

        return ins;
    }

    std::ostream& writeData( std::ostream &out ) const {
        out.write((const char*) &value, sizeof(value)); // Occupancy.
        out.write((const char*) &fv, sizeof(Frontier_t));  // Frontier.

        return out;
    }

public:
    Frontier_t fv;
};

class FrontierOcTree : public OccupancyOcTreeBase<FrontierOcTreeNode> {
public:
    FrontierOcTree( double in_resolution );
    FrontierOcTree* create() const { return new FrontierOcTree(resolution); }

    std::string getTreeType() const { return "FrontierOcTree"; }
//    virtual bool pruneNode( FrontierOcTreeNode *node);
//    virtual bool isNodeCollapsible( const FrontierOcTreeNode *node ) const;

    FrontierOcTreeNode* set_node_frontier( const OcTreeKey &key ) {
        FrontierOcTreeNode *node = search(key);
        if ( node ) {
            node->set_frontier();
        }
        return node;
    }

    FrontierOcTreeNode* set_node_frontier( float x, float y, float z ) {
        OcTreeKey key;
        if( !this->coordToKeyChecked( point3d(x,y,z), key ) ) {
            return nullptr;
        }

        return set_node_frontier(key);
    }

    FrontierOcTreeNode* clear_node_frontier( const OcTreeKey &key ) {
        FrontierOcTreeNode *node = search(key);
        if ( node ) {
            node->clear_frontier();
        }
        return node;
    }

    FrontierOcTreeNode* clear_node_frontier( float x, float y, float z ) {
        OcTreeKey key;
        if( !this->coordToKeyChecked( point3d(x,y,z), key ) ) {
            return nullptr;
        }

        return clear_node_frontier(key);
    }

protected:
    class StaticMemberInitializer{
    public:
        StaticMemberInitializer() {
            FrontierOcTree *tree = new FrontierOcTree(0.1);
            tree->clearKeyRays();
            AbstractOcTree::registerTreeType(tree);
        }

        void ensureLinking() {}
    };

    static StaticMemberInitializer frontierOcTreeMemberInit;
};

} // namespace f_map

#endif //POINTCLOUDUTILS_INCLUDES_OCCUPANCYMAP_FRONTIERMAP_HPP
