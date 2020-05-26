//
// Created by yaoyu on 5/25/20.
//

#include "OccupancyMap/FrontierMap.hpp"

using namespace f_map;

FrontierOcTree::FrontierOcTree( double in_resolution )
        : OccupancyOcTreeBase<FrontierOcTreeNode>(in_resolution) {
    frontierOcTreeMemberInit.ensureLinking();
}

FrontierOcTree::StaticMemberInitializer FrontierOcTree::frontierOcTreeMemberInit;
