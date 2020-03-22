//
// Created by yaoyu on 3/22/20.
//

#ifndef POINTCLOUDUTILS_LISTPOINTS_HPP
#define POINTCLOUDUTILS_LISTPOINTS_HPP

#include <iostream>
#include <sstream>
#include <string>

#include <pcl/point_types.h>

using namespace std;

namespace pcu
{

template < typename pT >
void list_points( const typename pcl::PointCloud<pT>::Ptr& pc,
        const string& firstLine="") {

    if ( 0 != firstLine.length() ) {
        cout << firstLine << endl;
    }

    for ( int i = 0; i < pc->size(); ++i ) {
        cout << pc->at(i) << endl;
    }
    cout << endl;
}

}

#endif //POINTCLOUDUTILS_LISTPOINTS_HPP
