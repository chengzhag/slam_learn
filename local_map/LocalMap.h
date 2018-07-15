//
// Created by pidan1231239 on 18-7-14.
//

#ifndef SLAM_LEARN_LOCALMAP_H
#define SLAM_LEARN_LOCALMAP_H

#include "Map.h"

namespace sky {

    using namespace cv;

    class LocalMap {
    protected:

    public:
        Map::Ptr map;
        typedef shared_ptr<LocalMap> Ptr;

        LocalMap() {}


    };

}


#endif //SLAM_LEARN_LOCALMAP_H
