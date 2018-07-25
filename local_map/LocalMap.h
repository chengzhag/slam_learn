//
// Created by pidan1231239 on 18-7-14.
//

#ifndef SLAM_LEARN_LOCALMAP_H
#define SLAM_LEARN_LOCALMAP_H

#include "Map.h"
#include "Matcher.h"
#include "Triangulater.h"

namespace sky {

    class LocalMap {

    protected:
        Matcher matcher;

    public:
        Map::Ptr map;
        typedef shared_ptr<LocalMap> Ptr;

        LocalMap() {}

        void addFrame(KeyFrame::Ptr frame);

        KeyFrame::Ptr getLastFrame();

    };


}


#endif //SLAM_LEARN_LOCALMAP_H
