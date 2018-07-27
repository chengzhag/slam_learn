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

    private:
        Matcher matcher;

    public:
        typedef shared_ptr<LocalMap> Ptr;
        Map::Ptr map;

        LocalMap() {}

        void addFrame(const KeyFrame::Ptr &frame);

        KeyFrame::Ptr getLastFrame();

    };


}


#endif //SLAM_LEARN_LOCALMAP_H
