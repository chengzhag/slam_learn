//
// Created by pidan1231239 on 18-7-14.
//

#ifndef SLAM_LEARN_LOCALMAP_H
#define SLAM_LEARN_LOCALMAP_H

#include "Map.h"
#include "Matcher.h"
#include "Triangulater.h"
#include <boost/thread.hpp>
#include "MapViewer.h"

namespace sky {

    class LocalMap {

    private:
        Matcher matcher;
        boost::thread thread;
        KeyFrame::Ptr refFrame, currFrame;

        MapViewer mapViewer;

    public:
        typedef shared_ptr<LocalMap> Ptr;
        Map::Ptr map;
        boost::mutex mapMutex;

        LocalMap() {}

        void init(const Map::Ptr &map);

        void addFrame(const KeyFrame::Ptr &frame);

        inline KeyFrame::Ptr getLastFrame() {
            boost::mutex::scoped_lock lock(mapMutex);
            return map->getLastFrame();
        }

    private:
        void threadFunc();

        void prepareKeyFrame();

        void filtMapPoints();

        void triangulate();

        void ba();

        void filtKeyFrames();
    };


}


#endif //SLAM_LEARN_LOCALMAP_H
