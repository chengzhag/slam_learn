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
#include "Config.h"
#include <unordered_set>

namespace sky {

    class LocalMap {

    private:
        float maxInlierPointDis;
        int maxKeyFrames;
        Matcher matcher;
        boost::thread thread;
        KeyFrame::Ptr refFrame, currFrame;
        Triangulater triangulater;

        MapViewer mapViewer;

    public:
        typedef shared_ptr<LocalMap> Ptr;
        Map::Ptr map;
        boost::mutex mapMutex;

        LocalMap(float maxInlierPointDis = Config::get<float>("LocalMap.maxInlierPointDis"),
                 int maxKeyFrames = Config::get<float>("LocalMap.maxKeyFrames")
        ) :
                maxInlierPointDis(maxInlierPointDis),
                maxKeyFrames(maxKeyFrames) {}

        void init(const Map::Ptr &map);

        void addFrame(const KeyFrame::Ptr &frame);

        inline bool isAdding() const {
            return thread.joinable();
        }

        inline void waitForThread() {
            if (isAdding()) {
#ifdef DEBUG
                cout << "LocalMap: Waiting for thread to finish..." << endl;
#endif
                thread.join();
            }
        }

        void viewMatchInCVV() const;

        inline void viewReprojInCVV() const {
            triangulater.viewReprojInCVV();
        }

        inline KeyFrame::Ptr getLastFrame() const {
            return map->getLastFrame();
        }

    private:
        void threadFunc();

        void prepareKeyFrame();

        void triangulate();

        void ba();

        unordered_set<MapPoint::Ptr> newMapPoints;

        void filtKeyFrames();

        bool isGoodFrame(const KeyFrame::Ptr &keyFrame) const;

        void filtMapPoints();

        bool isGoodPoint(const MapPoint::Ptr &mapPoint) const;
    };


}


#endif //SLAM_LEARN_LOCALMAP_H
