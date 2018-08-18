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
        float maxInlierPointDis, maxReprojErr;
        int maxKeyFrames,minKeyFrames, minMapPoints;
        Matcher matcher;
        shared_ptr<boost::thread> thread;
        KeyFrame::Ptr refFrame, currFrame;
        Triangulater triangulater;

        MapViewer mapViewer;

    public:
        typedef shared_ptr<LocalMap> Ptr;
        Map::Ptr map;
        boost::mutex mapMutex;

        LocalMap(int minMapPoints = Config::get<float>("LocalMap.minMapPoints"),
                 float maxInlierPointDis = Config::get<float>("LocalMap.maxInlierPointDis"),
                 int minKeyFrames = Config::get<float>("LocalMap.minKeyFrames"),
                 float maxReprojErr = Config::get<float>("LocalMap.maxReprojErr")
        ) :
                minMapPoints(minMapPoints),
                maxInlierPointDis(maxInlierPointDis),
                minKeyFrames(minKeyFrames),
                maxKeyFrames(minKeyFrames),
                maxReprojErr(maxReprojErr),
                matcher(
                        Config::get<float>("LocalMap.Matcher.rankRatio"),
                        Config::get<float>("LocalMap.Matcher.disThresRatio"),
                        Config::get<float>("LocalMap.Matcher.disThresMin"),
                        Config::get<float>("LocalMap.Matcher.testRatio")
                ) {
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "LocalMap: Initializing..." << endl;
            printVariable(minMapPoints);
            printVariable(maxInlierPointDis);
            printVariable(minKeyFrames);
            printVariable(maxReprojErr);

            printVariable(matcher.rankRatio);
            printVariable(matcher.disThresRatio);
            printVariable(matcher.disThresMin);
            printVariable(matcher.testRatio);
        }

        ~LocalMap() {
            waitForThread();
        }

        void init(const Map::Ptr &map);

        void addFrame(const KeyFrame::Ptr &frame);

        inline bool isAdding() const {
            if (thread) {
                thread->timed_join(boost::posix_time::microseconds(0));
                return thread->joinable();
            }
            return false;
        }

        inline void waitForThread() {
            if (thread) {
                thread->join();
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

        int isGoodFrame(const KeyFrame::Ptr &keyFrame) const;

        void filtMapPoints(Map::Ptr &map);

        int isGoodPoint(const MapPoint::Ptr &mapPoint) const;
    };


}


#endif //SLAM_LEARN_LOCALMAP_H
