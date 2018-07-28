//
// Created by pidan1231239 on 18-7-14.
//

#ifndef SLAM_LEARN_TRACKER_H
#define SLAM_LEARN_TRACKER_H

#include "common_include.h"
#include "LocalMap.h"
#include "Solver3D2D.h"
#include "Config.h"

namespace sky {

    class Tracker {

    private:
        float minKeyFrameDis, maxKeyFrameDis;
        int minKeyFrameInlierNum, minKeyFrameInterval, frameInterval = 0;
        LocalMap::Ptr localMap;
        Solver3D2D solver3D2D;
        KeyFrame::Ptr frame;

    public:
        typedef shared_ptr<Tracker> Ptr;

        Tracker(
                LocalMap::Ptr localMap,
                float minKeyFrameDis = Config::get<float>("Tracker.minKeyFrameDis"),
                float maxKeyFrameDis = Config::get<float>("Tracker.maxKeyFrameDis"),
                int minKeyFrameInlierNum = Config::get<int>("Tracker.minKeyFrameInlierNum"),
                int minKeyFrameInterval = Config::get<int>("Tracker.minKeyFrameInterval")
        ) :
                localMap(localMap),
                minKeyFrameDis(minKeyFrameDis),
                maxKeyFrameDis(maxKeyFrameDis),
                minKeyFrameInlierNum(minKeyFrameInlierNum),
                minKeyFrameInterval(minKeyFrameInterval) {}

        void step(const KeyFrame::Ptr &frame);

    private:
        bool isKeyFrame() const;


    };

}

#endif //SLAM_LEARN_TRACKER_H
