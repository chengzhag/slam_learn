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
        float minKeyFrameDis, maxKeyFrameDis,maxKeyFrameTrackRatio;
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
                float maxKeyFrameTrackRatio = Config::get<float>("Tracker.maxKeyFrameTrackRatio"),
                int minKeyFrameInlierNum = Config::get<int>("Tracker.minKeyFrameInlierNum"),
                int minKeyFrameInterval = Config::get<int>("Tracker.minKeyFrameInterval")
        ) :
                localMap(localMap),
                solver3D2D(localMap),
                minKeyFrameDis(minKeyFrameDis),
                maxKeyFrameDis(maxKeyFrameDis),
                maxKeyFrameTrackRatio(maxKeyFrameTrackRatio),
                minKeyFrameInlierNum(minKeyFrameInlierNum),
                minKeyFrameInterval(minKeyFrameInterval) {
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "   << "Tracker: Initializing..." << endl;
            printVariable(minKeyFrameDis);
            printVariable(maxKeyFrameDis);
            printVariable(maxKeyFrameTrackRatio);
            printVariable(minKeyFrameInlierNum);
            printVariable(minKeyFrameInterval);
        }

        bool step(const KeyFrame::Ptr &frame);

    private:
        bool isKeyFrame() const;


    };

}

#endif //SLAM_LEARN_TRACKER_H
