//
// Created by pidan1231239 on 18-7-14.
//

#ifndef SLAM_LEARN_LOCALMAP_H
#define SLAM_LEARN_LOCALMAP_H

#include "Map.h"
#include "Solver2D2D.h"

namespace sky {

    using namespace cv;

    class LocalMap {
    protected:
        Solver2D2D solver2D2D;
    public:
        Map::Ptr map;
        typedef shared_ptr<LocalMap> Ptr;

        LocalMap(const cv::Ptr<DescriptorMatcher> &matcher) :
                solver2D2D(matcher) {}

        void addFrame(KeyFrame::Ptr frame) {
            //匹配上一个关键帧并通过2D2D选取内点，后三角化
            solver2D2D.solve(map->keyFrames.back(), frame, false);
            auto lastFrameMap = solver2D2D.triangulate();
            //添加关键帧和地图点
            map->addFrame(frame);
            for (auto &point:lastFrameMap->mapPoints) {
                map->addMapPoint(point);
            }
            //TODO:筛选地图点
        }
    };

}


#endif //SLAM_LEARN_LOCALMAP_H
