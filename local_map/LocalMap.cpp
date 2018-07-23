//
// Created by pidan1231239 on 18-7-14.
//

#include "LocalMap.h"
#include "BA.h"

namespace sky {

    void LocalMap::addFrame(KeyFrame::Ptr frame) {
        //匹配上一个关键帧并通过2D2D选取内点，后三角化
        solver2D2D.solve(map->keyFrames.back(), frame, false);
        auto lastFrameMap = solver2D2D.triangulate();

        BA ba;
        ba(lastFrameMap, {BA::Mode_Fix_Points, BA::Mode_Fix_Intrinsic, BA::Mode_Fix_First_Frame});
        cout << "LocalMap: " << frame->dis2Coor(map->keyFrames.back()->Tcw.translation())
             << " from last keyFrame" << endl;

        //添加关键帧和地图点
        map->addFrame(frame);
        for (auto &point:lastFrameMap->mapPoints) {
            map->addMapPoint(point);
        }
        //TODO:筛选地图点
    }

}