//
// Created by pidan1231239 on 18-7-14.
//

#include "LocalMap.h"
#include "BA.h"
#include "Matcher.h"
#include "Triangulater.h"
#include <opencv2/cvv.hpp>

namespace sky {

    void LocalMap::addFrame(const KeyFrame::Ptr &frame) {
        //匹配上一个关键帧，后三角化
        auto lastFrame = map->getLastFrame();
        matcher.match(lastFrame->descriptors, frame->descriptors);
        cvv::debugDMatch(lastFrame->image, lastFrame->keyPoints, frame->image, frame->keyPoints, matcher.matches,
                         CVVISUAL_LOCATION,
                         "match used in triangulation");
        Triangulater triangulater;
        auto lastFrameMap = triangulater.triangulate(
                lastFrame, frame, matcher.matches);

        //BA ba;
        //ba(lastFrameMap, {BA::Mode_Fix_Points, BA::Mode_Fix_Intrinsic, BA::Mode_Fix_First_Frame});
#ifdef DEBUG
        cout << "LocalMap: Adding keyFrame... " << frame->getDis2(lastFrame)
             << " from last keyFrame" << endl;
#endif
        //添加关键帧和地图点
        map->addFrame(frame);
        for (auto &point:lastFrameMap->mapPoints) {
            map->addMapPoint(point);
        }
        //TODO:筛选地图点
        //BA ba;
        //ba(map, {BA::Mode_Fix_Intrinsic, BA::Mode_Fix_First_Frame});
    }

    KeyFrame::Ptr LocalMap::getLastFrame() {
        return map->getLastFrame();
    }

}