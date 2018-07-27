//
// Created by pidan1231239 on 18-7-14.
//

#include "LocalMap.h"
#include "BA.h"
#include "Matcher.h"
#include "Triangulater.h"
#include <opencv2/cvv.hpp>

namespace sky {
    void LocalMap::init(const Map::Ptr &map) {
        boost::mutex::scoped_lock lock(mapMutex);
        this->map = map;
        mapViewer.update(this->map);
        lock.unlock();
    }

    void LocalMap::addFrame(const KeyFrame::Ptr &frame) {
        currFrame = frame;
        //在加入keyFrame前等待上一次线程结束
        if (thread.joinable()) {
#ifdef DEBUG
            cout << "LocalMap: Waiting for thread to finish..." << endl;
#endif
            thread.join();
        }

        refFrame = getLastFrame();
#ifdef DEBUG
        cout << "LocalMap: Adding keyFrame... " << frame->getDis2(refFrame)
             << " from last keyFrame" << endl;
#endif

        //开始线程
        thread = boost::thread(boost::bind(&LocalMap::threadFunc, this));

    }


    void LocalMap::threadFunc() {
        prepareKeyFrame();
        triangulate();
        ba();
        //BA可能导致一些外点，不如把筛选过程放到BA后
        filtMapPoints();
        filtKeyFrames();

        boost::mutex::scoped_lock lock(mapMutex);
        mapViewer.update(map);
        lock.unlock();
    }

    void LocalMap::prepareKeyFrame() {
#ifdef DEBUG
        cout << "LocalMap: prepareKeyFrame... " << endl;
#endif
        //匹配上一个关键帧
        matcher.match(refFrame->descriptors, currFrame->descriptors);
        cvv::debugDMatch(refFrame->image, refFrame->keyPoints,
                         currFrame->image, currFrame->keyPoints,
                         matcher.matches,
                         CVVISUAL_LOCATION,
                         "match used in triangulation");
    }

    void LocalMap::filtMapPoints() {
#ifdef DEBUG
        cout << "LocalMap: filtMapPoints... " << endl;
#endif
        boost::mutex::scoped_lock lock(mapMutex);
        for (auto it = map->mapPoints.begin(); it != map->mapPoints.end();) {
            if (isGoodPoint(*it))
                ++it;
            else
                it = map->mapPoints.erase(it);
        }
    }

    bool LocalMap::isGoodPoint(const MapPoint::Ptr &mapPoint) {
/*#ifdef DEBUG
        cout << "\t" << mapPoint->observedFrames.size() << " observedFrames" << endl;
#endif*/
        if (map->keyFrames.size() >= 3)
            if (mapPoint->observedFrames.size() < 3)
                return false;

        //根据到每个观测帧的最大距离来判断
        for (auto &observedFrame:mapPoint->observedFrames) {
            if (observedFrame.first->getDis2(mapPoint) > maxInlierPointDis)
                return false;
        }
        return true;
    }

    void LocalMap::triangulate() {
#ifdef DEBUG
        cout << "LocalMap: triangulate... " << endl;
#endif
        //三角化
        Triangulater triangulater;
        auto triangulateMap = triangulater.triangulate(
                refFrame, currFrame, matcher.matches);

        //BA ba;
        //ba(triangulateMap, {BA::Mode_Fix_Points, BA::Mode_Fix_Intrinsic, BA::Mode_Fix_First_Frame});

        //添加关键帧和地图点
        boost::mutex::scoped_lock lock(mapMutex);
        map->addFrame(currFrame);
        for (auto &point:triangulateMap->mapPoints) {
            map->addMapPoint(point);
        }
        lock.unlock();
    }

    void LocalMap::ba() {
#ifdef DEBUG
        cout << "LocalMap: ba... " << endl;
#endif
        boost::mutex::scoped_lock lock(mapMutex);
        BA ba;
        ba(map, {BA::Mode_Fix_Intrinsic, BA::Mode_Fix_First_2Frames});
        lock.unlock();
    }

    void LocalMap::filtKeyFrames() {
#ifdef DEBUG
        cout << "LocalMap: filtKeyFrames... " << endl;
#endif

    }

}