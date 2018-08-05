//
// Created by pidan1231239 on 18-7-14.
//

#include "LocalMap.h"
#include "BA.h"
#include "Matcher.h"
#include "Triangulater.h"
#include <opencv2/cvv.hpp>
#include "basic.h"

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
        waitForThread();

        boost::mutex::scoped_lock lock(mapMutex);
        refFrame = getLastFrame();
        lock.unlock();

#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "LocalMap: Adding keyFrame... "
             << disBetween(frame, refFrame) << " from last keyFrame" << endl;
#endif

        //开始线程
        thread = shared_ptr<boost::thread>(
                new boost::thread(boost::bind(&LocalMap::threadFunc, this))
        );

    }

    void LocalMap::viewMatchInCVV() const {
        cvv::debugDMatch(refFrame->image, refFrame->keyPoints,
                         currFrame->image, currFrame->keyPoints,
                         matcher.matches,
                         CVVISUAL_LOCATION,
                         "match used in triangulation");
    }

    void LocalMap::threadFunc() {
        prepareKeyFrame();
        triangulate();
        ba();
        //BA可能导致一些外点，不如把筛选过程放到BA后
        filtKeyFrames();
        filtMapPoints();

        boost::mutex::scoped_lock lock(mapMutex);
        mapViewer.update(map);
        lock.unlock();

#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "LocalMap: End adding! " << endl;
#endif
    }

    void LocalMap::prepareKeyFrame() {
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "LocalMap: prepareKeyFrame... " << endl;
#endif
        //匹配上一个关键帧
        matcher.match(refFrame->descriptors, currFrame->descriptors);
    }

    void LocalMap::triangulate() {
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "LocalMap: triangulate... " << endl;
#endif
        //三角化
        auto triangulateMap = triangulater.triangulate(
                refFrame, currFrame, matcher.matches);

        BA ba({BA::Mode_Fix_Intrinsic, BA::Mode_Fix_First_Frame});
        ba.loadMap(triangulateMap);
        ba.ba();
        ba.writeMap();

        //添加关键帧和地图点
        newMapPoints.clear();
        boost::mutex::scoped_lock lock(mapMutex);
        map->addFrame(currFrame);
        for (auto &point:triangulateMap->mapPoints) {
            map->addMapPoint(point);
            newMapPoints.insert(point);
        }
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "LocalMap: "
             << triangulateMap->mapPoints.size() << " points added to localMap. " << endl;
#endif
        lock.unlock();
    }

    void LocalMap::ba() {
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "LocalMap: ba... " << endl;
#endif
        BA ba({BA::Mode_Fix_Intrinsic, BA::Mode_Fix_First_2Frames});

        boost::mutex::scoped_lock lock(mapMutex);
        ba.loadMap(map);
        lock.unlock();

        ba.ba();

        lock.lock();
        ba.writeMap();
        lock.unlock();
    }

    void LocalMap::filtKeyFrames() {
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "LocalMap: filtKeyFrames... " << endl;
#endif
        int iFrames = 0;
        boost::mutex::scoped_lock lock(mapMutex);
        for (auto it = map->keyFrames.rbegin(); it != map->keyFrames.rend();) {
            if (isGoodFrame(*it) && iFrames < maxKeyFrames) {
                ++it;
                ++iFrames;
            } else {
                it = list<KeyFrame::Ptr>::reverse_iterator(map->keyFrames.erase((++it).base()));
            }
        }
        lock.unlock();
    }

    bool LocalMap::isGoodFrame(const KeyFrame::Ptr &keyFrame) const {

        return true;
    }

    void LocalMap::filtMapPoints() {
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "LocalMap: filtMapPoints... " << endl;
#endif
        boost::mutex::scoped_lock lock(mapMutex);
        for (auto it = map->mapPoints.begin(); it != map->mapPoints.end();) {
            if (isGoodPoint(*it))
                ++it;
            else
                it = map->mapPoints.erase(it);
        }
        lock.unlock();
    }

    bool LocalMap::isGoodPoint(const MapPoint::Ptr &mapPoint) const {
/*#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: "   << "\t" << !setHas(newMapPoints, mapPoint) << "\t"
             << mapPoint->observedFrames.size() << " observedFrames" << endl;
#endif*/
        if (map->keyFrames.size() >= 4)
            if (!setHas(newMapPoints, mapPoint)
                && mapPoint->observedFrames.size() < 3)
                return false;

        for (auto &observedFrame:mapPoint->observedFrames) {
            //根据到每个观测帧的最大距离来判断
            if (disBetween(observedFrame.first, mapPoint) > maxInlierPointDis)
                return false;

            //根据重投影误差删除外点
            cv::Point2d reprojCoor;
            if (!proj2frame(mapPoint, observedFrame.first, reprojCoor))
                return false;
            auto reprojErr = disBetween(observedFrame.second, reprojCoor);
            if (reprojErr > triangulater.maxReprojErr)
                return false;
        }

        //如果不被当前LocalMap中的关键帧观测，则过滤
        for (auto &keyFrame:map->keyFrames) {
            if (mapPoint->hasObservedFrame(keyFrame))
                break;
            if (keyFrame == map->keyFrames.back())
                return false;
        }

        return true;
    }

}