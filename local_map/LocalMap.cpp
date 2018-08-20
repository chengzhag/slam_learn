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
        this->localMap = map;
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
        //mapViewer->update(localMap);
//        boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));

        ba();
        //mapViewer->update(localMap);
//        boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));

        //BA可能导致一些外点，不如把筛选过程放到BA后
        filtKeyFrames();
        //mapViewer->update(localMap);
//        boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));

        filtMapPoints(localMap);
        //mapViewer->update(localMap);
//        boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));

/*        prepareKeyFrame();
        triangulate();
        ba();
        //BA可能导致一些外点，不如把筛选过程放到BA后
        filtKeyFrames();
        filtMapPoints(localMap);

        mapViewer.update(localMap);*/

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

        //添加关键帧和地图点
        newMapPoints.clear();
        boost::mutex::scoped_lock lock(mapMutex);
        localMap->addFrame(currFrame);
        for (auto &point:triangulateMap->mapPoints) {
            localMap->addMapPoint(point);
            newMapPoints.insert(point);
        }
        lock.unlock();

        /*//BA
        Map::Ptr baMap(new Map);
        baMap->addFrame(refFrame);
        baMap->addFrame(currFrame);
        lock.lock();
        refFrame->forEachMapPoint(
                [&](auto &MapPoint) {
                    baMap->addMapPoint(MapPoint);
                }
        );
        currFrame->forEachMapPoint(
                [&](auto &MapPoint) {
                    baMap->addMapPoint(MapPoint);
                }
        );

        BA ba({BA::Mode_Fix_Intrinsic, BA::Mode_Fix_First_Frame});
        ba.loadMap(baMap);
        ba.ba();
        ba.writeMap();
        lock.unlock();

        //三角化BA后的地图点筛选
        filtMapPoints(triangulateMap);*/

#ifdef DEBUG
        lock.lock();
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "LocalMap: "
             << triangulateMap->mapPoints.size() << " points added to localMap. " << endl;
        lock.unlock();
#endif

    }

    void LocalMap::ba() {
        boost::mutex::scoped_lock lock(mapMutex);
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "LocalMap: ba... " << endl;
#endif
        BA ba({BA::Mode_Fix_First_2Frames});
        ba.loadMap(localMap);
        ba.ba();
        ba.writeMap();
        lock.unlock();
    }

    void LocalMap::filtKeyFrames() {
        boost::mutex::scoped_lock lock(mapMutex);

        //根据当前LocalMap地图点数自适应增减maxKeyFrames
        if (localMap->mapPoints.size() < minMapPoints && localMap->keyFrames.size() > maxKeyFrames)
            ++maxKeyFrames;
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "LocalMap: filtKeyFrames... " << endl;
        printVariable(maxKeyFrames);
        auto oldKeyFramesNum = localMap->keyFrames.size();
        printVariable(oldKeyFramesNum);
#endif
        //筛选关键帧
        int iFrames = 0;
        for (auto itFrame = localMap->keyFrames.rbegin(); itFrame != localMap->keyFrames.rend();) {
            int good = isGoodFrame(*itFrame);
            if (good > 0 && iFrames < maxKeyFrames) {
                ++itFrame;
                ++iFrames;
            } else {
                if (good > 0) {
                    //将该帧保存到map中
                    map->addFrame(*itFrame);
                } else {
                    //将不好的关键帧与地图点之间的关联断开
                    localMap->deleteObservation(*itFrame);
                }
                //删除被筛选掉的关键帧在LocalMap中的观测点,保留观测关系
                itFrame = list<KeyFrame::Ptr>::reverse_iterator(localMap->keyFrames.erase((++itFrame).base()));
            }
        }
        //删除 在当前LocalMap中没有观测帧 的点
        for (auto itMapPoint = localMap->mapPoints.begin(); itMapPoint != localMap->mapPoints.end();) {
            for (auto &keyFrame:localMap->keyFrames) {
                if ((*itMapPoint)->hasFrame(keyFrame)) {
                    ++itMapPoint;
                    break;
                }
                if (keyFrame == localMap->keyFrames.back()) {
                    map->addMapPoint(*itMapPoint);
                    itMapPoint = localMap->mapPoints.erase(itMapPoint);
                }
            }

        }
        if (localMap->mapPoints.size() > minMapPoints && maxKeyFrames > minKeyFrames)
            --maxKeyFrames;
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
             << "LocalMap: filtKeyFrames done. "
             << oldKeyFramesNum - localMap->keyFrames.size() << " filtered. "
             << localMap->keyFrames.size() << " keyFrames remained."
             << endl;
#endif
        lock.unlock();
    }

    int LocalMap::isGoodFrame(const KeyFrame::Ptr &keyFrame) const {
/*        float reprojErr = 0;
        int mapPointsNum = 0;
        keyFrame->forEachMapPoint([&](auto &mapPoint) {
            cv::Point2d reprojCoor;
            if (proj2frame(mapPoint, keyFrame, reprojCoor)) {
                reprojErr += disBetween<float>(
                        keyFrame->getKeyPointCoor(mapPoint->getIndex(keyFrame)),
                        reprojCoor
                );
                ++mapPointsNum;
            }
        });
        reprojErr /= mapPointsNum;
        if (reprojErr > maxReprojErr) {
            return -1;
        }*/

        return 1;
    }

    void LocalMap::filtMapPoints(Map::Ptr &map) {
        boost::mutex::scoped_lock lock(mapMutex);

#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "LocalMap: filtMapPoints... " << endl;
        auto oldMapPointsNum = map->mapPoints.size();
        printVariable(oldMapPointsNum);
        int numNoEnoughObservation = 0, numNotInView = 0, numNotInErrThres = 0, numNotInDisRange = 0;
        int errCode;
#endif
        for (auto it = map->mapPoints.begin(); it != map->mapPoints.end();) {
            errCode = isGoodPoint(*it);
            if (errCode > 0)
                ++it;
            else {
                map->deleteObservation(*it);
                it = map->mapPoints.erase(it);
#ifdef DEBUG
                switch (errCode) {
                    case -1:
                        ++numNoEnoughObservation;
                        break;
                    case -2:
                        ++numNotInView;
                        break;
                    case -3:
                        ++numNotInErrThres;
                        break;
                    case -4:
                        ++numNotInDisRange;
                        break;
                }
#endif
            }
        }
#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
             << "LocalMap: filtMapPoints done. "
             << oldMapPointsNum - map->mapPoints.size() << " filtered. "
             << map->mapPoints.size() << " mapPoints remained."
             << endl;
        printVariable(numNoEnoughObservation);
        printVariable(numNotInView);
        printVariable(numNotInErrThres);
        printVariable(numNotInDisRange);
#endif
        lock.unlock();
    }

    int LocalMap::isGoodPoint(const MapPoint::Ptr &mapPoint) const {
/*#ifdef DEBUG
        cout << "[" << boost::this_thread::get_id() << "]DEBUG: "   << "\t" << !setHas(newMapPoints, mapPoint) << "\t"
             << mapPoint->frame2indexs.size() << " frame2indexs" << endl;
#endif*/

        if (localMap->keyFrames.size() >= 4)
            if (!setHas(newMapPoints, mapPoint)
                && mapPoint->getFrameNum() < 3)
                return -1;


        bool inRangeNum = false;
        float reprojErr = 0;
        for (auto &frame2index:mapPoint->frame2indexs) {
            //根据到每个观测帧的最大距离来判断,只要有一个帧满足最远距离要求，该点满足距离要求
/*            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                 << "dis between MapPoint " << (void *) mapPoint.get()
                 << " and KeyFrame " << (void *) frame2index.first.get()
                 << " is " << disBetween(frame2index.first, mapPoint) << endl;*/
            if (!inRangeNum)
                if (disBetween(frame2index.first, mapPoint) < maxInlierPointDis)
                    inRangeNum = true;

            //根据重投影误差删除外点
            cv::Point2d reprojCoor;
            if (!proj2frame(mapPoint, frame2index.first, reprojCoor))
                return -2;
            reprojErr += disBetween<float>(frame2index.first->getKeyPointCoor(frame2index.second), reprojCoor);
        }
        reprojErr /= mapPoint->frame2indexs.size();
        if (reprojErr > maxReprojErr)
            return -3;
        if (!inRangeNum)
            return -4;

/*        //如果不被当前LocalMap中的关键帧观测，则过滤
        for (auto &keyFrame:localMap->keyFrames) {
            if (mapPoint->hasFrame(keyFrame))
                break;
            if (keyFrame == localMap->keyFrames.back())
                return false;
        }*/

        return 1;
    }

}