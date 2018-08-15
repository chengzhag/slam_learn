//
// Created by pidan1231239 on 18-7-14.
//

#include "Tracker.h"
#include "basic.h"

namespace sky {

    bool Tracker::step(const KeyFrame::Ptr &frame) {
        static bool showedCVV = true;

        ++frameInterval;
        this->frame = frame;

        bool isAdding = localMap->isAdding();
#ifdef DEBUG
        if (isAdding)
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                 << "Tracker: Still adding keyFrame... " << endl;
#endif

        bool solverPass = solver3D2D.solve(frame);
        //如果tracking丢失，且关键帧添加未完成，等待上一个关键帧的处理,以获得更多地图点
        if (!solverPass && isAdding) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                 << "Tracker: Solver3D2D failed! Waiting for adding more mapPoints... " << endl;
#endif
            localMap->waitForThread();

            solverPass = solver3D2D.solve(frame);
        }
        //如果仍丢失，判定跟踪失败
        if (!solverPass) {
#ifdef CVVISUAL_DEBUGMODE
#ifdef DEBUG
            cerr << "[" << boost::this_thread::get_id() << "]ERROR: "
                 << "Tracker: Solver3D2D failed! Showing reprojection of last keyFrame... " << endl;
#endif
            localMap->map->viewFrameProjInCVV(localMap->getLastFrame(),
                                              "Tracker: Solver3D2D failed! LocalMap proj to crrFrame");
#endif
            return false;
        } else {
            //判断是否插入关键帧
            if (isKeyFrame()) {
#ifdef DEBUG
                cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Tracker: Adding keyframe..." << endl;
#endif
                localMap->waitForThread();
                //如果到添加关键帧前还没有显示过，则显示
                if (!showedCVV) {
                    localMap->viewMatchInCVV();
                    localMap->viewReprojInCVV();
                    showedCVV = true;
                }
                solver3D2D.addFrame2inliers();
                localMap->addFrame(frame);
                frameInterval = 0;
                showedCVV = false;
            } else {
                solver3D2D.addFrame2inliers(false);
            }
        }

        //如果后面没有关键帧添加，可以等上一关键帧添加结束显示
        if (!showedCVV && !localMap->isAdding()) {
            localMap->viewMatchInCVV();
            localMap->viewReprojInCVV();
            showedCVV = true;
        }

        return true;
    }

    bool Tracker::isKeyFrame() const {
        boost::mutex::scoped_lock lock(localMap->mapMutex);
        auto lasFrame = localMap->getLastFrame();
        auto dis2LastFrame = disBetween(frame, lasFrame);
        auto localMapFrameNum = localMap->map->keyFrames.size();
        auto localMapPointNum = localMap->map->mapPoints.size();
        lock.unlock();


        //关键帧的最小、最大间距
        if (dis2LastFrame < minKeyFrameDis) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                 << "Tracker: Not a keyFrame. Distance to the last keyFrame " << dis2LastFrame
                 << " is less than minKeyFrameDis " << minKeyFrameDis << endl;
#endif
            return false;
        }
        if (dis2LastFrame > maxKeyFrameDis) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                 << "Tracker: Not a keyFrame. Distance to the last keyFrame " << dis2LastFrame
                 << " is more than maxKeyFrameDis " << maxKeyFrameDis << endl;
#endif
            return false;
        }
/*        if (dis2LastFrame > maxKeyFrameDis) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                 << "Tracker: Is a keyFrame. Distance to the last keyFrame " << dis2LastFrame
                 << " is more than maxKeyFrameDis " << maxKeyFrameDis << endl;
#endif
            return true;
        }*/

        //关键帧之间的最小间隔帧数
        if (frameInterval < minKeyFrameInterval) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                 << "Tracker: Not a keyFrame. Num of frames to the last keyFrame " << frameInterval
                 << " is less than minKeyFrameInterval " << minKeyFrameInterval << endl;
#endif
            return false;
        }

        //关键帧的最少地图点
        if (solver3D2D.getInlierNum() < minKeyFrameInlierNum) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "Tracker: Not a keyFrame. InlierNum "
                 << solver3D2D.getInlierNum()
                 << " is less than minKeyFrameInlierNum " << minKeyFrameInlierNum << endl;
#endif
            return false;
        }

        //关键帧解PnP所用地图点数相对于上一关键帧的最大比例
        int lastKeyFrameTrackingNum;
        if (localMapFrameNum >= 3)
            lastKeyFrameTrackingNum = lasFrame->getMapPointsNum();
        else
            lastKeyFrameTrackingNum = localMapPointNum;
        auto trackRatio2LastFrame = (float) solver3D2D.getInlierNum() / lastKeyFrameTrackingNum;
        printVariable(lastKeyFrameTrackingNum);
        printVariable(solver3D2D.getInlierNum());
        printVariable(trackRatio2LastFrame);
        if (trackRatio2LastFrame > maxKeyFrameTrackRatio) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                 << "Tracker: Not a keyFrame. TrackRatio2LastFrame "
                 << trackRatio2LastFrame
                 << " is more than maxKeyFrameTrackRatio " << maxKeyFrameTrackRatio << endl;
#endif
            return false;
        }


        return true;
    }

}