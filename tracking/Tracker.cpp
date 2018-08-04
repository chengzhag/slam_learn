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
        boost::mutex::scoped_lock lock(localMap->mapMutex);
        bool solverPass = solver3D2D.solve(localMap->map, frame);
        //如果tracking丢失，且关键帧添加未完成，等待上一个关键帧的处理,以获得更多地图点
        if (!solverPass && isAdding) {
#ifdef DEBUG
            cout << "Tracker: solverPass failed! waiting for adding more mapPoints... " << endl;
#endif
            lock.unlock();
            localMap->waitForThread();
            lock.lock();
            solverPass = solver3D2D.solve(localMap->map, frame);
        }
        //如果仍丢失，判定跟踪失败
        if (!solverPass) {
#ifdef CVVISUAL_DEBUGMODE
#ifdef DEBUG
            cerr << "Tracker: solverPass failed! showing reprojection of last keyFrame... " << endl;
#endif
            localMap->map->viewFrameProjInCVV(localMap->getLastFrame());
#endif
            lock.unlock();
            return false;
        }
        lock.unlock();
        //如果跟踪成功，判定是否为关键帧
        if (solverPass) {
            //判断是否插入关键帧
            if (isKeyFrame()) {
#ifdef DEBUG
                cout << "Tracker: Adding keyframe..." << endl;
#endif
                localMap->waitForThread();
                //如果到添加关键帧前还没有显示过，则显示
                if (!showedCVV) {
                    localMap->viewMatchInCVV();
                    localMap->viewReprojInCVV();
                    showedCVV = true;
                }
                localMap->addFrame(frame);
                frameInterval = 0;
                showedCVV = false;
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
            cout << "Tracker: Not a keyFrame. Distance to the last keyFrame " << dis2LastFrame
                 << " is less than minKeyFrameDis " << minKeyFrameDis << endl;
#endif
            return false;
        }
        if (dis2LastFrame > 2*maxKeyFrameDis) {
#ifdef DEBUG
            cout << "Tracker: Not a keyFrame. Distance to the last keyFrame " << dis2LastFrame
                 << " is more than 2*maxKeyFrameDis " << 2*maxKeyFrameDis << endl;
#endif
            return false;
        }
        if (dis2LastFrame > maxKeyFrameDis) {
#ifdef DEBUG
            cout << "Tracker: Is a keyFrame. Distance to the last keyFrame " << dis2LastFrame
                 << " is more than maxKeyFrameDis " << maxKeyFrameDis << endl;
#endif
            return true;
        }

        //关键帧解PnP所用地图点数相对于上一关键帧的最大比例
        int lastKeyFrameTrackingNum;
        if (localMapFrameNum >= 3)
            lastKeyFrameTrackingNum = lasFrame->inlierPnPnum;
        else
            lastKeyFrameTrackingNum = localMapPointNum;
        auto trackRatio2LastFrame = (float) solver3D2D.getInlierNum() / lastKeyFrameTrackingNum;
        coutVariable(lastKeyFrameTrackingNum);
        coutVariable(solver3D2D.getInlierNum());
        coutVariable(trackRatio2LastFrame);
        if (trackRatio2LastFrame < maxKeyFrameTrackRatio) {
#ifdef DEBUG
            cout << "Tracker: Is a keyFrame. TrackRatio2LastFrame " << trackRatio2LastFrame
                 << " is less than maxKeyFrameTrackRatio " << maxKeyFrameTrackRatio << endl;
#endif
            return true;
        }

        //关键帧之间的最小间隔帧数
        if (frameInterval < minKeyFrameInterval) {
#ifdef DEBUG
            cout << "Tracker: Not a keyFrame. Num of frames to the last keyFrame " << frameInterval
                 << " is less than minKeyFrameInterval " << minKeyFrameInterval << endl;
#endif
            return false;
        }

        //关键帧之间的最小间隔帧数
        if (solver3D2D.getInlierNum() < minKeyFrameInlierNum) {
#ifdef DEBUG
            cout << "Tracker: Not a keyFrame. InlierNum " << solver3D2D.getInlierNum()
                 << " is less than minKeyFrameInlierNum " << minKeyFrameInlierNum << endl;
#endif
            return false;
        }

        return true;
    }

}