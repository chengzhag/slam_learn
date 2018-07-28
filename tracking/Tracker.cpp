//
// Created by pidan1231239 on 18-7-14.
//

#include "Tracker.h"

namespace sky {

    void Tracker::step(const KeyFrame::Ptr &frame) {
        ++frameInterval;
        this->frame = frame;
        boost::mutex::scoped_lock lock(localMap->mapMutex);
        bool solverPass = solver3D2D.solve(localMap->map, frame);
        lock.unlock();
        if (solverPass) {
            //判断是否插入关键帧
            if (isKeyFrame()) {
#ifdef DEBUG
                cout << "Tracker: Adding keyframe..." << endl;
#endif
                localMap->addFrame(frame);
                frameInterval = 0;
            }
        }
    }

    bool Tracker::isKeyFrame() const {
        //关键帧之间的最小间隔帧数
        if (frameInterval < minKeyFrameInterval) {
#ifdef DEBUG
            cout << "Tracker: Not a keyFrame. Num of frames to the last keyFrame " << frameInterval
                 << " is less than minKeyFrameInterval " << minKeyFrameInterval << endl;
#endif
            return false;
        }

        //关键帧的最小、最大间距
        boost::mutex::scoped_lock lock(localMap->mapMutex);
        auto dis2LastFrame = frame->getDis2(localMap->getLastFrame());
        lock.unlock();
        if (dis2LastFrame < minKeyFrameDis) {
#ifdef DEBUG
            cout << "Tracker: Not a keyFrame. Distance to the last keyFrame " << dis2LastFrame
                 << " is less than minKeyFrameDis " << minKeyFrameDis << endl;
#endif
            return false;
        }
        if (dis2LastFrame > maxKeyFrameDis) {
#ifdef DEBUG
            cout << "Tracker: Not a keyFrame. Distance to the last keyFrame " << dis2LastFrame
                 << " is more than maxKeyFrameDis " << maxKeyFrameDis << endl;
#endif
            return false;
        }

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