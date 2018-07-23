//
// Created by pidan1231239 on 18-7-14.
//

#include "Tracker.h"

namespace sky {

    void Tracker::step(KeyFrame::Ptr frame) {
        if(solver3D2D.solve(localMap->map, frame)){
            //判断是否插入关键帧
            if (isKeyFrame(frame)) {
                localMap->addFrame(frame);
            } else {
                cout << "Tracker: InlierRatio " << solver3D2D.getInlierRatio()
                     << " is not enough!" << endl;
            }
        }
    }

    bool Tracker::isKeyFrame(KeyFrame::Ptr frame){
        return solver3D2D.getInlierRatio() > keyFrameMinInlierRatio
               && frame->dis2Coor(localMap->map->keyFrames.back()->Tcw.translation())>minKeyFrameDis;
    }

}