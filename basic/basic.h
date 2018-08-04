//
// Created by pidan1231239 on 18-8-4.
//

#ifndef SLAM_LEARN_BASIC_H
#define SLAM_LEARN_BASIC_H

#include <opencv2/opencv.hpp>
#include "MapPoint.h"
#include "KeyFrame.h"

namespace sky {


    //将3D点投影到帧，如果在帧里，返回真
    bool proj2frame(const Vector3d &pt_world, const KeyFrame::Ptr &keyFrame, Vector2d &pixelColRow);

    inline bool proj2frame(const MapPoint::Ptr &mapPoint, const KeyFrame::Ptr &keyFrame, Vector2d &pixelColRow) {
        return proj2frame(mapPoint->pos, keyFrame, pixelColRow);
    }

    inline bool proj2frame(const MapPoint::Ptr &mapPoint, const KeyFrame::Ptr &keyFrame, cv::Point2d &pixelColRow) {
        Vector2d pixelVec;
        bool r = proj2frame(mapPoint, keyFrame, pixelVec);
        pixelColRow.x = pixelVec[0];
        pixelColRow.y = pixelVec[1];
        return r;
    }

    //计算距离

    float disBetween(const Sophus::Vector3d &coor1, const Sophus::Vector3d &coor2);

    inline float disBetween(const Sophus::Vector3d &coor, const KeyFrame::Ptr &keyFrame) {
        return disBetween(coor, keyFrame->Tcw.translation());
    }

    inline float disBetween(const KeyFrame::Ptr &keyFrame, const Sophus::Vector3d &coor) {
        return disBetween(coor, keyFrame->Tcw.translation());
    }

    inline float disBetween(const KeyFrame::Ptr &keyFrame1, const KeyFrame::Ptr &keyFrame2) {
        return disBetween(keyFrame1->Tcw.translation(), keyFrame2->Tcw.translation());
    }

    inline float disBetween(const KeyFrame::Ptr &keyFrame, const MapPoint::Ptr &mapPoint) {
        return disBetween(mapPoint->pos, keyFrame);
    }

    // check if a point is in this frame
    inline bool isInFrame(const Vector3d &pt_world, const KeyFrame::Ptr &keyFrame) {
        Vector2d pixelColRow;
        return proj2frame(pt_world, keyFrame, pixelColRow);
    }

    inline bool isInFrame(const MapPoint::Ptr &mapPoint, const KeyFrame::Ptr &keyFrame) {
        Vector2d pixelColRow;
        return proj2frame(mapPoint->pos, keyFrame, pixelColRow);
    }

}

#endif //SLAM_LEARN_UTILITY_H
