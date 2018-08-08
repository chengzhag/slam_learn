//
// Created by pidan1231239 on 18-7-13.
//

#include "Map.h"
#include <opencv2/cvv.hpp>
#include "basic.h"

namespace sky {

    void Map::addFrame(const KeyFrame::Ptr &frame) {
        if (frame)
            keyFrames.push_back(frame);
    }

    void Map::addMapPoint(const MapPoint::Ptr &mapPoint) {
        if (mapPoint)
            mapPoints.push_back(mapPoint);
    }

    KeyFrame::Ptr Map::getLastFrame() const {
        if (!keyFrames.empty())
            return keyFrames.back();
        else {
            cerr << "[" << boost::this_thread::get_id() << "]ERROR: "
                 << "Map: Cannot get lastFrame! No Frame in the map" << endl;
            return nullptr;
        }

    }

    bool Map::viewFrameProjInCVV(const KeyFrame::Ptr &frame, string message) const {
#ifdef CVVISUAL_DEBUGMODE
        //通过匹配点的方式可视化重投影误差
        vector<cv::KeyPoint> rawPoints, projPoints;
        vector<cv::DMatch> projMatches;
        int i = 0;
        for (const MapPoint::Ptr &mapPoint:mapPoints) {
            if (mapPoint->hasFrame(frame)) {
                cv::Point2f rawPos;
                mapPoint->getPixelCoor(frame, rawPos);
                cv::Point2d projPos;
                proj2frame(mapPoint, frame, projPos);

                rawPoints.push_back(cv::KeyPoint(cv::Point2d(rawPos), 1));
                projPoints.push_back(cv::KeyPoint(projPos, 1));
                projMatches.push_back(cv::DMatch(i, i, disBetween<float>(rawPos, projPos)));
                ++i;
            }
        }
        cvv::debugDMatch(frame->image, rawPoints, frame->image, projPoints, projMatches,
                         CVVISUAL_LOCATION,
                         message.c_str());
#endif
    }

}