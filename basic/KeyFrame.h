//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_KEYFRAME_H
#define SLAM_LEARN_KEYFRAME_H

#include "common_include.h"
#include "Frame.h"
#include "MapPoint.h"
#include <opencv2/opencv.hpp>
#include <opencv2/cvv.hpp>
#include <unordered_map>

namespace sky {

    using namespace cv;

    class KeyFrame : public Frame {
    public:
        typedef shared_ptr<KeyFrame> Ptr;
        Mat image;
        vector<cv::KeyPoint> keyPoints;
        Mat descriptors;
        unordered_map<int, MapPoint::Ptr> mapPoints;//在descriptors或keyPoints中的序号和对应的地图点

        KeyFrame(const Camera::Ptr &camera, const Mat &image, cv::Ptr<cv::Feature2D> feature2D) :
                Frame(camera, image), image(image) {
            //TODO:将特征检测移出构造函数
#ifdef DEBUG
            cout << "detectAndCompute features: " << endl;
#endif
            feature2D->detectAndCompute(image, noArray(), keyPoints, descriptors);
#ifdef DEBUG
            cout << "\tfound " << keyPoints.size() << " keypoints" << endl;
#endif
        }

        //在descriptors或keyPoints中的序号
        bool hasMapPoint(int i) {
            return mapPoints.find(i) != mapPoints.end();
        }

        MapPoint::Ptr getMapPoint(int i) {
            return mapPoints[i];
        }

        Point2f getKeyPointCoor(int i) {
            return keyPoints[i].pt;
        }

        void addMapPoint(int i, MapPoint::Ptr &mapPoint) {
            mapPoints[i] = mapPoint;
        }
    };

}


#endif //SLAM_LEARN_KEYFRAME_H
