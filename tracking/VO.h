//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_VO_H
#define SLAM_LEARN_VO_H

#include "common_include.h"
#include "Initializer.h"
#include "LocalMap.h"
#include "Tracker.h"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

namespace sky {

    class VO {

    protected:
        Initializer::Ptr initializer;
        cv::Ptr<cv::Feature2D> feature2D;
        int state = 0;

        LocalMap::Ptr localMap;
        Tracker::Ptr tracker;

        Camera::Ptr camera;

    public:
        typedef shared_ptr<VO> Ptr;

        VO(Camera::Ptr camera,
           LocalMap::Ptr localMap,
           string featureType = Config::get<string>("VO.featureType")
        ) :
                camera(camera),
                localMap(localMap),
                tracker(new Tracker(localMap)),
                initializer(new Initializer) {
            if (featureType == "ORB")
                feature2D = cv::ORB::create(Config::get<int>("VO.nfeatures"));
            else if (featureType == "SIFT")
                feature2D = cv::xfeatures2d::SIFT::create(Config::get<int>("VO.nfeatures"), 3, 0.04, 10);
        }

        void step(const Mat &image);

        inline int getState() const {
            return state;
        }

    };

}

#endif //SLAM_LEARN_VO_H
