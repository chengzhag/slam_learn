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
//#include <opencv2/xfeatures2d.hpp>

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
           cv::Ptr<cv::Feature2D> feature2D = cv::ORB::create(Config::get<int>("ORB.nfeatures"))) :
                camera(camera), feature2D(feature2D),
                localMap(localMap), tracker(new Tracker(localMap)),
                initializer(new Initializer) {}

        void step(Mat &image);

        int getState();

    };

}

#endif //SLAM_LEARN_VO_H
