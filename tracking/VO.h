//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_VO_H
#define SLAM_LEARN_VO_H

#include "common_include.h"
#include "Initializer.h"
#include "LocalMap.h"
#include "Tracker.h"

namespace sky {

    class VO {

    protected:
        Initializer::Ptr initializer;
        cv::Ptr<cv::Feature2D> feature2D;
        int state = 0;

        LocalMap::Ptr localMap;

        Tracker::Ptr tracker;

    public:
        typedef shared_ptr<VO> Ptr;
        cv::Ptr<cv::DescriptorMatcher> matcher;
        Camera::Ptr camera;

        VO(Camera::Ptr camera,
           cv::Ptr<cv::DescriptorMatcher> matcher,
           cv::Ptr<cv::Feature2D> feature2D,
           LocalMap::Ptr localMap) :
                camera(camera), matcher(matcher), feature2D(feature2D),
                localMap(localMap), tracker(new Tracker(localMap, matcher)),
                initializer(new Initializer(matcher)) {}

        void step(Mat &image);

        int getState();

    };

}

#endif //SLAM_LEARN_VO_H
