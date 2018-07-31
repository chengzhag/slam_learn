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
           string featureType = Config::get<string>("VO.featureType"),
           int nfeatures = Config::get<int>("VO.nfeatures")
        ) :
                camera(camera),
                localMap(localMap),
                tracker(new Tracker(localMap)),
                initializer(new Initializer) {
            cout << "VO: Initializing..." << endl;
            coutVariable(featureType);
            coutVariable(nfeatures);

            if (featureType == "ORB")
                feature2D = cv::ORB::create(nfeatures);
            else if (featureType == "SIFT")
                feature2D = cv::xfeatures2d::SIFT::create(nfeatures, 3, 0.04, 10);
        }

        bool step(const Mat &image);

        inline int getState() const {
            return state;
        }

    };

}

#endif //SLAM_LEARN_VO_H
