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
#include "ORBextractor.h"

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
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: " << "VO: Initializing..." << endl;
            printVariable(featureType);
            printVariable(nfeatures);

            if (featureType == "ORB") {
/*                float scaleFactor = Config::get<float>("VO.ORB.scaleFactor");
                int nlevels = Config::get<int>("VO.ORB.nlevels");
                int edgeThreshold = Config::get<int>("VO.ORB.edgeThreshold");
                int firstLevel = Config::get<int>("VO.ORB.firstLevel");
                int WTA_K = Config::get<int>("VO.ORB.WTA_K");
                int patchSize = Config::get<int>("VO.ORB.patchSize");
                int fastThreshold = Config::get<int>("VO.ORB.fastThreshold");
                printVariable(scaleFactor);
                printVariable(nlevels);
                printVariable(edgeThreshold);
                printVariable(firstLevel);
                printVariable(WTA_K);
                printVariable(patchSize);
                printVariable(fastThreshold);

                feature2D = cv::ORB::create(nfeatures, scaleFactor, nlevels, edgeThreshold, firstLevel,
                                            WTA_K, cv::ORB::HARRIS_SCORE, patchSize, fastThreshold);*/


                float scaleFactor = Config::get<float>("VO.ORB.scaleFactor");
                int nlevels = Config::get<int>("VO.ORB.nlevels");
                int fastThreshold = Config::get<int>("VO.ORB.fastThreshold");
                printVariable(scaleFactor);
                printVariable(nlevels);
                printVariable(fastThreshold);

                feature2D = ORBextractor::create(nfeatures, scaleFactor, nlevels, ORBextractor::FAST_SCORE, fastThreshold);


            } else if (featureType == "SIFT") {
                auto nOctaveLayers = Config::get<int>("VO.SIFT.nOctaveLayers");
                auto contrastThreshold = Config::get<double>("VO.SIFT.contrastThreshold");
                auto edgeThreshold = Config::get<double>("VO.SIFT.edgeThreshold");
                auto sigma = Config::get<double>("VO.SIFT.sigma");
                printVariable(nOctaveLayers);
                printVariable(contrastThreshold);
                printVariable(edgeThreshold);
                printVariable(sigma);

                feature2D = cv::xfeatures2d::SIFT::create(
                        nfeatures, nOctaveLayers, contrastThreshold, edgeThreshold, sigma);
            }

        }

        bool step(const Mat &image);

        inline int getState() const {
            return state;
        }

    };

}

#endif //SLAM_LEARN_VO_H
