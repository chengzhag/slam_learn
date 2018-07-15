//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_VO_H
#define SLAM_LEARN_VO_H

#include "common_include.h"
#include "Initializer.h"
#include "LocalMap.h"

namespace sky {

    using namespace cv;

    class VO {
    public:
        typedef shared_ptr<VO> Ptr;
        cv::Ptr<DescriptorMatcher> matcher;
        Camera::Ptr camera;

        VO(const Camera::Ptr camera,
           const cv::Ptr<DescriptorMatcher> &matcher,
           cv::Ptr<cv::Feature2D> feature2D,
           LocalMap::Ptr localMap) :
                camera(camera), matcher(matcher), feature2D(feature2D), localMap(localMap) {
            initializer = Initializer::Ptr(new Initializer(matcher));
        }

        void step(Mat &image) {
            switch (state) {
                //初始化状态
                case 0: {
                    if (initializer->step(KeyFrame::Ptr(new KeyFrame(camera, image, feature2D)))) {
                        //保存初始化的地图
                        localMap->map = initializer->initialMap;
                        state = 1;
                        initializer = nullptr;
                    }
                    break;
                }
                    //追踪状态
                case 1: {

                    break;
                }
            }
        }

        int getState() {
            return state;
        }

    protected:
        Initializer::Ptr initializer;
        cv::Ptr<cv::Feature2D> feature2D;
        int state = 0;

        LocalMap::Ptr localMap;
    };

}

#endif //SLAM_LEARN_VO_H
