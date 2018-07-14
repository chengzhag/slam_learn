//
// Created by pidan1231239 on 18-7-13.
//

#ifndef SLAM_LEARN_VO_H
#define SLAM_LEARN_VO_H

#include "Initializer.h"

namespace sky {

    using namespace cv;

    class VO {
    public:
        typedef shared_ptr<VO> Ptr;
        cv::Ptr<DescriptorMatcher> matcher;
        Camera::Ptr camera;

        VO(const Camera::Ptr camera,
           const cv::Ptr<DescriptorMatcher> &matcher,
           cv::Ptr<cv::Feature2D> feature2D) :
                camera(camera), matcher(matcher), feature2D(feature2D) {
            initializer = Initializer::Ptr(new Initializer(matcher));
        }

        void step(Mat &image) {
            switch (state) {
                //初始化状态
                case 0: {
#ifdef DEBUG
                    cout << endl << "==============solving 2D-2D==============" << endl;
#endif
                    Frame::Ptr frame(new Frame(camera, image));
                    if (initializer->step(KeyFrame::Ptr(new KeyFrame(frame, image, feature2D)))) {
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

    protected:
        Initializer::Ptr initializer;
        cv::Ptr<cv::Feature2D> feature2D;
        int state = 0;
    };

}

#endif //SLAM_LEARN_VO_H
