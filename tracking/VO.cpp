//
// Created by pidan1231239 on 18-7-13.
//

#include "VO.h"

namespace sky {

    bool VO::step(const Mat &image) {
        KeyFrame::Ptr keyFrame(new KeyFrame(camera, image, feature2D));
        switch (state) {
            //初始化状态
            case 0: {
                if (initializer->step(keyFrame)) {
                    //保存初始化的地图
                    localMap->init(initializer->initialMap);
                    state = 1;
                    initializer = nullptr;
                }
                break;
            }
                //追踪状态
            case 1: {
                if (!tracker->step(keyFrame)) {
                    state = 2;
                    cerr << "[" << boost::this_thread::get_id() << "]ERROR: " << "VO: Failed Tracking! " << endl;
                    return false;
                }
                break;
            }
            case 2: {

                break;
            }
        }

        //绘制当前帧和跟踪的关键点
        Mat imPoints;
        keyFrame->image.copyTo(imPoints);
        int i = 0;
        for (auto &keyPoint:keyFrame->keyPoints) {
            cv::Scalar color(255, 0, 0);
            int radius = 2;
            if (keyFrame->hasMapPoint(i++)) {
                color = cv::Scalar(0, 0, 255);
                radius = 5;
            }
            cv::circle(imPoints, keyPoint.pt, radius, color, 2);
        }
        cv::imshow("currKeyFrame and keyPoints",imPoints);
        cv::waitKey(1);

        return true;
    }

}
