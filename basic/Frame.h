//
// Created by pidan1231239 on 18-7-12.
//

#ifndef SLAM_LEARN_FRAME_H
#define SLAM_LEARN_FRAME_H


#include "common_include.h"
#include "Camera.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace sky {

    class Frame {
    public:
        typedef shared_ptr<Frame> Ptr;
        SE3 Tcw;      // transform from world to camera
        Camera::Ptr camera;     // Pinhole RGBD Camera model
        int cols, rows;

        Frame(const Camera::Ptr camera, const Mat &image) :
                camera(camera), cols(image.cols), rows(image.rows) {}

        ~Frame();


        Vector3d getCamCenterEigen() const;

        cv::Mat getTcwMatCV(int rtype);

        cv::Mat getTcw34MatCV(int rtype);

        cv::Mat getTwcMatCV(int rtype);


        template<typename T>
        cv::Matx<T, 1, 3> getAngleAxisWcMatxCV() {
            Sophus::AngleAxisd angleAxis(Tcw.so3().matrix());
            auto axis = angleAxis.angle() * angleAxis.axis();
            cv::Matx<T, 1, 3> angleAxisCV(axis[0],axis[1],axis[2]);
            return angleAxisCV;
        }

        template<typename T>
        void setTcw(Matx<T,2,3> angleAxisAndTranslation){
            Tcw.so3()=SO3(angleAxisAndTranslation(0,0),
                          angleAxisAndTranslation(0,1),
                          angleAxisAndTranslation(0,2));
            Tcw.translation()=Vector3d(angleAxisAndTranslation(1,0),
                                       angleAxisAndTranslation(1,1),
                                       angleAxisAndTranslation(1,2));
        }



/*        cv::Mat getProjMatCV() {
            return camera->getKMatCV()*getTcw34MatCV();
        }*/


        // check if a point is in this frame
        bool isInFrame(const Vector3d &pt_world);

        //计算帧到某坐标的距离
        double dis2Coor(Sophus::Vector3d coor){
            Sophus::Vector3d coorFrom=Tcw.translation();
            double dis=0;
            for(int i=0;i<3;++i)
                dis+=pow(coorFrom[i]-coor[i],2);
            return sqrt(dis);
        }
    };

}



#endif //SLAM_LEARN_FRAME_H
