//
// Created by pidan1231239 on 18-7-23.
//

#include "MapViewer.h"

namespace sky {

    void MapViewer::run() {
#ifdef CLOUDVIEWER_DEBUG
        boost::function0<void> f = boost::bind(&MapViewer::threadFunc, this);
        boost::thread t(f);
#endif
    }

    void MapViewer::update(Map::Ptr map) {
#ifdef CLOUDVIEWER_DEBUG
        if (!map)
            return;
#ifdef DEBUG
        cout << "MapViewer: Visualizing Point Could..." << endl;
        cout << "\t" << map->keyFrames.size() << " keyFrames" << endl;
        cout << "\t" << map->mapPoints.size() << " mapPoints" << endl;
#endif
        cloud->clear();
        for (auto point:map->mapPoints) {
            pcl::PointXYZRGB pointXYZ(point->rgb[0], point->rgb[1], point->rgb[2]);
            pointXYZ.x = point->pos(0);
            pointXYZ.y = point->pos(1);
            pointXYZ.z = point->pos(2);
            cloud->push_back(pointXYZ);
        }
        cloud->width = cloud->size();
        cloud->height = 1;
        viewer.updatePointCloud(cloud, "Triangulated Point Cloud");

        auto frame = map->keyFrames.back();
        if (lastFrame != frame) {
            addFrame(frame);
            lastFrame = frame;
        }
#endif
    }

#ifdef CLOUDVIEWER_DEBUG
    void MapViewer::threadFunc() {
        while (!viewer.wasStopped()) {
            viewer.spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100));
        }
    }
#endif

    void MapViewer::addFrame(KeyFrame::Ptr frame, string camName) {
#ifdef CLOUDVIEWER_DEBUG
        //添加一帧的位姿
        Eigen::Matrix4f camPose;
        auto T_c_w = frame->Tcw.inverse().matrix();
        for (int i = 0; i < camPose.rows(); ++i)
            for (int j = 0; j < camPose.cols(); ++j)
                camPose(i, j) = T_c_w(i, j);
        viewer.addCoordinateSystem(1.0, Eigen::Affine3f(camPose), camName);
#endif
    }

}
