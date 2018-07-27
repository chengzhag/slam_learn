//
// Created by pidan1231239 on 18-7-23.
//

#include "MapViewer.h"

namespace sky {

    void MapViewer::update(const Map::Ptr &map) {
#ifdef CLOUDVIEWER_DEBUG
        if (!map)
            return;
#ifdef DEBUG
        cout << "MapViewer: Visualizing " << map->keyFrames.size() << " keyFrames and " << map->mapPoints.size()
             << " mapPoints" << endl;
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
        cloud->is_dense = true;
        viewer.updatePointCloud(cloud, "Triangulated Point Cloud");

        //viewer.removeAllShapes();
        boost::mutex::scoped_lock lock(updateMutex);
        viewer.removeAllCoordinateSystems();
        int i = 0;
        for (auto &frame:map->keyFrames) {
            addFrame(frame, "frame" + to_string(++i));
        }
        lock.unlock();

        if (updateWait) {
            wait4keyMutex.lock();
        }
#endif
    }

#ifdef CLOUDVIEWER_DEBUG

    void MapViewer::threadFunc() {
        while (!viewer.wasStopped()) {
            boost::mutex::scoped_lock lock(updateMutex);
            viewer.spinOnce();
            lock.unlock();
            boost::this_thread::sleep_for(boost::chrono::milliseconds(15));
        }
    }

    void MapViewer::keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event) {
        if (event.getKeySym() == "Return" && event.keyDown()) {
            wait4keyMutex.unlock();
        } else if (event.getKeySym() == "space" && event.keyDown()) {
            wait4keyMutex.unlock();
            updateWait = !updateWait;
        }
    }

#endif

#ifdef CLOUDVIEWER_DEBUG

    void MapViewer::addFrame(const KeyFrame::Ptr &frame, string camName) {
        //添加一帧的位姿图标
        Eigen::Matrix4f camPose;
        //auto T_c_w = frame->Tcw.inverse();
        auto T_c_w = frame->Tcw.inverse().matrix();
        for (int i = 0; i < camPose.rows(); ++i)
            for (int j = 0; j < camPose.cols(); ++j)
                camPose(i, j) = T_c_w(i, j);
        viewer.addCoordinateSystem(1.0, Eigen::Affine3f(camPose), camName);
        /*Vector3d pointStart(0, 0, 0), pointTo(0, 0, 1);
        pointStart = T_c_w * pointStart;
        pointTo = T_c_w * pointStart;
        viewer.addLine(pcl::PointXYZ(pointStart[0], pointStart[1], pointStart[2]),
                       pcl::PointXYZ(pointTo[0], pointTo[1], pointTo[2]),
                       1, 1, 1,
                       camName);*/
    }

#endif
}
