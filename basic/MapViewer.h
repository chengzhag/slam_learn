//
// Created by pidan1231239 on 18-7-23.
//

#ifndef SLAM_LEARN_MAPVIEWER_H
#define SLAM_LEARN_MAPVIEWER_H

#include "Map.h"
#include "common_include.h"

#ifdef CLOUDVIEWER_DEBUG

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "Config.h"

#endif


namespace sky {

    class MapViewer {


    private:
#ifdef CLOUDVIEWER_DEBUG
        pcl::visualization::PCLVisualizer viewer;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        boost::thread thread;
        boost::mutex updateMutex;
        bool updateWait, wait4keyDown = false, trackCurrFrame;
        float disCamera;
#endif

    public:
#ifdef CLOUDVIEWER_DEBUG

        MapViewer(bool updateWait = Config::get<int>("MapViewer.updateWait"),
                  bool trackCurrFrame = Config::get<int>("MapViewer.trackCurrFrame"),
                  float disCamera = Config::get<float>("MapViewer.disCamera")
        ) :
                updateWait(updateWait),
                trackCurrFrame(trackCurrFrame),
                disCamera(disCamera),
                viewer("3D Viewer"),
                cloud(new pcl::PointCloud<pcl::PointXYZRGB>) {
#ifdef DEBUG
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                 << "MapViewer: Initializing..." << endl;
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                 << "MapViewer: usage: " << endl;
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                 << "\tpress 'Return' to jump to next frame" << endl;
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                 << "\tpress 'space' to skip waiting for all frames" << endl;
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                 << "\tpress 'up' to room in while tracking currFrame" << endl;
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                 << "\tpress 'down' to room out while tracking currFrame" << endl;
            cout << "[" << boost::this_thread::get_id() << "]DEBUG: "
                 << "\tpress 't' to switch tracking status" << endl;
            printVariable(updateWait);
            printVariable(trackCurrFrame);
            printVariable(disCamera);
#endif

            viewer.setBackgroundColor(
                    Config::get<double>("MapViewer.backgroundColor.r"),
                    Config::get<double>("MapViewer.backgroundColor.g"),
                    Config::get<double>("MapViewer.backgroundColor.b")
            );
            viewer.addPointCloud(cloud, "Triangulated Point Cloud");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    3,
                                                    "Triangulated Point Cloud");
            viewer.initCameraParameters();
            viewer.addCoordinateSystem(1.0);
            viewer.setCameraPosition(0, -disCamera, 0, 0, 0, 1);

            thread = boost::thread(boost::bind(&MapViewer::threadFunc, this));
            viewer.registerKeyboardCallback(boost::bind(&MapViewer::keyboardEventOccurred, this, _1));

        }

#endif

        void update(const Map::Ptr &map, bool drawNorms = false);

        ~MapViewer() {}

    private:
#ifdef CLOUDVIEWER_DEBUG

        void threadFunc();

        void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event);

        void addFrame(const KeyFrame::Ptr &frame, string camName = "");

#endif
    };

}


#endif //SLAM_LEARN_MAPVIEWER_H
