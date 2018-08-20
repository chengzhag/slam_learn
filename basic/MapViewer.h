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
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudLocal;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudMap;
        boost::thread thread;
        boost::mutex updateMutex;
        bool updateWait, wait4keyDown = false, trackCurrFrame, drawNorms;
        float disCamera;
#endif

    public:
        typedef shared_ptr<MapViewer> Ptr;
#ifdef CLOUDVIEWER_DEBUG

        MapViewer(bool updateWait = Config::get<int>("MapViewer.updateWait"),
                  bool trackCurrFrame = Config::get<int>("MapViewer.trackCurrFrame"),
                  float disCamera = Config::get<float>("MapViewer.disCamera"),
                  bool drawNorms = Config::get<int>("MapViewer.drawNorms")
        ) :
                updateWait(updateWait),
                trackCurrFrame(trackCurrFrame),
                disCamera(disCamera),
                drawNorms(drawNorms),
                viewer("3D Viewer"),
                cloudLocal(new pcl::PointCloud<pcl::PointXYZRGB>),
                cloudMap(new pcl::PointCloud<pcl::PointXYZRGB>) {
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
            printVariable(drawNorms);
#endif

            viewer.setBackgroundColor(
                    Config::get<double>("MapViewer.backgroundColor.r"),
                    Config::get<double>("MapViewer.backgroundColor.g"),
                    Config::get<double>("MapViewer.backgroundColor.b")
            );
            viewer.addPointCloud(cloudLocal, "cloudLocal");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    4,
                                                    "cloudLocal");
            viewer.addPointCloud(cloudMap, "cloudMap");
            viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                    2,
                                                    "cloudMap");

            viewer.initCameraParameters();
            viewer.addCoordinateSystem(1.0);
            viewer.setCameraPosition(0, -disCamera, 0, 0, 0, 1);

            thread = boost::thread(boost::bind(&MapViewer::threadFunc, this));
            viewer.registerKeyboardCallback(boost::bind(&MapViewer::keyboardEventOccurred, this, _1));

        }

#endif

        void update(const Map::Ptr &map, const Map::Ptr &localMap);

        ~MapViewer() {}

    private:
#ifdef CLOUDVIEWER_DEBUG

        void updateCloud(const Map::Ptr &map, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudLocal);

        void threadFunc();

        void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event);

        void addFrame(const KeyFrame::Ptr &frame, double size = 1.0, string camName = "");

#endif
    };

}


#endif //SLAM_LEARN_MAPVIEWER_H
