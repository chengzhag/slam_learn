//
// Created by pidan1231239 on 18-7-26.
//

#include "basic.h"

namespace sky {

    bool proj2frame(const Vector3d &pt_world, const KeyFrame::Ptr &keyFrame, Vector2d &pixelColRow) {
        Vector3d p_cam = keyFrame->camera->world2camera(pt_world, keyFrame->Tcw);
        pixelColRow = keyFrame->camera->world2pixel(pt_world, keyFrame->Tcw);
        // cout<<"pt_world = "<<endl<<pt_world<<endl;
        // cout<<"P_pixel = "<<pixelColRow.transpose()<<endl<<endl;
        if (p_cam(2, 0) < 0) return false;
        return pixelColRow(0, 0) > 0
               && pixelColRow(1, 0) > 0
               && pixelColRow(0, 0) < keyFrame->image.cols
               && pixelColRow(1, 0) < keyFrame->image.rows;
    }

    float disBetween(const Sophus::Vector3d &coor1, const Sophus::Vector3d &coor2) {
        float dis = 0;
        for (int i = 0; i < 3; ++i)
            dis += pow(coor1[i] - coor2[i], 2);
        return sqrt(dis);
    }

}
