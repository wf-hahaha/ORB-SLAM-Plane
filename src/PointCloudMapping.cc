/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "PointCloudMapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include "Converter.h"
namespace ORB_SLAM2 {
    PointCloudMapping::PointCloudMapping() {

        mAllCloudPoints = boost::make_shared<PointCloud>();

        viewerThread = make_shared<thread>(bind(&PointCloudMapping::viewer, this));
    }

    void PointCloudMapping::shutdown() {
        {
            unique_lock<mutex> lck(shutDownMutex);
            shutDownFlag = true;
            keyFrameUpdated.notify_one();
        }
        viewerThread->join();
    }

    void PointCloudMapping::insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth) {
        cout << "receive a keyframe, id = " << kf->mnId << endl;
        unique_lock<mutex> lck(keyframeMutex);
        mvKeyframes.push_back(kf);

        keyFrameUpdated.notify_one();
    }


    void PointCloudMapping::viewer() {
        pcl::visualization::CloudViewer viewer("viewer");
        pcl::VoxelGrid<PointT>  voxel;
        voxel.setLeafSize( 0.04, 0.04, 0.04);
        while (true) {
            {
                unique_lock<mutex> lck_shutdown(shutDownMutex);
                if (shutDownFlag) {
                    break;
                }
            }
            {
                unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
                keyFrameUpdated.wait(lck_keyframeUpdated);
            }

            // keyframe is updated
            size_t N = 0;
            {
                unique_lock<mutex> lck(keyframeMutex);
                N = mvKeyframes.size();
            }

            for (size_t i = lastKeyframeSize; i < N; i++) {
                AddKFPointCloud(mvKeyframes[i]);
            }

            PointCloud::Ptr tmp(new PointCloud());
            voxel.setInputCloud( mAllCloudPoints );
            voxel.filter( *tmp );
            mAllCloudPoints->swap( *tmp );
            viewer.showCloud( mAllCloudPoints );

            lastKeyframeSize = N;
        }
    }

    void PointCloudMapping::AddKFPointCloud(KeyFrame *pKF) {
        for (int j = 0; j < pKF->mnPlaneNum; ++j) {
            int ir = pKF->mvpMapPlanes[j]->mRed;
            int ig = pKF->mvpMapPlanes[j]->mGreen;
            int ib = pKF->mvpMapPlanes[j]->mBlue;
            for(auto& p : pKF->mvPlanePoints[j].points){
                p.r = ir;
                p.g = ig;
                p.b = ib;
            }
            Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( pKF->GetPose() );
            PointCloud::Ptr cloud(new PointCloud);
            pcl::transformPointCloud( pKF->mvPlanePoints[j], *cloud, T.inverse().matrix());
            *mAllCloudPoints += *cloud;
        }
    }

}



















