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
    PointCloudMapping::PointCloudMapping(Map *map): mMap(map) {

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

    void PointCloudMapping::insertKeyFrame(KeyFrame *kf) {
//        cout << "receive a keyframe, id = " << kf->mnId << endl;
        unique_lock<mutex> lck(keyframeMutex);
        mvKeyframes.push_back(kf);
        keyFrameUpdated.notify_one();
    }

//    void PointCloudMapping::viewer() {
//        pcl::visualization::CloudViewer viewer("viewer");
//        pcl::VoxelGrid<PointT>  voxel;
//        voxel.setLeafSize( 0.02, 0.02, 0.02);
//        while(1)
//        {
//            {
//                unique_lock<mutex> lck_shutdown(shutDownMutex);
//                if (shutDownFlag) {
//                    break;
//                }
//            }
//            vector<MapPlane*> PlaneMap = mMap->GetAllMapPlanes();
//            mAllCloudPoints->points.clear();
//            for(auto pMP : PlaneMap){
//                map<KeyFrame*, int> observations = pMP->GetObservations();
//                int ir = pMP->mRed;
//                int ig = pMP->mGreen;
//                int ib = pMP->mBlue;
//                for(auto mit = observations.begin(), mend = observations.end(); mit != mend; mit++){
//                    KeyFrame* frame = mit->first;
//                    int id = mit->second;
//                    for(auto& p : frame->mvPlanePoints[id].points){
//                        if(p.r != 255 && p.g != 255 && p.b != 255) {
//                            p.r = ir;
//                            p.g = ig;
//                            p.b = ib;
//                        }
//                    }
//                    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( frame->GetPose() );
//                    PointCloud::Ptr cloud(new PointCloud);
//                    pcl::transformPointCloud( frame->mvPlanePoints[id], *cloud, T.inverse().matrix());
//                    PointCloud::Ptr tmp(new PointCloud());
//                    voxel.setInputCloud( cloud );
//                    voxel.filter( *tmp );
//                    mAllCloudPoints->swap( *tmp );
//                    *mAllCloudPoints += *tmp;
//                }
//            }
//
////            PointCloud::Ptr tmp(new PointCloud());
////            voxel.setInputCloud( mAllCloudPoints );
////            voxel.filter( *tmp );
////            mAllCloudPoints->swap( *tmp );
//            viewer.showCloud( mAllCloudPoints );
//        }
//    }

    void PointCloudMapping::viewer() {
        boost::shared_ptr< pcl::visualization::PCLVisualizer > viewer(new pcl::visualization::PCLVisualizer("Plane viewer"));
        pcl::VoxelGrid<PointT>  voxel;
        voxel.setLeafSize( 0.02, 0.02, 0.02);
        viewer->setBackgroundColor(255, 255, 255);
        while(1)
        {
            viewer->spinOnce();
            {
                unique_lock<mutex> lck_shutdown(shutDownMutex);
                if (shutDownFlag) {
                    break;
                }
            }
//            {
//                unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
//                keyFrameUpdated.wait( lck_keyframeUpdated );
//            }

            // keyframe is updated
            size_t N=0;
            {
                unique_lock<mutex> lck( keyframeMutex );
                N = mvKeyframes.size();
            }

            for ( size_t i=lastKeyframeSize; i<N ; i++ )
            {
                KeyFrame* frame = mvKeyframes[i];
                for(int j = 0; j<frame->mvpMapPlanes.size();++j){
                    MapPlane* pMP = frame->mvpMapPlanes[j];
                    if(pMP== nullptr)
                        continue;

                    int ir = pMP->mRed;
                    int ig = pMP->mGreen;
                    int ib = pMP->mBlue;

                    if(j >= frame->mnRealPlaneNum){
                        break;
                    }
                    for(auto& p : frame->mvPlanePoints[j].points){
                        if(p.r != 255 && p.g != 255 && p.b != 255) {
                            p.r = ir;
                            p.g = ig;
                            p.b = ib;
                        }
                    }
                    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( frame->GetPose() );
                    PointCloud::Ptr cloud(new PointCloud);
                    pcl::transformPointCloud( frame->mvPlanePoints[j], *cloud, T.inverse().matrix());
                    PointCloud::Ptr tmp(new PointCloud());
                    voxel.setInputCloud( cloud );
                    voxel.filter( *tmp );
                    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(tmp);
                    std::stringstream cloudname;
                    cloudname << frame->mnId << "_" << j ;
                    viewer->addPointCloud(tmp, rgb, cloudname.str());
                }
            }
            lastKeyframeSize = N;
            viewer->spinOnce();
        }
    }

//    void PointCloudMapping::viewer() {
//        pcl::visualization::CloudViewer viewer("viewer");
//        pcl::VoxelGrid<PointT>  voxel;
//        voxel.setLeafSize( 0.04, 0.04, 0.04);
//        while (true) {
//            {
//                unique_lock<mutex> lck_shutdown(shutDownMutex);
//                if (shutDownFlag) {
//                    break;
//                }
//            }
//            {
//                unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
//                keyFrameUpdated.wait(lck_keyframeUpdated);
//            }
//
//            // keyframe is updated
//            size_t N = 0;
//            {
//                unique_lock<mutex> lck(keyframeMutex);
//                N = mvKeyframes.size();
//            }
//
//            for (size_t i = lastKeyframeSize; i < N; i++) {
//                AddKFPointCloud(mvKeyframes[i]);
//            }
//
//            PointCloud::Ptr tmp(new PointCloud());
//            voxel.setInputCloud( mAllCloudPoints );
//            voxel.filter( *tmp );
//            mAllCloudPoints->swap( *tmp );
//            viewer.showCloud( mAllCloudPoints );
//
//            lastKeyframeSize = N;
//        }
//    }
//
//    void PointCloudMapping::AddKFPointCloud(KeyFrame *pKF) {
//        for (int j = 0; j < pKF->mnPlaneNum; ++j) {
//            int ir = pKF->mvpMapPlanes[j]->mRed;
//            int ig = pKF->mvpMapPlanes[j]->mGreen;
//            int ib = pKF->mvpMapPlanes[j]->mBlue;
//            for(auto& p : pKF->mvPlanePoints[j].points){
//                p.r = ir;
//                p.g = ig;
//                p.b = ib;
//            }
//            Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( pKF->GetPose() );
//            PointCloud::Ptr cloud(new PointCloud);
//            pcl::transformPointCloud( pKF->mvPlanePoints[j], *cloud, T.inverse().matrix());
//            *mAllCloudPoints += *cloud;
//        }
//    }

}



















