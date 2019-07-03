//
// Created by fishmarch on 19-5-24.
//


#include "MapPlane.h"

#include<mutex>

namespace ORB_SLAM2{
    long unsigned int MapPlane::nLastId = 0;
    mutex MapPlane::mGlobalMutex;

    MapPlane::MapPlane(const cv::Mat &Pos, ORB_SLAM2::KeyFrame *pRefKF, int idx, bool s):
    mnBALocalForKF(0), mvBoundaryPoints(new PointCloud()), mbSeen(s) {
        Pos.copyTo(mWorldPos);
        mnId = nLastId++;

        mRed = rand() % 255;
        mBlue = rand() % 255;
        mGreen = rand() % 255;

        Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(pRefKF->GetPose());
        if (s) {
            pcl::transformPointCloud(pRefKF->mvBoundaryPoints[idx], *mvBoundaryPoints, T.inverse().matrix());
            AddObservation(pRefKF, idx);
        } else {
            pcl::transformPointCloud(pRefKF->mvNotSeenBoundaryPoints[idx], *mvBoundaryPoints, T.inverse().matrix());
            AddNotSeenObservation(pRefKF, idx);
        }
    }

    void MapPlane::AddObservation(ORB_SLAM2::KeyFrame *pKF, int idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
            return;
        mObservations[pKF] = idx;
    }

    void MapPlane::AddNotSeenObservation(ORB_SLAM2::KeyFrame *pKF, int idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mNotSeenObservations.count(pKF))
            return;
        mNotSeenObservations[pKF] = idx;
    }

    void MapPlane::AddVerObservation(ORB_SLAM2::KeyFrame *pKF, int idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mVerObservations.count(pKF))
            return;
        mVerObservations[pKF] = idx;
    }

    void MapPlane::AddParObservation(ORB_SLAM2::KeyFrame *pKF, int idx) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mParObservations.count(pKF))
            return;
        mParObservations[pKF] = idx;
    }

    void MapPlane::EraseObservation(ORB_SLAM2::KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF)){
            mObservations.erase(pKF);
        }
    }

    void MapPlane::EraseNotSeenObservation(ORB_SLAM2::KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mNotSeenObservations.count(pKF)){
            mNotSeenObservations.erase(pKF);
        }
    }

    map<ORB_SLAM2::KeyFrame*, int> MapPlane::GetObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    }

    map<ORB_SLAM2::KeyFrame*, int> MapPlane::GetNotSeenObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mNotSeenObservations;
    }

    map<ORB_SLAM2::KeyFrame*, int> MapPlane::GetVerObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mVerObservations;
    }

    map<ORB_SLAM2::KeyFrame*, int> MapPlane::GetParObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mParObservations;
    }

    int MapPlane::GetIndexInKeyFrame(ORB_SLAM2::KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
            return mObservations[pKF];
        else
            return -1;
    }

    int MapPlane::GetNotSeenIndexInKeyFrame(ORB_SLAM2::KeyFrame *pKF) {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mNotSeenObservations.count(pKF))
            return mNotSeenObservations[pKF];
        else
            return -1;
    }

    void MapPlane::SetWorldPos(const cv::Mat &Pos)
    {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        Pos.copyTo(mWorldPos);
    }

    cv::Mat MapPlane::GetWorldPos(){
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos.clone();
    }

    void MapPlane::UpdateBoundary(const ORB_SLAM2::Frame &pF, int id) {
        Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( pF.mTcw );
        pcl::transformPointCloud( pF.mvBoundaryPoints[id], *mvBoundaryPoints, T.inverse().matrix());
    }
}