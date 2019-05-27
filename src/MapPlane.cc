//
// Created by fishmarch on 19-5-24.
//


#include "MapPlane.h"

#include<mutex>

namespace ORB_SLAM2{
    long unsigned int MapPlane::nLastId = 0;

    MapPlane::MapPlane(const cv::Mat &Pos, ORB_SLAM2::KeyFrame *pRefKF, int idx)

    {
        Pos.copyTo(mWorldPos);
        mnId = nLastId++;
        AddObservation(pRefKF, idx);
//        mPlanePoints = pRefKF->mvPlanePoints[idx];

        srand((int)time(nullptr));
        mRed = rand() % 256;
        mBlue = rand() % 256;
        mGreen = rand() % 256;
//        SetColor();
    }

//    void MapPlane::SetColor() {
//        for(auto& p : mPlanePoints){
//            p.r = mRed;
//            p.g = mGreen;
//            p.b = mBlue;
//        }
//    }

    void MapPlane::AddObservation(ORB_SLAM2::KeyFrame *pKF, int idx) {
        if(mObservations.count(pKF))
            return;
        mObservations[pKF] = idx;
    }

    void MapPlane::EraseObservation(ORB_SLAM2::KeyFrame *pKF) {
        if(mObservations.count(pKF)){
            mObservations.erase(pKF);
        }
    }
    void MapPlane::SetWorldPos(const cv::Mat &Pos)
    {
        unique_lock<mutex> lock(mMutexPos);
        Pos.copyTo(mWorldPos);
    }

    cv::Mat MapPlane::GetWorldPos(){
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos.clone();
    }
}