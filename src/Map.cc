/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::AddMapPlane(MapPlane *pMP) {
    unique_lock<mutex> lock(mMutexMap);
    mvpMapPlanes.push_back(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

vector<MapPlane*> Map::GetAllMapPlanes() {
    unique_lock<mutex> lock(mMutexMap);
    return mvpMapPlanes;
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::MapPlanesInMap() {
    unique_lock<mutex> lock(mMutexMap);
    return mvpMapPlanes.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    for(vector<MapPlane*>::iterator sit=mvpMapPlanes.begin(), send=mvpMapPlanes.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mvpMapPlanes.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::AssociatePlanes(ORB_SLAM2::KeyFrame *pF, const float &dTh, const float &aTh) {
    bool find;
    unique_lock<mutex> lock(mMutexMap);
    for (int i = 0; i < pF->mnPlaneNum; ++i) {
        find = true;
        for (int j = 0; find && j < mvpMapPlanes.size(); ++j) {
            cv::Mat pM = pF->ComputePlaneWorldCoeff(i);
            cv::Mat pW = mvpMapPlanes[j]->GetWorldPos();
            float d = pM.at<float>(3,0) - pW.at<float>(3,0);
            if(d > dTh || d < -dTh)
                continue;
            float angle = pM.at<float>(0,0) * pW.at<float>(0,0) +
                          pM.at<float>(1,0) * pW.at<float>(1,0) +
                          pM.at<float>(2,0) * pW.at<float>(2,0);
            if(angle < aTh)
                continue;

            find = false;
            pF->mvpMapPlanes[i] = mvpMapPlanes[j];

        }
    }
}
void Map::AssociatePlanes(ORB_SLAM2::Frame &pF, const float &dTh, const float &aTh) {
        bool finding = true;
        clock_t time1 = clock();
        unique_lock<mutex> lock(mMutexMap);
        for (int i = 0; i < pF.mnPlaneNum; ++i) {
            finding = true;
//            pF.mvpMapPlanes[i] = nullptr;
            for (int j = 0; finding && j < mvpMapPlanes.size(); ++j) {
                cv::Mat pM = pF.ComputePlaneWorldCoeff(i);
                cv::Mat pW = mvpMapPlanes[j]->GetWorldPos();
                float d = pM.at<float>(3,0) - pW.at<float>(3,0);
                if(d > dTh || d < -dTh)
                    continue;
                float angle = pM.at<float>(0,0) * pW.at<float>(0,0) +
                              pM.at<float>(1,0) * pW.at<float>(1,0) +
                              pM.at<float>(2,0) * pW.at<float>(2,0);
                if(angle < aTh)
                    continue;

                finding = false;
                pF.mvpMapPlanes[i] = mvpMapPlanes[j];
            }
            if(finding)
                pF.mbNewPlane = true;
        }
//        cout<< "Time of  association : " << 1000*(clock() - time1)/(double)CLOCKS_PER_SEC << "ms" << endl;
    }

    void Map::AssociatePlanes(ORB_SLAM2::Frame &pF, const float &dTh, const float &aTh,
            const float &verTh, const float &parTh, bool out) {

        unique_lock<mutex> lock(mMutexMap);
        bool find, findVer, findPer;
        pF.mbNewPlane = false;

        if(out)
            cout << "Plane associate in map  " << pF.mnId << " : " << endl;

        for (int i = 0; i < pF.mnPlaneNum; ++i) {
            find = true;
            findVer = true;
            findPer = true;
            pF.mvpMapPlanes[i] = static_cast<MapPlane*>(nullptr);
            pF.mvpVerticalPlanes[i] = static_cast<MapPlane*>(nullptr);
            pF.mvpParallelPlanes[i] = static_cast<MapPlane*>(nullptr);

            for (int j = 0; (find || findVer || findPer) && j < mvpMapPlanes.size(); ++j) {
                cv::Mat pM = pF.ComputePlaneWorldCoeff(i);
                cv::Mat pW = mvpMapPlanes[j]->GetWorldPos();

                float angle = pM.at<float>(0, 0) * pW.at<float>(0, 0) +
                              pM.at<float>(1, 0) * pW.at<float>(1, 0) +
                              pM.at<float>(2, 0) * pW.at<float>(2, 0);
                float d = pM.at<float>(3, 0) - pW.at<float>(3, 0);
                if(out)
                    cout << " angle : " << angle << "  d :" << d << "  ";
                if (find && (angle > aTh || angle < -aTh) && d < dTh && d > -dTh) // associate plane
                {
                    if(out)
                        cout << "  associate!" << endl;
                    find = false;
                    pF.mvpMapPlanes[i] = mvpMapPlanes[j];
                    continue;
                }

                // vertical planes
                if (findVer && angle < verTh && angle > -verTh) {
                    if(out)
                        cout << "  vertical!" << endl;
                    findVer = false;
                    pF.mvpVerticalPlanes[i] = mvpMapPlanes[j];
                    continue;
                }

                //parallel planes
                if (findPer && (angle > parTh || angle < -parTh)) {
                    if(out)
                        cout << "  parallel!" << endl;
                    findPer = false;
                    pF.mvpParallelPlanes[i] = mvpMapPlanes[j];
                }else{
                    if(out)
                        cout << endl;
                }
            }
            if (find) {
                pF.mbNewPlane = true;
                if(out)
                    cout << "Find New Plane! " << endl;
            }
            if(out)
                cout << endl;
        }

//        cout<< "Time of  association : " << 1000*(clock() - time1)/(double)CLOCKS_PER_SEC << "ms" << endl;
    }

    void Map::AssociatePlanesInFrame(ORB_SLAM2::Frame &pF, const float &dTh, const float &aTh, const float &verTh,
                                     const float &parTh, bool out) {

        unique_lock<mutex> lock(mMutexMap);
        bool find, findVer, findPer;
        pF.mbNewPlane = false;

        if(out)
            cout << "Plane associate in map  ID: " << pF.mnId << " : " << endl;

        for (int i = 0; i < pF.mnPlaneNum; ++i) {
            find = true;
            findVer = true;
            findPer = true;
            pF.mvpMapPlanes[i] = static_cast<MapPlane*>(nullptr);
            pF.mvpVerticalPlanes[i] = static_cast<MapPlane*>(nullptr);
            pF.mvpParallelPlanes[i] = static_cast<MapPlane*>(nullptr);
            cv::Mat pM = pF.mvPlaneCoefficients[i];
//            cout << "observe plane:  " << i << "  " << pM << endl;

            for (int j = 0; (find || findVer || findPer) && j < mvpMapPlanes.size(); ++j) {
                cv::Mat pW = ComputePlaneInFrame(pF,j);
                float angle = pM.at<float>(0, 0) * pW.at<float>(0, 0) +
                              pM.at<float>(1, 0) * pW.at<float>(1, 0) +
                              pM.at<float>(2, 0) * pW.at<float>(2, 0);
                float d = pM.at<float>(3, 0) - pW.at<float>(3, 0);
                if(out)
                    cout << " angle : " << angle << "  d :" << d << "  ";
                if (find && (angle > aTh || angle < -aTh) && d < dTh && d > -dTh) // associate plane
                {
                    if(out)
                        cout << "  associate!" << endl;
                    find = false;
                    pF.mvpMapPlanes[i] = mvpMapPlanes[j];
                    continue;
                }

                // vertical planes
                if (findVer && angle < verTh && angle > -verTh) {
                    if(out)
                        cout << "  vertical!" << endl;
                    findVer = false;
                    pF.mvpVerticalPlanes[i] = mvpMapPlanes[j];
                    continue;
                }

                //parallel planes
                if (findPer && (angle > parTh || angle < -parTh)) {
                    if(out)
                        cout << "  parallel!" << endl;
                    findPer = false;
                    pF.mvpParallelPlanes[i] = mvpMapPlanes[j];
                }else{
                    if(out)
                        cout << endl;
                }
            }
            if (find) {
                pF.mbNewPlane = true;
                if(out)
                    cout << "Find New Plane! " << endl;
            }
            if(out)
                cout << endl;
        }

//        cout<< "Time of  association : " << 1000*(clock() - time1)/(double)CLOCKS_PER_SEC << "ms" << endl;
    }

    void Map::AssociatePlanesByBoundary(ORB_SLAM2::Frame &pF, const float &dTh, const float &aTh, const float &verTh,
                                        const float &parTh, bool out) {

        unique_lock<mutex> lock(mMutexMap);
        pF.mbNewPlane = false;

        if(out)
            cout << "Plane associate in map  ID :  " << pF.mnId << "   num of Plane: "  << pF.mnPlaneNum << " TH: " << dTh << endl;

        for (int i = 0; i < pF.mnPlaneNum; ++i) {

            cv::Mat pM = pF.ComputePlaneWorldCoeff(i);
            int p = -1;
            if(out)
                cout << " plane  " << i << " : " << endl;
            float ldTh = dTh;
            float lverTh = verTh;
            float lparTh = parTh;
            for (int j = 0;j < mvpMapPlanes.size(); ++j) {
                cv::Mat pW = mvpMapPlanes[j]->GetWorldPos();

                float angle = pM.at<float>(0, 0) * pW.at<float>(0, 0) +
                              pM.at<float>(1, 0) * pW.at<float>(1, 0) +
                              pM.at<float>(2, 0) * pW.at<float>(2, 0);

                if(out)
                    cout << j << ":  angle : " << angle << endl;

                if ((angle > aTh || angle < -aTh)) // associate plane
                {

                    double dis = PointDistanceFromPlane(pM, mvpMapPlanes[j]->mvBoundaryPoints, out);
                    if(dis < ldTh) {
                        ldTh = dis;
                        if (out)
                            cout << "  associate!" << endl;
                        pF.mvpMapPlanes[i] = static_cast<MapPlane*>(nullptr);
                        pF.mvpMapPlanes[i] = mvpMapPlanes[j];

                        p = j;
                        continue;
                    }
                }

                // vertical planes
                if (angle < lverTh && angle > -lverTh) {
                    if(out)
                        cout << "  vertical!" << endl;
                    lverTh = abs(angle);
                    pF.mvpVerticalPlanes[i] = static_cast<MapPlane*>(nullptr);
                    pF.mvpVerticalPlanes[i] = mvpMapPlanes[j];
                    continue;
                }

                //parallel planes
                if ((angle > lparTh || angle < -lparTh)) {
                    if(out)
                        cout << "  parallel!" << endl;
                    lparTh = abs(angle);
                    pF.mvpParallelPlanes[i] = static_cast<MapPlane*>(nullptr);
                    pF.mvpParallelPlanes[i] = mvpMapPlanes[j];
                }else{
                    if(out)
                        cout << endl;
                }
            }
            if (p == -1) {
                pF.mbNewPlane = true;
                if(out)
                    cout << "Find New Plane! " << endl;
            }else{
                mvpMapPlanes[p]->UpdateBoundary(pF, i);
            }

            if(out)
                cout << endl;
        }

//        cout<< "Time of  association : " << 1000*(clock() - time1)/(double)CLOCKS_PER_SEC << "ms" << endl;
    }


    cv::Mat Map::ComputePlaneInFrame(const ORB_SLAM2::Frame &pF, int i) {
        cv::Mat temp;
        cv::transpose(pF.mTwc, temp);
        return temp*(mvpMapPlanes[i]->GetWorldPos());
}

double Map::PointDistanceFromPlane(const cv::Mat &plane, PointCloud::Ptr boundry, bool out) {
//    double sum = 0;
    double res = 100;
    if(out)
        cout << " compute dis: " << endl;
    for(auto p : boundry->points){
        double dis = abs(plane.at<float>(0, 0) * p.x +
                   plane.at<float>(1, 0) * p.y +
                   plane.at<float>(2, 0) * p.z +
                   plane.at<float>(3, 0));
        if(dis < res)
            res = dis;
//        if(out)
//            cout << " " << dis << " ";
//        sum += dis;
    }
//    res = sum / boundry->points.size();
    if(out)
        cout << endl << "ave : " << res << endl;
    return res;
}

} //namespace ORB_SLAM










