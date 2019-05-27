//
// Created by fishmarch on 19-5-24.
//

#ifndef ORB_SLAM2_MAPPLANE_H
#define ORB_SLAM2_MAPPLANE_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include <mutex>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>

class KeyFrame;
namespace ORB_SLAM2 {
    class MapPlane {
        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud <PointT> PointCloud;
    public:
        MapPlane(const cv::Mat &Pos, KeyFrame* pRefKF, int idx);

        void SetWorldPos(const cv::Mat &Pos);
        cv::Mat GetWorldPos();

        void AddObservation(KeyFrame* pKF, int idx);
        void EraseObservation(KeyFrame* pKF);

    public:
        long unsigned int mnId; ///< Global ID for MapPlane;
        static long unsigned int nLastId;
//        PointCloud mPlanePoints;
        int mRed;
        int mGreen;
        int mBlue;
    protected:
        cv::Mat mWorldPos; ///< Position in absolute coordinates
        std::map<KeyFrame*, int> mObservations;
        std::mutex mMutexPos;

//        void SetColor();
    };
}
#endif //ORB_SLAM2_MAPPLANE_H
