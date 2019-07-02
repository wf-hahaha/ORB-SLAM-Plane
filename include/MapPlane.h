//
// Created by fishmarch on 19-5-24.
//

#ifndef ORB_SLAM2_MAPPLANE_H
#define ORB_SLAM2_MAPPLANE_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"
#include "Converter.h"

#include <opencv2/core/core.hpp>
#include <mutex>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>


namespace ORB_SLAM2 {
    class KeyFrame;
    class Frame;
    class MapPlane {
        typedef pcl::PointXYZRGB PointT;
        typedef pcl::PointCloud <PointT> PointCloud;
    public:
        MapPlane(const cv::Mat &Pos, KeyFrame* pRefKF, int idx, bool s = true);

        void SetWorldPos(const cv::Mat &Pos);
        cv::Mat GetWorldPos();

        void AddObservation(KeyFrame* pKF, int idx);
        void AddParObservation(KeyFrame* pKF, int idx);
        void AddVerObservation(KeyFrame* pKF, int idx);

        void EraseObservation(KeyFrame* pKF);
        map<KeyFrame*, int> GetObservations();
        map<KeyFrame*, int> GetParObservations();
        map<KeyFrame*, int> GetVerObservations();
        int GetIndexInKeyFrame(KeyFrame *pKF);
        void UpdateBoundary(const Frame& pF, int id);

    public:
        long unsigned int mnId; ///< Global ID for MapPlane;
        static long unsigned int nLastId;
        static std::mutex mGlobalMutex;
        long unsigned int mnBALocalForKF; //used in local BA
        PointCloud::Ptr mvBoundaryPoints;
        bool mbSeen;

        //used for visualization
        int mRed;
        int mGreen;
        int mBlue;
    protected:
        cv::Mat mWorldPos; ///< Position in absolute coordinates
        std::map<KeyFrame*, int> mObservations;
        std::map<KeyFrame*, int> mParObservations;
        std::map<KeyFrame*, int> mVerObservations;

        std::mutex mMutexPos;
        std::mutex mMutexFeatures;

//        void SetColor();
    };
}
#endif //ORB_SLAM2_MAPPLANE_H
