#pragma once

#include "Twist.h"
#include "../ScanRegistration/CircularBuffer.h"
#include "../ScanRegistration/time_utils.h"
#include "./DsvLoading/define.h"


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/common/transforms.h>

namespace loam
{

/** IMU state data. */
typedef struct IMUState2
{
   /** The time of the measurement leading to this state (in seconds). */
   Time stamp;

   /** The current roll angle. */
   Angle roll;

   /** The current pitch angle. */
   Angle pitch;

   /** \brief Interpolate between two IMU states.
    *
    * @param start the first IMU state
    * @param end the second IMU state
    * @param ratio the interpolation ratio
    * @param result the target IMU state for storing the interpolation result
    */
   static void interpolate(const IMUState2& start,
                           const IMUState2& end,
                           const float& ratio,
                           IMUState2& result)
   {
      float invRatio = 1 - ratio;

      result.roll = start.roll.rad() * invRatio + end.roll.rad() * ratio;
      result.pitch = start.pitch.rad() * invRatio + end.pitch.rad() * ratio;
   };
} IMUState2;

class BasicLaserMapping
{
public:
   explicit BasicLaserMapping(const float& scanPeriod = 0.1, const size_t& maxIterations = 10);

   /** \brief Try to process buffered data. */
   void process(const pcl::PointCloud<pcl::PointXYZI>::Ptr,
                const pcl::PointCloud<pcl::PointXYZI>& cornerPointsSharp,
                const pcl::PointCloud<pcl::PointXYZI>& surPointsFlat,
                const long long& scanTime,
                const std::vector<NAVDATA>&,
                pcl::PointCloud<pcl::PointXYZI>::Ptr&);
private:
    Eigen::Affine3f NAVDATA2Transform(const NAVDATA& nav);

   /** Run an optimization. */
   void optimizeTransformTobeMapped();

   void transformAssociateToMap();
   void transformUpdate();
   void pointAssociateToMap(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
   void pointAssociateTobeMapped(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
   void transformFullResToMap();

   bool createDownsizedMap();

   // private:
   size_t toIndex(int i, int j, int k) const
   { return i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k; }

private:
   Time _laserOdometryTime;

    NAVDATA cur_state; /* 当前时刻位姿 */

   float _scanPeriod;          ///< time per scan
   const int _stackFrameNum;
   const int _mapFrameNum;
   long _frameCount;
   long _mapFrameCount;

   size_t _maxIterations;  ///< maximum number of iterations
   float _deltaTAbort;     ///< optimization abort threshold for deltaT
   float _deltaRAbort;     ///< optimization abort threshold for deltaR

   int _laserCloudCenWidth;
   int _laserCloudCenHeight;
   int _laserCloudCenDepth;
   const size_t _laserCloudWidth;
   const size_t _laserCloudHeight;
   const size_t _laserCloudDepth;
   const size_t _laserCloudNum;

   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerLast;   ///< last corner points cloud
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfLast;     ///< last surface points cloud
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudFullRes;      ///< last full resolution cloud

   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerStack;
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfStack;
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerStackDS;  ///< down sampled
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfStackDS;    ///< down sampled

   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudCornerArray;
   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudSurfArray;
   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudCornerDSArray;  ///< down sampled
   std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> _laserCloudSurfDSArray;    ///< down sampled

   std::vector<size_t> _laserCloudValidInd; /* 保存视野内点的索引 */
   std::vector<size_t> _laserCloudSurroundInd; /* 保存视野外点的索引 */

   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudCornerFromMap; /* 从map中找出的特征点 */
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurfFromMap; /* 从map中找出的特征点 */

   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurround; /* 地图 */
   pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudSurroundDS;     ///< down sampled


   pcl::PointCloud<pcl::PointXYZI> _laserCloudOri;
   pcl::PointCloud<pcl::PointXYZI> _coeffSel;

   Twist _transformSum, _transformIncre, _transformTobeMapped, _transformBefMapped, _transformAftMapped;

   CircularBuffer<IMUState2> _imuHistory;    ///< history of IMU states

//   pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterCorner;   ///< voxel filter for down sizing corner clouds
//   pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterSurf;     ///< voxel filter for down sizing surface clouds
//   pcl::VoxelGrid<pcl::PointXYZI> _downSizeFilterMap;      ///< voxel filter for down sizing accumulated map

   bool _downsizedMapCreated = false;

   NAVDATA _transform;
};

} // end namespace loam





