#pragma once
#include "nanoflann_pcl.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdio.h>

#include "Twist.h"
#include "../ScanRegistration/time_utils.h"

namespace loam
{

  /** \brief Implementation of the LOAM laser odometry component.
   *
   */
  class BasicLaserOdometry
  {
  public:
    explicit BasicLaserOdometry(float scanPeriod = 0.1, size_t maxIterations = 25);

    /** \brief Try to process buffered data. */
    void process(pcl::PointCloud<pcl::PointXYZ>& laserCloudIn,
            const Time& scanTime,
            pcl::PointCloud<pcl::PointXYZI>&,
            pcl::PointCloud<pcl::PointXYZI>&,
            pcl::PointCloud<pcl::PointXYZI>&,
            pcl::PointCloud<pcl::PointXYZI>&);

    size_t transformToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

    long long pointcloudTime;
  private:
    void transformToStart(const pcl::PointXYZI& pi, pcl::PointXYZI& po);

    void pluginIMURotation(const Angle& bcx, const Angle& bcy, const Angle& bcz,
                           const Angle& blx, const Angle& bly, const Angle& blz,
                           const Angle& alx, const Angle& aly, const Angle& alz,
                           Angle &acx, Angle &acy, Angle &acz);

    void accumulateRotation(Angle cx, Angle cy, Angle cz,
                            Angle lx, Angle ly, Angle lz,
                            Angle &ox, Angle &oy, Angle &oz);

  private:
    float _scanPeriod;       ///< time per scan
    long _frameCount;        ///< number of processed frames
    size_t _maxIterations;   ///< maximum number of iterations
    bool _systemInited;      ///< initialization flag


    float _deltaTAbort;     ///< optimization abort threshold for deltaT
    float _deltaRAbort;     ///< optimization abort threshold for deltaR

    pcl::PointCloud<pcl::PointXYZI>::Ptr _lastCornerCloud;    ///< last corner points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _lastSurfaceCloud;   ///< last surface points cloud

    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloudOri;      ///< point selection
    pcl::PointCloud<pcl::PointXYZI>::Ptr _coeffSel;           ///< point selection coefficients

    nanoflann::KdTreeFLANN<pcl::PointXYZI> _lastCornerKDTree;   ///< last corner cloud KD-tree
    nanoflann::KdTreeFLANN<pcl::PointXYZI> _lastSurfaceKDTree;  ///< last surface cloud KD-tree

    pcl::PointCloud<pcl::PointXYZI>::Ptr _cornerPointsSharp;      ///< sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _cornerPointsLessSharp;  ///< less sharp corner points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _surfPointsFlat;         ///< flat surface points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _surfPointsLessFlat;     ///< less flat surface points cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloud;             ///< full resolution cloud

    std::vector<int> _pointSearchCornerInd1;    ///< first corner point search index buffer
    std::vector<int> _pointSearchCornerInd2;    ///< second corner point search index buffer

    std::vector<int> _pointSearchSurfInd1;    ///< first surface point search index buffer
    std::vector<int> _pointSearchSurfInd2;    ///< second surface point search index buffer
    std::vector<int> _pointSearchSurfInd3;    ///< third surface point search index buffer

    Twist _transform;     ///< optimized pose transformation
    Twist _transformSum;  ///< accumulated optimized pose transformation

    Angle _imuRollStart, _imuPitchStart, _imuYawStart;
    Angle _imuRollEnd, _imuPitchEnd, _imuYawEnd;

    Vector3 _imuShiftFromStart;
    Vector3 _imuVeloFromStart;
  };

} // end namespace loam
