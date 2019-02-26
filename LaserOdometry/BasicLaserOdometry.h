#pragma once
#include "nanoflann_pcl.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <stdio.h>

#include "Twist.h"
#include "../ScanRegistration/time_utils.h"
#include "./DsvLoading/define.h"

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
    void process(const std::vector<NAVDATA>& nav,
            const long long& scanTime,
            pcl::PointCloud<pcl::PointXYZI>&,
            pcl::PointCloud<pcl::PointXYZI>&,
            pcl::PointCloud<pcl::PointXYZI>&,
            pcl::PointCloud<pcl::PointXYZI>&);

    size_t transformToEnd(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

    long long pointcloudTime;
  private:
    void interpolate(const vector<NAVDATA>& data, const long long& time, NAVDATA& result);

    /* 计算当前帧坐标系到上一帧坐标系的变换 */
    void transformToLast(const NAVDATA& last, const NAVDATA& cur, NAVDATA& diff);

    /* 计算当前帧坐标系到全局坐标系的变换 */
    void transformToGlobal(const NAVDATA& last, const NAVDATA& diff, NAVDATA& cur);

    Eigen::Affine3f NAVDATA2Transform(const NAVDATA& nav);
    NAVDATA Transform2NAVDATA(const Eigen::Affine3f& tranform_);

    /* 沿用旧函数名, 将点云投影到上一帧对应坐标系 */
//    void transformToStart(const pcl::PointXYZI& pi, pcl::PointXYZI& po);
    void transformToGlobal(const pcl::PointCloud<pcl::PointXYZI>& ori, pcl::PointCloud<pcl::PointXYZI>::Ptr& out);

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

    pcl::PointCloud<pcl::PointXYZI>::Ptr _laserCloud;             ///< full resolution cloud
    pcl::PointCloud<pcl::PointXYZI> _cornerPointsSharp; /* 投影到上一幀坐標系中的特徵點雲 */
    pcl::PointCloud<pcl::PointXYZI> _surfPointsFlat; /* 投影到上一幀坐標系中的特徵點雲 */

    std::vector<int> _pointSearchCornerInd1;    ///< first corner point search index buffer
    std::vector<int> _pointSearchCornerInd2;    ///< second corner point search index buffer

    std::vector<int> _pointSearchSurfInd1;    ///< first surface point search index buffer
    std::vector<int> _pointSearchSurfInd2;    ///< second surface point search index buffer
    std::vector<int> _pointSearchSurfInd3;    ///< third surface point search index buffer

    NAVDATA _transform;     ///< optimized pose transformation
    NAVDATA _transformSum;  ///< accumulated optimized pose transformation

    NAVDATA cur_pose_estimated;

    Angle _imuRollStart, _imuPitchStart, _imuYawStart;
    Angle _imuRollEnd, _imuPitchEnd, _imuYawEnd;

    Vector3 _imuShiftFromStart;
    Vector3 _imuVeloFromStart;
  };

} // end namespace loam
