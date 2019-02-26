#include "BasicLaserMapping.h"
#include "nanoflann_pcl.h"
#include "math_utils.h"

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

namespace loam
{

using std::sqrt;
using std::fabs;
using std::asin;
using std::atan2;
using std::pow;


BasicLaserMapping::BasicLaserMapping(const float& scanPeriod, const size_t& maxIterations) :
   _scanPeriod(scanPeriod),
   _stackFrameNum(1),
   _mapFrameNum(5),
   _frameCount(0),
   _mapFrameCount(0),
   _maxIterations(maxIterations),
   _deltaTAbort(0.05),
   _deltaRAbort(0.05),
   _laserCloudCenWidth(10), // 搜索邻域宽度, cm为单位
   _laserCloudCenHeight(5), // 搜索邻域高度, cm为单位
   _laserCloudCenDepth(10), // 搜索邻域深度, cm为单位
   _laserCloudWidth(21), // 子cube沿宽方向的分割个数，每个子cube 50mm =1m/20
   _laserCloudHeight(11), // 高方向个数 50mm
   _laserCloudDepth(21), // 深度方向个数 50mm
   _laserCloudNum(_laserCloudWidth * _laserCloudHeight * _laserCloudDepth), // 子cube总数
   _laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudFullRes(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudCornerStack(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurfStack(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudCornerStackDS(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurfStackDS(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurround(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurroundDS(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudCornerFromMap(new pcl::PointCloud<pcl::PointXYZI>()),
   _laserCloudSurfFromMap(new pcl::PointCloud<pcl::PointXYZI>())
{
   // initialize frame counter
   _frameCount = _stackFrameNum - 1;
   _mapFrameCount = _mapFrameNum - 1;

   // setup cloud vectors
   _laserCloudCornerArray.resize(_laserCloudNum);
   _laserCloudSurfArray.resize(_laserCloudNum);
   _laserCloudCornerDSArray.resize(_laserCloudNum);
   _laserCloudSurfDSArray.resize(_laserCloudNum);

   for (size_t i = 0; i < _laserCloudNum; i++)
   {
      _laserCloudCornerArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      _laserCloudSurfArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      _laserCloudCornerDSArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
      _laserCloudSurfDSArray[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
   }

   // setup down size filters
//   _downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
//   _downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
}


void BasicLaserMapping::pointAssociateToMap(const pcl::PointXYZI& pi, pcl::PointXYZI& po)
{
   pcl::PointXYZI newPoint = pi;

//   rotateZXY(newPoint, cur_state.yaw, cur_state.roll, cur_state.pitch);
//   rotateZXY(newPoint, cur_state.roll, cur_state.pitch, cur_state.yaw);
   rotateZXY(newPoint, cur_state.yaw, 0, 0);
//   rotateZXY(po, _transformTobeMapped.rot_z, _transformTobeMapped.rot_x, _transformTobeMapped.rot_y);

   po.x = newPoint.x + cur_state.x;
   po.y = newPoint.y + cur_state.y;
   po.z = newPoint.z + cur_state.z;
}


bool BasicLaserMapping::createDownsizedMap()
{
   // 每5帧生成一次地图
   _mapFrameCount++;
   if (_mapFrameCount < _mapFrameNum)
      return false;

   _mapFrameCount = 0;

   // accumulate map cloud
   _laserCloudSurround->clear();
   for (auto ind : _laserCloudSurroundInd)
   {
      *_laserCloudSurround += *_laserCloudCornerArray[ind];
      *_laserCloudSurround += *_laserCloudSurfArray[ind];
   }

   // down size map cloud
   _laserCloudSurroundDS->clear();
   DownsizePointCloud(*_laserCloudSurround, *_laserCloudSurroundDS, 3.0); // 降采样
   return true;
}

Eigen::Affine3f BasicLaserMapping::NAVDATA2Transform(const NAVDATA &nav) {
   Eigen::Affine3f transform_ = Eigen::Affine3f::Identity();
   transform_.translation() << nav.x, nav.y, nav.z;
   transform_.rotate(Eigen::AngleAxisf (nav.roll, Eigen::Vector3f::UnitX()));
   transform_.rotate(Eigen::AngleAxisf (nav.pitch, Eigen::Vector3f::UnitY()));
   transform_.rotate(Eigen::AngleAxisf (nav.yaw, Eigen::Vector3f::UnitZ()));
   return transform_;
}

void BasicLaserMapping::interpolate(const vector<NAVDATA>& data,
                                         const long long& time,
                                         NAVDATA& result)
{
    int pos = 0;
    int length = data.size();
    for(;pos < length && data[pos].millisec < time; pos++);
    if(pos == length){
        result = data[--pos];
        return;
    }
    if(length == 1){
        result = data[0];
        return;
    }
    float ratio = 1.0 * (time - data[pos-1].millisec) / (data[pos].millisec - data[pos-1].millisec);
    float invRatio = 1 - ratio;
    result.millisec = time;
    result.roll = invRatio * data[pos-1].roll + ratio * data[pos].roll;
    result.pitch = invRatio * data[pos-1].pitch + ratio * data[pos].pitch;
    result.yaw = invRatio * data[pos-1].yaw + ratio * data[pos].yaw;
    result.x = invRatio * data[pos-1].x + ratio * data[pos].x;
    result.y = invRatio * data[pos-1].y + ratio * data[pos].y;
    result.z = invRatio * data[pos-1].z + ratio * data[pos].z;
};


void BasicLaserMapping::DownsizePointCloud(const pcl::PointCloud<pcl::PointXYZI> & laserCloudIn,
                                           pcl::PointCloud<pcl::PointXYZI> & laserCloudOut,
                                           double filter_size) {
   if(laserCloudIn.points.size() <= 0)
   {
      laserCloudOut = laserCloudIn;
      return;
   }
   pcl::UniformSampling<pcl::PointXYZI> filter;
   filter.setInputCloud(laserCloudIn.makeShared());
   filter.setRadiusSearch(filter_size);
   pcl::PointCloud<int> keypointIndices;
   filter.compute(keypointIndices);
   pcl::copyPointCloud(laserCloudIn, keypointIndices.points, laserCloudOut);
}


void BasicLaserMapping::process(const pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn,
                               const pcl::PointCloud<pcl::PointXYZI>& cornerPointsSharp,
                               const pcl::PointCloud<pcl::PointXYZI>& surPointsFlat,
                               const long long& scanTime,
                               const std::vector<NAVDATA>& nav,
                               pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudMap)
{
#if false
   pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudInDS(new pcl::PointCloud<pcl::PointXYZI>);
   DownsizePointCloud(*laserCloudIn, *laserCloudInDS, 3.f);

   std::cout << "laserCloudInNum -> " << laserCloudIn->points.size() << std::endl;
   std::cout << "laserCloudInDSNum -> " << laserCloudInDS->points.size() << std::endl;

   /* 查找当前帧对应的nav信息 */
   int cur_index = 0;
   int NavDataNum = nav.size();
   for(; cur_index < NavDataNum && nav[cur_index].millisec < scanTime; cur_index++);

   std::cout << "scanTime - navTime = " << scanTime - nav[cur_index].millisec << std::endl;

   cur_state = nav[cur_index];

   pcl::PointXYZI pointSel_;

   pcl::PointCloud<pcl::PointXYZI> laserCloudInDSStack;

   for (auto const& pt : laserCloudInDS->points)
   {
      pointAssociateToMap(pt, pointSel_);
      laserCloudMap->push_back(pointSel_);
   }
   return;
#endif

   _frameCount++;
   if(_frameCount < _stackFrameNum)
   {
      return;
   }
   _frameCount = 0;

   pcl::PointXYZI pointSel;

   interpolate(nav, scanTime, _transformSum); /* 使用imu位姿信息初始化_transform */

   for(auto const& pt : cornerPointsSharp.points)
   {
      pointSel = pcl::transformPoint(pt,NAVDATA2Transform(_transform));
      _laserCloudCornerStack->push_back(pointSel);
   }

   for(auto const& pt : surPointsFlat.points)
   {
      pointSel = pcl::transformPoint(pt,NAVDATA2Transform(_transform));
      _laserCloudSurfStack->push_back(pointSel);
   }

   pcl::PointXYZI pointOnZAxis;
   pointOnZAxis.x = 0.0;
   pointOnZAxis.y = 0.0;
   pointOnZAxis.z = 10.0;
   pointOnZAxis = pcl::transformPoint(pointOnZAxis, NAVDATA2Transform(_transform));

   auto const CUBE_SIZE = 200.0;
   auto const CUBE_HALF = CUBE_SIZE / 2;

   /* 计算中心点(当前位置)在地图超立方体中的索引 */
   int centerCubeI = int((_transformSum.x + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
   int centerCubeJ = int((_transformSum.y + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
   int centerCubeK = int((_transformSum.z + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

   if (_transformSum.x + CUBE_HALF < 0) centerCubeI--;
   if (_transformSum.y + CUBE_HALF < 0) centerCubeJ--;
   if (_transformSum.z + CUBE_HALF < 0) centerCubeK--;

   while (centerCubeI < 3)
   {
      for (int j = 0; j < _laserCloudHeight; j++)
      {
         for (int k = 0; k < _laserCloudDepth; k++)
         {
            for (int i = _laserCloudWidth - 1; i >= 1; i--)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i - 1, j, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
         }
      }
      centerCubeI++;
      _laserCloudCenWidth++;
   }

   while (centerCubeI >= _laserCloudWidth - 3)
   {
      for (int j = 0; j < _laserCloudHeight; j++)
      {
         for (int k = 0; k < _laserCloudDepth; k++)
         {
            for (int i = 0; i < _laserCloudWidth - 1; i++)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i + 1, j, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
         }
      }
      centerCubeI--;
      _laserCloudCenWidth--;
   }

   while (centerCubeJ < 3)
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int k = 0; k < _laserCloudDepth; k++)
         {
            for (int j = _laserCloudHeight - 1; j >= 1; j--)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j - 1, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
         }
      }
      centerCubeJ++;
      _laserCloudCenHeight++;
   }

   while (centerCubeJ >= _laserCloudHeight - 3)
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int k = 0; k < _laserCloudDepth; k++)
         {
            for (int j = 0; j < _laserCloudHeight - 1; j++)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j + 1, k);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
         }
      }
      centerCubeJ--;
      _laserCloudCenHeight--;
   }

   while (centerCubeK < 3)
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int j = 0; j < _laserCloudHeight; j++)
         {
            for (int k = _laserCloudDepth - 1; k >= 1; k--)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j, k - 1);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
         }
      }
      centerCubeK++;
      _laserCloudCenDepth++;
   }

   while (centerCubeK >= _laserCloudDepth - 3)
   {
      for (int i = 0; i < _laserCloudWidth; i++)
      {
         for (int j = 0; j < _laserCloudHeight; j++)
         {
            for (int k = 0; k < _laserCloudDepth - 1; k++)
            {
               const size_t indexA = toIndex(i, j, k);
               const size_t indexB = toIndex(i, j, k + 1);
               std::swap(_laserCloudCornerArray[indexA], _laserCloudCornerArray[indexB]);
               std::swap(_laserCloudSurfArray[indexA], _laserCloudSurfArray[indexB]);
            }
         }
      }
      centerCubeK--;
      _laserCloudCenDepth--;
   }

   /* 在中心块周围5*5*5的范围内找对应点 */
   _laserCloudValidInd.clear();
   _laserCloudSurroundInd.clear();
   for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
   {
      for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
      {
         for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++)
         {
            if (i >= 0 && i < _laserCloudWidth &&
                j >= 0 && j < _laserCloudHeight &&
                k >= 0 && k < _laserCloudDepth)
            {

               float centerX = 200.0f * (i - _laserCloudCenWidth);
               float centerY = 200.0f * (j - _laserCloudCenHeight);
               float centerZ = 200.0f * (k - _laserCloudCenDepth);

               pcl::PointXYZI transform_pos; // 坐标变换 值为当前车辆位置
               transform_pos.x = _transformSum.x;
               transform_pos.y = _transformSum.y;
               transform_pos.z = _transformSum.z;

               /* 计算当前点是否在视野内 */
               bool isInLaserFOV = false;
               for (int ii = -1; ii <= 1; ii += 2)
               {
                  for (int jj = -1; jj <= 1; jj += 2)
                  {
                     for (int kk = -1; kk <= 1; kk += 2)
                     {
                        pcl::PointXYZI corner;
                        corner.x = centerX + 100.0f * ii;
                        corner.y = centerY + 100.0f * jj;
                        corner.z = centerZ + 100.0f * kk;

                        float squaredSide1 = calcSquaredDiff(transform_pos, corner);
                        float squaredSide2 = calcSquaredDiff(pointOnZAxis, corner);

                        float check1 = 100.0f + squaredSide1 - squaredSide2
                           - 10.0f * sqrt(3.0f) * sqrt(squaredSide1);

                        float check2 = 100.0f + squaredSide1 - squaredSide2
                           + 10.0f * sqrt(3.0f) * sqrt(squaredSide1);

                        if (check1 < 0 && check2 > 0)
                        {
                           isInLaserFOV = true;
                        }
                     }
                  }
               }

               size_t cubeIdx = i + _laserCloudWidth * j + _laserCloudWidth * _laserCloudHeight * k;
               if (isInLaserFOV)
               {
                  _laserCloudValidInd.push_back(cubeIdx);
               }
               _laserCloudSurroundInd.push_back(cubeIdx);
            }
         }
      }
   }

   /* 从地图中选择特征点用于位姿优化 */
   _laserCloudCornerFromMap->clear();
   _laserCloudSurfFromMap->clear();
   for (auto const& ind : _laserCloudValidInd)
   {
      *_laserCloudCornerFromMap += *_laserCloudCornerArray[ind];
      *_laserCloudSurfFromMap += *_laserCloudSurfArray[ind];
   }

   // prepare feature stack clouds for pose optimization
//   pcl::transformPointCloud(*_laserCloudCornerStack, *_laserCloudCornerStack, NAVDATA2Transform(_transformSum));
//   pcl::transformPointCloud(*_laserCloudSurfStack, *_laserCloudSurfStack, NAVDATA2Transform(_transformSum));

   // down sample feature stack clouds
   _laserCloudCornerStackDS->clear();
   _laserCloudCornerStackDS = _laserCloudCornerStack;
   DownsizePointCloud(*_laserCloudCornerStack, *_laserCloudCornerStackDS, 3.0); // 降采样

   size_t laserCloudCornerStackNum = _laserCloudCornerStackDS->size();

   _laserCloudSurfStackDS->clear();
   _laserCloudSurfStackDS = _laserCloudSurfStack;
   DownsizePointCloud(*_laserCloudSurfStack, *_laserCloudSurfStackDS, 3.0); // 降采样
   size_t laserCloudSurfStackNum = _laserCloudSurfStackDS->size();

   _laserCloudCornerStack->clear();
   _laserCloudSurfStack->clear();

   optimizeTransformTobeMapped();

   // store down sized corner stack points in corresponding cube clouds
   for (int i = 0; i < laserCloudCornerStackNum; i++)
   {
      pointSel = pcl::transformPoint(_laserCloudCornerStackDS->points[i], NAVDATA2Transform(_transformSum)); // 坐标变换

      /* 求pointSel所对应的index */
      int cubeI = int((pointSel.x + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
      int cubeJ = int((pointSel.y + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
      int cubeK = int((pointSel.z + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

      if (pointSel.x + CUBE_HALF < 0) cubeI--;
      if (pointSel.y + CUBE_HALF < 0) cubeJ--;
      if (pointSel.z + CUBE_HALF < 0) cubeK--;

      if (cubeI >= 0 && cubeI < _laserCloudWidth &&
          cubeJ >= 0 && cubeJ < _laserCloudHeight &&
          cubeK >= 0 && cubeK < _laserCloudDepth)
      {
         size_t cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;
         _laserCloudCornerArray[cubeInd]->push_back(pointSel);
      }
   }

   // store down sized surface stack points in corresponding cube clouds
   for (int i = 0; i < laserCloudSurfStackNum; i++)
   {
      pointSel = pcl::transformPoint(_laserCloudSurfStackDS->points[i], NAVDATA2Transform(_transformSum)); // 坐标变换

      int cubeI = int((pointSel.x + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenWidth;
      int cubeJ = int((pointSel.y + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenHeight;
      int cubeK = int((pointSel.z + CUBE_HALF) / CUBE_SIZE) + _laserCloudCenDepth;

      if (pointSel.x + CUBE_HALF < 0) cubeI--;
      if (pointSel.y + CUBE_HALF < 0) cubeJ--;
      if (pointSel.z + CUBE_HALF < 0) cubeK--;

      if (cubeI >= 0 && cubeI < _laserCloudWidth &&
          cubeJ >= 0 && cubeJ < _laserCloudHeight &&
          cubeK >= 0 && cubeK < _laserCloudDepth)
      {
         size_t cubeInd = cubeI + _laserCloudWidth * cubeJ + _laserCloudWidth * _laserCloudHeight * cubeK;
         _laserCloudSurfArray[cubeInd]->push_back(pointSel);
      }
   }

//    down size all valid (within field of view) feature cube clouds
   for (auto const& ind : _laserCloudValidInd)
   {
      _laserCloudCornerDSArray[ind]->clear();
      DownsizePointCloud(*_laserCloudCornerArray[ind], *_laserCloudCornerDSArray[ind], 3.0); // 降采样

      _laserCloudSurfDSArray[ind]->clear();
      DownsizePointCloud(*_laserCloudSurfArray[ind], *_laserCloudSurfDSArray[ind], 3.0); // 降采样

      // swap cube clouds for next processing
      _laserCloudCornerArray[ind].swap(_laserCloudCornerDSArray[ind]);
      _laserCloudSurfArray[ind].swap(_laserCloudSurfDSArray[ind]);
   }

   _downsizedMapCreated = createDownsizedMap();
   laserCloudMap = _laserCloudSurroundDS;
}


nanoflann::KdTreeFLANN<pcl::PointXYZI> kdtreeCornerFromMap;
nanoflann::KdTreeFLANN<pcl::PointXYZI> kdtreeSurfFromMap;


void BasicLaserMapping::optimizeTransformTobeMapped()
{
   if (_laserCloudCornerFromMap->size() <= 10 || _laserCloudSurfFromMap->size() <= 100)
      return;

   pcl::PointXYZI pointSel, pointOri, coeff;

   std::vector<int> pointSearchInd(5, 0);
   std::vector<float> pointSearchSqDis(5, 0);

   std::cout << "_laserCloudCornerFromMap -> " << _laserCloudCornerFromMap->size() << ", " << _laserCloudCornerFromMap->points.size() << std::endl;
   std::cout << "_laserCloudSurfFromMap -> " << _laserCloudSurfFromMap->size() << ", " << _laserCloudSurfFromMap->points.size() << std::endl;
   kdtreeCornerFromMap.setInputCloud(_laserCloudCornerFromMap);
   kdtreeSurfFromMap.setInputCloud(_laserCloudSurfFromMap);

   Eigen::Matrix<float, 5, 3> matA0;
   Eigen::Matrix<float, 5, 1> matB0;
   Eigen::Vector3f matX0;
   Eigen::Matrix3f matA1;
   Eigen::Matrix<float, 1, 3> matD1;
   Eigen::Matrix3f matV1;

   matA0.setZero();
   matB0.setConstant(-1);
   matX0.setZero();

   matA1.setZero();
   matD1.setZero();
   matV1.setZero();

   bool isDegenerate = false;
   Eigen::Matrix<float, 6, 6> matP;

   size_t laserCloudCornerStackNum = _laserCloudCornerStackDS->size();
   size_t laserCloudSurfStackNum = _laserCloudSurfStackDS->size();

   for (size_t iterCount = 0; iterCount < _maxIterations; iterCount++)
   {
      _laserCloudOri.clear();
      _coeffSel.clear();

      for (int i = 0; i < laserCloudCornerStackNum; i++)
      {
         pointOri = _laserCloudCornerStackDS->points[i];
         pointSel = pcl::transformPoint(pointOri, NAVDATA2Transform(_transformSum)); // 坐标变换
         kdtreeCornerFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

         if (pointSearchSqDis[4] < 1.0)
         {
            Vector3 vc(0, 0, 0);

            for (int j = 0; j < 5; j++)
               vc += Vector3(_laserCloudCornerFromMap->points[pointSearchInd[j]]);
            vc /= 5.0;

            Eigen::Matrix3f mat_a;
            mat_a.setZero();

            for (int j = 0; j < 5; j++)
            {
               Vector3 a = Vector3(_laserCloudCornerFromMap->points[pointSearchInd[j]]) - vc;

               mat_a(0, 0) += a.x() * a.x();
               mat_a(0, 1) += a.x() * a.y();
               mat_a(0, 2) += a.x() * a.z();
               mat_a(1, 1) += a.y() * a.y();
               mat_a(1, 2) += a.y() * a.z();
               mat_a(2, 2) += a.z() * a.z();
            }
            matA1 = mat_a / 5.0;

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(matA1);
            matD1 = esolver.eigenvalues().real();
            matV1 = esolver.eigenvectors().real();

            if (matD1(0, 2) > 3 * matD1(0, 1))
            {

               float x0 = pointSel.x;
               float y0 = pointSel.y;
               float z0 = pointSel.z;
               float x1 = vc.x() + 0.1 * matV1(0, 2);
               float y1 = vc.y() + 0.1 * matV1(1, 2);
               float z1 = vc.z() + 0.1 * matV1(2, 2);
               float x2 = vc.x() - 0.1 * matV1(0, 2);
               float y2 = vc.y() - 0.1 * matV1(1, 2);
               float z2 = vc.z() - 0.1 * matV1(2, 2);

               float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                 * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                                 + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                 * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                                 + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                                 * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

               float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

               float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                           + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

               float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                            - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

               float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                            + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

               float ld2 = a012 / l12;

               float s = 1 - 0.9f * fabs(ld2);

               coeff.x = s * la;
               coeff.y = s * lb;
               coeff.z = s * lc;
               coeff.intensity = s * ld2;

               if (s > 0.1)
               {
                  _laserCloudOri.push_back(pointOri);
                  _coeffSel.push_back(coeff);
               }
            }
         }
      }

      for (int i = 0; i < laserCloudSurfStackNum; i++)
      {
         pointOri = _laserCloudSurfStackDS->points[i];
         pointSel = pcl::transformPoint(pointOri, NAVDATA2Transform(_transformSum)); // 坐标变换
         kdtreeSurfFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

         if (pointSearchSqDis[4] < 1.0)
         {
            for (int j = 0; j < 5; j++)
            {
               matA0(j, 0) = _laserCloudSurfFromMap->points[pointSearchInd[j]].x;
               matA0(j, 1) = _laserCloudSurfFromMap->points[pointSearchInd[j]].y;
               matA0(j, 2) = _laserCloudSurfFromMap->points[pointSearchInd[j]].z;
            }
            matX0 = matA0.colPivHouseholderQr().solve(matB0);

            float pa = matX0(0, 0);
            float pb = matX0(1, 0);
            float pc = matX0(2, 0);
            float pd = 1;

            float ps = sqrt(pa * pa + pb * pb + pc * pc);
            pa /= ps;
            pb /= ps;
            pc /= ps;
            pd /= ps;

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
               if (fabs(pa * _laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                        pb * _laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                        pc * _laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2)
               {
                  planeValid = false;
                  break;
               }
            }

            if (planeValid)
            {
               float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
               float s = 1 - 0.9f * fabs(pd2) / sqrt(calcPointDistance(pointSel));

               coeff.x = s * pa;
               coeff.y = s * pb;
               coeff.z = s * pc;
               coeff.intensity = s * pd2;

               if (s > 0.1)
               {
                  _laserCloudOri.push_back(pointOri);
                  _coeffSel.push_back(coeff);
               }
            }
         }
      }

      float srx = sin(_transformSum.roll); // 坐标变换
      float crx = cos(_transformSum.roll);
      float sry = sin(_transformSum.pitch);
      float cry = cos(_transformSum.pitch);
      float srz = sin(_transformSum.yaw);
      float crz = cos(_transformSum.yaw);

      size_t laserCloudSelNum = _laserCloudOri.size();
      if (laserCloudSelNum < 50)
         continue;

      Eigen::Matrix<float, Eigen::Dynamic, 6> matA(laserCloudSelNum, 6);
      Eigen::Matrix<float, 6, Eigen::Dynamic> matAt(6, laserCloudSelNum);
      Eigen::Matrix<float, 6, 6> matAtA;
      Eigen::VectorXf matB(laserCloudSelNum);
      Eigen::VectorXf matAtB;
      Eigen::VectorXf matX;

      for (int i = 0; i < laserCloudSelNum; i++)
      {
         pointOri = _laserCloudOri.points[i];
         coeff = _coeffSel.points[i];

         float arx = (crx*sry*srz*pointOri.x + crx * crz*sry*pointOri.y - srx * sry*pointOri.z) * coeff.x
            + (-srx * srz*pointOri.x - crz * srx*pointOri.y - crx * pointOri.z) * coeff.y
            + (crx*cry*srz*pointOri.x + crx * cry*crz*pointOri.y - cry * srx*pointOri.z) * coeff.z;

         float ary = ((cry*srx*srz - crz * sry)*pointOri.x
                      + (sry*srz + cry * crz*srx)*pointOri.y + crx * cry*pointOri.z) * coeff.x
            + ((-cry * crz - srx * sry*srz)*pointOri.x
               + (cry*srz - crz * srx*sry)*pointOri.y - crx * sry*pointOri.z) * coeff.z;

         float arz = ((crz*srx*sry - cry * srz)*pointOri.x + (-cry * crz - srx * sry*srz)*pointOri.y)*coeff.x
            + (crx*crz*pointOri.x - crx * srz*pointOri.y) * coeff.y
            + ((sry*srz + cry * crz*srx)*pointOri.x + (crz*sry - cry * srx*srz)*pointOri.y)*coeff.z;

         matA(i, 0) = arx;
         matA(i, 1) = ary;
         matA(i, 2) = arz;
         matA(i, 3) = coeff.x;
         matA(i, 4) = coeff.y;
         matA(i, 5) = coeff.z;
         matB(i, 0) = -coeff.intensity;
      }

      matAt = matA.transpose();
      matAtA = matAt * matA;
      matAtB = matAt * matB;
      matX = matAtA.colPivHouseholderQr().solve(matAtB);

      if (iterCount == 0)
      {
         Eigen::Matrix<float, 1, 6> matE;
         Eigen::Matrix<float, 6, 6> matV;
         Eigen::Matrix<float, 6, 6> matV2;

         Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float, 6, 6> > esolver(matAtA);
         matE = esolver.eigenvalues().real();
         matV = esolver.eigenvectors().real();

         matV2 = matV;

         isDegenerate = false;
         float eignThre[6] = { 100, 100, 100, 100, 100, 100 };
         for (int i = 0; i < 6; i++)
         {
            if (matE(0, i) < eignThre[i])
            {
               for (int j = 0; j < 6; j++)
               {
                  matV2(i, j) = 0;
               }
               isDegenerate = true;
            }
            else
            {
               break;
            }
         }
         matP = matV.inverse() * matV2;
      }

      if (isDegenerate)
      {
         Eigen::Matrix<float, 6, 1> matX2(matX);
         matX = matP * matX2;
      }

      _transformSum.roll += matX(0, 0); // 坐标变换
      _transformSum.pitch += matX(1, 0);
      _transformSum.yaw += matX(2, 0);
      _transformSum.x += matX(3, 0);
      _transformSum.y += matX(4, 0);
      _transformSum.z += matX(5, 0);

      float deltaR = sqrt(pow(rad2deg(matX(0, 0)), 2) +
                          pow(rad2deg(matX(1, 0)), 2) +
                          pow(rad2deg(matX(2, 0)), 2));
      float deltaT = sqrt(pow(matX(3, 0) * 100, 2) +
                          pow(matX(4, 0) * 100, 2) +
                          pow(matX(5, 0) * 100, 2));

      if (deltaR < _deltaRAbort && deltaT < _deltaTAbort)
         break;
   }

//   transformUpdate();
}


} // end namespace loam