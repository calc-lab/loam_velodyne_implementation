#include <pcl/filters/filter.h>

#include "LaserOdometry.h"
#include "math_utils.h"

namespace loam
{

  using std::sin;
  using std::cos;
  using std::asin;
  using std::atan2;
  using std::sqrt;
  using std::fabs;
  using std::pow;

  LaserOdometry::LaserOdometry(float scanPeriod, uint16_t ioRatio, size_t maxIterations):
  _frameCount(0)
  {
  }

  void LaserOdometry::process(const std::vector<NAVDATA>& nav,
                              const long long& scanTime,
                              pcl::PointCloud<pcl::PointXYZI>& _cornerPointsSharp,
                              pcl::PointCloud<pcl::PointXYZI>& _cornerPointsLessSharp,
                              pcl::PointCloud<pcl::PointXYZI>& _surfPointsLessFlat,
                              pcl::PointCloud<pcl::PointXYZI>& _surfPointsFlat)
  {
    BasicLaserOdometry::process(nav, scanTime, _cornerPointsSharp, _cornerPointsLessSharp, _surfPointsLessFlat, _surfPointsFlat);
  }

} // end namespace loam
