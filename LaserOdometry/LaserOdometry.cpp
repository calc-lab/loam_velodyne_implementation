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


  void LaserOdometry::process(pcl::PointCloud<pcl::PointXYZ>& laserCloudIn,
                              const Time& scanTime,
                              pcl::PointCloud<pcl::PointXYZI>& _cornerPointsSharp,
                              pcl::PointCloud<pcl::PointXYZI>& _cornerPointsLessSharp,
                              pcl::PointCloud<pcl::PointXYZI>& _surfPointsLessFlat,
                              pcl::PointCloud<pcl::PointXYZI>& _surfPointsFlat)
  {

    BasicLaserOdometry::process(laserCloudIn, scanTime, _cornerPointsSharp, _cornerPointsLessSharp, _surfPointsLessFlat, _surfPointsFlat);
  }

} // end namespace loam
