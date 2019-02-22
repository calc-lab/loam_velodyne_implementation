#ifndef LOAM_LASERODOMETRY_H
#define LOAM_LASERODOMETRY_H


#include "Twist.h"
#include "nanoflann_pcl.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "BasicLaserOdometry.h"

namespace loam
{

  class LaserOdometry : public BasicLaserOdometry
  {
  public:
    explicit LaserOdometry(float scanPeriod = 0.1, uint16_t ioRatio = 2, size_t maxIterations = 25);

    void process(const std::vector<NAVDATA>& nav,
            const long long& scanTime,
            pcl::PointCloud<pcl::PointXYZI>&,
            pcl::PointCloud<pcl::PointXYZI>&,
            pcl::PointCloud<pcl::PointXYZI>&,
            pcl::PointCloud<pcl::PointXYZI>&);

  private:
    uint16_t _ioRatio;       ///< ratio of input to output frames
    long _frameCount;
  };

} // end namespace loam

#endif //LOAM_LASERODOMETRY_H
