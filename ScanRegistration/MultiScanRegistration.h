#ifndef LOAM_MULTISCANREGISTRATION_H
#define LOAM_MULTISCANREGISTRATION_H

#include <stdint.h>

#include "BasicScanRegistration.h"

namespace loam {
class MultiScanMapper {
public:
  /** \brief Construct a new multi scan mapper instance.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
  MultiScanMapper(const float& lowerBound = -24.9f,
                  const float& upperBound = 2,
                  const uint16_t& nScanRings = 64);

  const float& getLowerBound() { return _lowerBound; }
  const float& getUpperBound() { return _upperBound; }
  const uint16_t& getNumberOfScanRings() { return _nScanRings; }

  /** \brief Set mapping parameters.
   *
   * @param lowerBound - the lower vertical bound (degrees)
   * @param upperBound - the upper vertical bound (degrees)
   * @param nScanRings - the number of scan rings
   */
  void set(const float& lowerBound,
           const float& upperBound,
           const uint16_t& nScanRings);

  /** \brief Map the specified vertical point angle to its ring ID.
   *
   * @param angle the vertical point angle (in rad)
   * @return the ring ID
   */
  int getRingForAngle(const float& angle);

  /** Multi scan mapper for Velodyne VLP-16 according to data sheet. */
  static inline MultiScanMapper Velodyne_VLP_16() { return MultiScanMapper(-15, 15, 16); };

  /** Multi scan mapper for Velodyne HDL-32 according to data sheet. */
  static inline MultiScanMapper Velodyne_HDL_32() { return MultiScanMapper(-30.67f, 10.67f, 32); };

  /** Multi scan mapper for Velodyne HDL-64E according to data sheet. */
  static inline MultiScanMapper Velodyne_HDL_64E() { return MultiScanMapper(-24.9f, 2, 64); };


private:
  float _lowerBound;      ///< the vertical angle of the first scan ring
  float _upperBound;      ///< the vertical angle of the last scan ring
  uint16_t _nScanRings;   ///< number of scan rings
  float _factor;          ///< linear interpolation factor
};



/** \brief Class for registering point clouds received from multi-laser lidars.
 *
 */
class MultiScanRegistration : virtual public BasicScanRegistration
{
public:
  MultiScanRegistration(const MultiScanMapper& scanMapper = MultiScanMapper());
  void process(const pcl::PointCloud<pcl::PointXYZI>::Ptr,
          const Time& scanTime,
          pcl::PointCloud<pcl::PointXYZI> &,
          pcl::PointCloud<pcl::PointXYZI> &,
          pcl::PointCloud<pcl::PointXYZI> &,
          pcl::PointCloud<pcl::PointXYZI> &);

private:
  MultiScanMapper _scanMapper;  ///< mapper for mapping vertical point angles to scan ring IDs
  std::vector<pcl::PointCloud<pcl::PointXYZI>> _laserCloudScans;
};

} // end namespace loam


#endif //LOAM_MULTISCANREGISTRATION_H
