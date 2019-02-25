#ifndef LOAM_LASERMAPPING_H
#define LOAM_LASERMAPPING_H


#include "BasicLaserMapping.h"

namespace loam
{
/** \brief Implementation of the LOAM laser mapping component.
 *
 */
class LaserMapping : public BasicLaserMapping
{
public:
   explicit LaserMapping(const float& scanPeriod = 0.1, const size_t& maxIterations = 10);

   /** \brief Setup component in active mode.
    *
    * @param node the ROS node handle
    * @param privateNode the private ROS node handle
    */
//   virtual bool setup(ros::NodeHandle& node, ros::NodeHandle& privateNode);

   /** \brief Handler method for a new last corner cloud.
    *
    * @param cornerPointsLastMsg the new last corner cloud message
    */
//   void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLastMsg);

   /** \brief Handler method for a new last surface cloud.
    *
    * @param surfacePointsLastMsg the new last surface cloud message
    */
//   void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& surfacePointsLastMsg);

   /** \brief Handler method for a new full resolution cloud.
    *
    * @param laserCloudFullResMsg the new full resolution cloud message
    */
//   void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg);

   /** \brief Handler method for a new laser odometry.
    *
    * @param laserOdometry the new laser odometry message
    */
//   void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry);

   /** \brief Handler method for IMU messages.
    *
    * @param imuIn the new IMU message
    */
//   void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn);

   /** \brief Process incoming messages in a loop until shutdown (used in active mode). */
//   void spin();

   /** \brief Try to process buffered data. */
   void process(const pcl::PointCloud<pcl::PointXYZI>::Ptr,
           const pcl::PointCloud<pcl::PointXYZI>& cornerPointsSharp,
           const pcl::PointCloud<pcl::PointXYZI>& surPointsFlat,
           const long long& scanTime,
           const std::vector<NAVDATA>&,
           pcl::PointCloud<pcl::PointXYZI>::Ptr&);


protected:
   /** \brief Reset flags, etc. */
   void reset();

private:
   Time _timeLaserCloudCornerLast;   ///< time of current last corner cloud
   Time _timeLaserCloudSurfLast;     ///< time of current last surface cloud
   Time _timeLaserCloudFullRes;      ///< time of current full resolution cloud
   Time _timeLaserOdometry;          ///< time of current laser odometry

   bool _newLaserCloudCornerLast;  ///< flag if a new last corner cloud has been received
   bool _newLaserCloudSurfLast;    ///< flag if a new last surface cloud has been received
   bool _newLaserCloudFullRes;     ///< flag if a new full resolution cloud has been received
   bool _newLaserOdometry;         ///< flag if a new laser odometry has been received


//   nav_msgs::Odometry _odomAftMapped;      ///< mapping odometry message
};

} // end namespace loam

#endif //LOAM_LASERMAPPING_H
