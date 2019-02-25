#include "LaserMapping.h"

namespace loam
{

LaserMapping::LaserMapping(const float& scanPeriod, const size_t& maxIterations)
{
   // initialize mapping odometry and odometry tf messages
//   _odomAftMapped.header.frame_id = "/camera_init";
//   _odomAftMapped.child_frame_id = "/aft_mapped";

//   _aftMappedTrans.frame_id_ = "/camera_init";
//   _aftMappedTrans.child_frame_id_ = "/aft_mapped";
}

/*
bool LaserMapping::setup(ros::NodeHandle& node, ros::NodeHandle& privateNode)
{
   // fetch laser mapping params
   float fParam;
   int iParam;

   if (privateNode.getParam("scanPeriod", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid scanPeriod parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         setScanPeriod(fParam);
         ROS_INFO("Set scanPeriod: %g", fParam);
      }
   }

   if (privateNode.getParam("maxIterations", iParam))
   {
      if (iParam < 1)
      {
         ROS_ERROR("Invalid maxIterations parameter: %d (expected > 0)", iParam);
         return false;
      }
      else
      {
         setMaxIterations(iParam);
         ROS_INFO("Set maxIterations: %d", iParam);
      }
   }

   if (privateNode.getParam("deltaTAbort", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid deltaTAbort parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         setDeltaTAbort(fParam);
         ROS_INFO("Set deltaTAbort: %g", fParam);
      }
   }

   if (privateNode.getParam("deltaRAbort", fParam))
   {
      if (fParam <= 0)
      {
         ROS_ERROR("Invalid deltaRAbort parameter: %f (expected > 0)", fParam);
         return false;
      }
      else
      {
         setDeltaRAbort(fParam);
         ROS_INFO("Set deltaRAbort: %g", fParam);
      }
   }

   if (privateNode.getParam("cornerFilterSize", fParam))
   {
      if (fParam < 0.001)
      {
         ROS_ERROR("Invalid cornerFilterSize parameter: %f (expected >= 0.001)", fParam);
         return false;
      }
      else
      {
         downSizeFilterCorner().setLeafSize(fParam, fParam, fParam);
         ROS_INFO("Set corner down size filter leaf size: %g", fParam);
      }
   }

   if (privateNode.getParam("surfaceFilterSize", fParam))
   {
      if (fParam < 0.001)
      {
         ROS_ERROR("Invalid surfaceFilterSize parameter: %f (expected >= 0.001)", fParam);
         return false;
      }
      else
      {
         downSizeFilterSurf().setLeafSize(fParam, fParam, fParam);
         ROS_INFO("Set surface down size filter leaf size: %g", fParam);
      }
   }

   if (privateNode.getParam("mapFilterSize", fParam))
   {
      if (fParam < 0.001)
      {
         ROS_ERROR("Invalid mapFilterSize parameter: %f (expected >= 0.001)", fParam);
         return false;
      }
      else
      {
         downSizeFilterMap().setLeafSize(fParam, fParam, fParam);
         ROS_INFO("Set map down size filter leaf size: %g", fParam);
      }
   }

   // advertise laser mapping topics
   _pubLaserCloudSurround = node.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);
   _pubLaserCloudFullRes  = node.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 2);
   _pubOdomAftMapped      = node.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 5);

   // subscribe to laser odometry topics
   _subLaserCloudCornerLast = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_corner_last", 2, &LaserMapping::laserCloudCornerLastHandler, this);

   _subLaserCloudSurfLast = node.subscribe<sensor_msgs::PointCloud2>
      ("/laser_cloud_surf_last", 2, &LaserMapping::laserCloudSurfLastHandler, this);

   _subLaserOdometry = node.subscribe<nav_msgs::Odometry>
      ("/laser_odom_to_init", 5, &LaserMapping::laserOdometryHandler, this);

   _subLaserCloudFullRes = node.subscribe<sensor_msgs::PointCloud2>
      ("/velodyne_cloud_3", 2, &LaserMapping::laserCloudFullResHandler, this);

   // subscribe to IMU topic
   _subImu = node.subscribe<sensor_msgs::Imu>("/imu/data", 50, &LaserMapping::imuHandler, this);

   return true;
}
*/

/*
void LaserMapping::laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr& cornerPointsLastMsg)
{
   _timeLaserCloudCornerLast = cornerPointsLastMsg->header.stamp;
   laserCloudCornerLast().clear();
   pcl::fromROSMsg(*cornerPointsLastMsg, laserCloudCornerLast());
   _newLaserCloudCornerLast = true;
}
*/
 /*
void LaserMapping::laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr& surfacePointsLastMsg)
{
   _timeLaserCloudSurfLast = surfacePointsLastMsg->header.stamp;
   laserCloudSurfLast().clear();
   pcl::fromROSMsg(*surfacePointsLastMsg, laserCloudSurfLast());
   _newLaserCloudSurfLast = true;
}
*/
/*
void LaserMapping::laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudFullResMsg)
{
   _timeLaserCloudFullRes = laserCloudFullResMsg->header.stamp;
   pointcloudTime = _timeLaserCloudFullRes.toNSec();
   
   laserCloud().clear();
   pcl::fromROSMsg(*laserCloudFullResMsg, laserCloud());
   _newLaserCloudFullRes = true;
}
*/
/*
void LaserMapping::laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
   _timeLaserOdometry = laserOdometry->header.stamp;

   double roll, pitch, yaw;
   geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
   tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

   updateOdometry(-pitch, -yaw, roll,
                  laserOdometry->pose.pose.position.x,
                  laserOdometry->pose.pose.position.y,
                  laserOdometry->pose.pose.position.z);

   _newLaserOdometry = true;
}
*/
/*
void LaserMapping::imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
   double roll, pitch, yaw;
   tf::Quaternion orientation;
   tf::quaternionMsgToTF(imuIn->orientation, orientation);
   tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
   updateIMU({ fromROSTime(imuIn->header.stamp) , roll, pitch });
}
*/
/*
void LaserMapping::spin()
{
   ros::Rate rate(100);
   bool status = ros::ok();

   while (status)
   {
      ros::spinOnce();

      // try processing buffered data
      process();

      status = ros::ok();
      rate.sleep();
   }
}
*/
/*
void LaserMapping::reset()
{
   _newLaserCloudCornerLast = false;
   _newLaserCloudSurfLast = false;
   _newLaserCloudFullRes = false;
   _newLaserOdometry = false;
}
*/
/*
bool LaserMapping::hasNewData()
{
   return _newLaserCloudCornerLast && _newLaserCloudSurfLast &&
      _newLaserCloudFullRes && _newLaserOdometry &&
      fabs((_timeLaserCloudCornerLast - _timeLaserOdometry).toSec()) < 0.005 &&
      fabs((_timeLaserCloudSurfLast - _timeLaserOdometry).toSec()) < 0.005 &&
      fabs((_timeLaserCloudFullRes - _timeLaserOdometry).toSec()) < 0.005;
}
*/
void LaserMapping::process(const pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn,
    const pcl::PointCloud<pcl::PointXYZI>& cornerPointsSharp,
    const pcl::PointCloud<pcl::PointXYZI>& surPointsFlat,
    const long long& scanTime,
    const std::vector<NAVDATA>& nav,
    pcl::PointCloud<pcl::PointXYZI>::Ptr& laserCloudMap)
{
   BasicLaserMapping::process(laserCloudIn, cornerPointsSharp,  surPointsFlat, scanTime, nav, laserCloudMap);
}

} // end namespace loam
