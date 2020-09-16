

#ifndef LOAM_SCANREGISTRATION_H
#define LOAM_SCANREGISTRATION_H


#include "common.h"

#include <stdint.h>

//#include <ros/node_handle.h>
//#include <sensor_msgs/Imu.h>

#include "BasicScanRegistration.h"


namespace loam
{
  /** \brief Base class for LOAM scan registration implementations.
   *
   * As there exist various sensor devices, producing differently formatted point clouds,
   * specific implementations are needed for each group of sensor devices to achieve an accurate registration.
   * This class provides common configurations, buffering and processing logic.
   */
  class ScanRegistration : protected BasicScanRegistration
  {
  public:

    /** \brief Setup component.
     *
     * @param node the ROS node handle
     * @param privateNode the private ROS node handle
     */
    virtual bool setupROS(/*ros::NodeHandle& node, ros::NodeHandle& privateNode,*/ RegistrationParams& config_out);

    /** \brief Handler method for IMU messages.
     *
     * @param imuIn the new IMU message
     */
    virtual void handleIMUMessage(const sensor_msgs::Imu::ConstPtr& imuIn);

  protected:
    /** \brief Publish the current result via the respective topics. */
    void publishResult();

  private:

    /** \brief Parse node parameter.
    *
    * @param nh the ROS node handle
    * @return true, if all specified parameters are valid, false if at least one specified parameter is invalid
    */
    bool parseParams(/*const ros::NodeHandle& nh,*/ RegistrationParams& config_out);

  private:
    //ros::Subscriber _subImu;                    ///< IMU message subscriber
    //ros::Publisher _pubLaserCloud;              ///< full resolution cloud message publisher
    //ros::Publisher _pubCornerPointsSharp;       ///< sharp corner cloud message publisher
    //ros::Publisher _pubCornerPointsLessSharp;   ///< less sharp corner cloud message publisher
    //ros::Publisher _pubSurfPointsFlat;          ///< flat surface cloud message publisher
    //ros::Publisher _pubSurfPointsLessFlat;      ///< less flat surface cloud message publisher
    //ros::Publisher _pubImuTrans;                ///< IMU transformation message publisher
  };

} // end namespace loam


#endif //LOAM_SCANREGISTRATION_H
