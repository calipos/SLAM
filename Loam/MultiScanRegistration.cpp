

#include "MultiScanRegistration.h"
#include "math_utils.h"

//#include <pcl_conversions/pcl_conversions.h>


namespace loam {

MultiScanMapper::MultiScanMapper(const float& lowerBound,
                                 const float& upperBound,
                                 const uint16_t& nScanRings)
{
    _lowerBound = lowerBound;
    _upperBound = upperBound;
    _nScanRings = nScanRings;
    _factor = (nScanRings - 1) / (upperBound - lowerBound);
}

void MultiScanMapper::set(const float &lowerBound,
                          const float &upperBound,
                          const uint16_t &nScanRings)
{
  _lowerBound = lowerBound;
  _upperBound = upperBound;
  _nScanRings = nScanRings;
  _factor = (nScanRings - 1) / (upperBound - lowerBound);
}



int MultiScanMapper::getRingForAngle(const float& angle) {
  return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}






MultiScanRegistration::MultiScanRegistration(const MultiScanMapper& scanMapper)
    : _scanMapper(scanMapper)
{};



bool MultiScanRegistration::setup(RegistrationParams&config_in)
{
    CHECK(true == setupROS(config_in));
  configure(config);
  return true;
}

bool MultiScanRegistration::setupROS(RegistrationParams& config_out)
{
  if (!ScanRegistration::setupROS( config_out))
    return false;

  // fetch scan mapping params
  std::string lidarName= config_out.lidarName;

  if (lidarName!="") {
    if (lidarName == "VLP-16") {
      _scanMapper = MultiScanMapper::Velodyne_VLP_16();
    } else if (lidarName == "HDL-32") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_32();
    } else if (lidarName == "HDL-64E") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_64E();
    } else {
        LOG(ERROR) << "Invalid lidar parameter: %s (only \"VLP-16\", \"HDL-32\" and \"HDL-64E\" are supported)", lidarName.c_str();
      return false;
    }

    LOG(INFO) << "Set  %s  scan mapper.", lidarName.c_str();
    if (1/*!privateNode.hasParam("scanPeriod")*/) {
      config_out.scanPeriod = 0.1;
      LOG(INFO)<<"Set scanPeriod: %f", config_out.scanPeriod;
    }
  } else {
    float vAngleMin, vAngleMax;
    int nScanRings;

    if (privateNode.getParam("minVerticalAngle", vAngleMin) &&
        privateNode.getParam("maxVerticalAngle", vAngleMax) &&
        privateNode.getParam("nScanRings", nScanRings)) {
      if (vAngleMin >= vAngleMax) {
          LOG(ERROR) << "Invalid vertical range (min >= max)";
        return false;
      } else if (nScanRings < 2) {
          LOG(ERROR) << "Invalid number of scan rings (n < 2)";
        return false;
      }

      _scanMapper.set(vAngleMin, vAngleMax, nScanRings);
      LOG(INFO) << "Set linear scan mapper from %g to %g degrees with %d scan rings.", vAngleMin, vAngleMax, nScanRings;
    }
  }

  // subscribe to input cloud topic
  //_subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>
  //    ("/multi_scan_points", 2, &MultiScanRegistration::handleCloudMessage, this);

  return true;
}



void MultiScanRegistration::handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  //if (_systemDelay > 0) 
  //{
  //  --_systemDelay;
  //  return;
  //}

  //pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
  //pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

  //process(laserCloudIn, fromROSTime(laserCloudMsg->header.stamp));
}



void MultiScanRegistration::process(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn, const Time& scanTime)
{
  size_t cloudSize = laserCloudIn.size();

  // determine scan start and end orientations
  float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
  float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y,
                             laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  bool halfPassed = false;
  pcl::PointXYZI point;
  _laserCloudScans.resize(_scanMapper.getNumberOfScanRings());
  // clear all scanline points
  std::for_each(_laserCloudScans.begin(), _laserCloudScans.end(), [](auto&&v) {v.clear(); }); 

  // extract valid points from input cloud
  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn[i].y;
    point.y = laserCloudIn[i].z;
    point.z = laserCloudIn[i].x;

    // skip NaN and INF valued points
    if (!std::isfinite(point.x) ||
        !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }

    // skip zero valued points
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    // calculate vertical point angle and scan ID
    float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
    int scanID = _scanMapper.getRingForAngle(angle);
    if (scanID >= _scanMapper.getNumberOfScanRings() || scanID < 0 ){
      continue;
    }

    // calculate horizontal point angle
    float ori = -std::atan2(point.x, point.z);
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;

      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    // calculate relative scan time based on point orientation
    float relTime = config().scanPeriod * (ori - startOri) / (endOri - startOri);
    point.intensity = scanID + relTime;

    projectPointToStartOfSweep(point, relTime);

    _laserCloudScans[scanID].push_back(point);
  }

  processScanlines(scanTime, _laserCloudScans);
  publishResult();
}

} // end namespace loam
