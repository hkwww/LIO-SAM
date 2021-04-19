#ifndef OCCUPANCY_GRID_MAP_MARKER_H
#define OCCUPANCY_GRID_MAP_MARKER_H

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include "utility.h"

typedef pcl::PointXYZ VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;

template <class T>
struct point {
  inline point() : x(0), y(0) {}
  inline point(T _x, T _y) : x(_x), y(_y) {}
  T x, y;
};

typedef point<int> IntPoint;
typedef point<float> FloatPoint;

typedef struct {
  int num_points;
  IntPoint* points;
} IntLine;

class OccupancyGridMapMarker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<OccupancyGridMapMarker> Ptr;

  OccupancyGridMapMarker(ros::NodeHandle nh);

  void registerScan(VPointCloud::Ptr cloud, FloatPoint OrientedPoint);

  void gridMapInit(double MapMinX, double MapMinY, double MapMaxX,
                   double MapMaxY, double MapResolution);

  void getLine(IntPoint start, IntPoint end, IntLine* line);

  void gridLineCore(IntPoint start, IntPoint end, IntLine* line);

  inline unsigned int mapIdx(IntPoint p) {
    return gridMap.info.width * p.y + p.x;
  }

  //米制转换到栅格地图中
  inline IntPoint metricToCell(FloatPoint p) {
    IntPoint result;
    result.x = (p.x - gridMap.info.origin.position.x) / gridMap.info.resolution;
    result.y = (p.y - gridMap.info.origin.position.y) / gridMap.info.resolution;
    return result;
  }

 private:
  ros::NodeHandle n;

  nav_msgs::OccupancyGrid gridMap;

  ros::Publisher pubGridMap;

  double mapMaxX;

  double mapMaxY;

  double mapMinX;

  double mapMinY;

  double mapResolution;

  double loFree;  //若栅格为free，栅格的概率增加lo_free

  double loOcc;  //同理
};

#endif
