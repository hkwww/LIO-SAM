#include <ros/ros.h>
#include "utility.h"

using namespace std;

struct PointXYZIRPYT {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;  // preferred way of adding a XYZ+padding
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRPYT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
        float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time,
                                                                 time))

typedef PointXYZIRPYT PointTypePose;

bool LoadKeyFramePoses(std::string path, std::vector<Eigen::Matrix4f>& poses) {
  pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D(
      new pcl::PointCloud<PointTypePose>);
  pcl::io::loadPCDFile(path, *cloudKeyPoses6D);

  for (size_t i = 0; i < cloudKeyPoses6D->points.size(); i++) {
    auto thisPoint = cloudKeyPoses6D->points[i];
    Eigen::Affine3f key_pose =
        pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z,
                               thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    poses.push_back(key_pose.matrix());
  }

  if (poses.empty()) {
    std::cout << "Load Key Frame Poses Wrong " << std::endl;
    return false;
  }

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "mapGeneration");
  ros::NodeHandle n;

  std::string lio_path = "/home/hkw/rosbag/rs_mti";
  std::string key_pose_path = lio_path + "/transformations.pcd";
  std::string key_frame_cloud_path = lio_path + "/temp_data";

  std::vector<Eigen::Matrix4f> kf_poses;
  LoadKeyFramePoses(key_pose_path, kf_poses);

  std::vector<pcl::PointCloud<PointType>> kf_clouds;

  for (size_t i = 0; i < kf_poses.size(); i++) {
  }
}
