#pragma once

#include <geometry_msgs/PoseWithCovariance.h>
#include <gtsam/geometry/Pose3.h>

void ToPose(const gtsam::Pose3& i_pose, geometry_msgs::Pose& pose) {
  pose.orientation.x = i_pose.rotation().toQuaternion().x();
  pose.orientation.y = i_pose.rotation().toQuaternion().y();
  pose.orientation.z = i_pose.rotation().toQuaternion().z();
  pose.orientation.w = i_pose.rotation().toQuaternion().w();
  pose.position.x = i_pose.translation().x();
  pose.position.y = i_pose.translation().y();
  pose.position.z = i_pose.translation().z();
}

void ToPoseWithCovariance(const gtsam::Pose3& i_pose,
                          const Eigen::Matrix<double, 6, 6>& covariance,
                          geometry_msgs::PoseWithCovariance& pose) {
  ToPose(i_pose, pose.pose);
  
  // Finally set the covariance in the message
  pose.covariance[0] = covariance(0,0); // 0
  pose.covariance[1] = covariance(0,1);
  pose.covariance[2] = covariance(0,2);
  pose.covariance[3] = covariance(0,3);
  pose.covariance[4] = covariance(0,4);
  pose.covariance[5] = covariance(0,5);
  pose.covariance[6] = covariance(1,0); // 1
  pose.covariance[7] = covariance(1,1);
  pose.covariance[8] = covariance(1,2);
  pose.covariance[9] = covariance(1,3);
  pose.covariance[10] = covariance(1,4);
  pose.covariance[11] = covariance(1,5);
  pose.covariance[12] = covariance(2,0); // 2
  pose.covariance[13] = covariance(2,1);
  pose.covariance[14] = covariance(2,2);
  pose.covariance[15] = covariance(2,3);
  pose.covariance[16] = covariance(2,4);
  pose.covariance[17] = covariance(2,5);
  pose.covariance[18] = covariance(3,0); // 3
  pose.covariance[19] = covariance(3,1);
  pose.covariance[20] = covariance(3,2);
  pose.covariance[21] = covariance(3,3);
  pose.covariance[22] = covariance(3,4);
  pose.covariance[23] = covariance(3,5);
  pose.covariance[24] = covariance(4,0); // 4
  pose.covariance[25] = covariance(4,1);
  pose.covariance[26] = covariance(4,2);
  pose.covariance[27] = covariance(4,3);
  pose.covariance[28] = covariance(4,4);
  pose.covariance[29] = covariance(4,5);
  pose.covariance[30] = covariance(5,0); // 5
  pose.covariance[31] = covariance(5,1);
  pose.covariance[32] = covariance(5,2);
  pose.covariance[33] = covariance(5,3);
  pose.covariance[34] = covariance(5,4);
  pose.covariance[35] = covariance(5,5);
}
