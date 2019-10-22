#pragma once

#include <Eigen/Dense>

class Config {
public:
  /// Default constructor
  Config(){}

  std::string fixedId;

  /// Assumed "true" global gravity
  Eigen::Matrix<double, 3, 1> gravity;

  /// Amount of IMU and FEAT we should wait to initialize to
  int imuWait;
  int featWait;

  /// Relative transform between CAM0 and IMU
  Eigen::Matrix<double,3,1> p_IinC0;
  Eigen::Matrix<double,3,3> R_C0toI;

  /// Priors
  Eigen::Vector4d prior_qGtoI;
  Eigen::Vector3d prior_pIinG;
  Eigen::Vector3d prior_vIinG;
  Eigen::Vector3d prior_ba;
  Eigen::Vector3d prior_bg;
  
  /// Noise values for the image
  double sigma_camera; ///< Noise value for CAMERA points
  double sigma_camera_sq; ///< Noise value for CAMERA points squared

  // Noise values for the IMU
  double sigma_a; ///< Acceleration white noise
  double sigma_a_sq; ///< Acceleration white noise squared
  double sigma_g; ///< Gyro white noise
  double sigma_g_sq; ///< Gyro white noise squared
  double sigma_wa; ///< Acceleration bias walk
  double sigma_wa_sq; ///< Acceleration bias walk squared
  double sigma_wg; ///< Gyro bias walk
  double sigma_wg_sq; ///< Gyro bias walk squared

  // Noise values for Initialization
  double sigma_prior_rotation; // 0.1 rad on roll, pitch, yaw
  double sigma_prior_translation; // 30cm std on x, y, z
  double sigma_velocity; // 0.1 m/s
  double sigma_bias; // 0.1 m/s
  double sigma_pose_rotation; // 0.1 rad on roll, pitch, yaw
  double sigma_pose_translation; // 30cm std on x, y, z
};
