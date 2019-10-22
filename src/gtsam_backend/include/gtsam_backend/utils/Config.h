/**
 * MIT License
 * Copyright (c) 2018 Kevin Eckenhoff
 * Copyright (c) 2018 Patrick Geneva
 * Copyright (c) 2018 Guoquan Huang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


#ifndef CONFIG_H
#define CONFIG_H

#include <Eigen/Dense>

class Config {

public:

    /// Default constructor
    Config(){}

    /// If we should use serial simulation
    bool useSerialSIM;

    /// Path to simulation dataset
    std::string simPath;

    std::string fixedId;

    /// Min number of poses to initalize a feature
    int minPoseFeatureInit;

    /// Amount of poses that we should stop updating the feature after
    int uvWindowSize;

    bool useENU;

    /// If we should use the groundtruth to initialize
    bool keepRestInit;
    bool useOrientationInit;
    bool useVINSInit;
    bool useOrientationInitValues;
    bool useVINSInitValues;

    bool useSmartFactor;
    bool useVisualLoopClosure;
    int secondPriorState;
    
    // Rates
    int imuRate;
    int camRate;
    int skipkf;

    /// Amount of IMU we should wait to initialize to
    int imuWait;

    /// Relative transform between CAM0 and IMU
    Eigen::Matrix<double,3,1> p_IinC0;
    Eigen::Matrix<double,3,3> R_C0toI;

    /// Priors
    Eigen::Vector4d prior_qGtoI;
    Eigen::Vector3d prior_pIinG;
    Eigen::Vector3d prior_vIinG;
    Eigen::Vector3d prior_ba;
    Eigen::Vector3d prior_bg;

    /// Relative transform between CAM1 and IMU
    Eigen::Matrix<double,3,1> p_IinC1;
    Eigen::Matrix<double,3,3> R_C1toI;

    /// Assumed "true" global gravity
    Eigen::Matrix<double,3,1> gravity;

    /// Timeshift cam0 to imu0: [s] (t_imu = t_cam + shift)
    double timeshift = 0;

    /// Noise values for the image
    double sigma_camera; ///< Noise value for CAMERA points
    double sigma_camera_sq; ///< Noise value for CAMERA points squared

    // Noise values for the IMU
    double sigma_g; ///< Gyro white noise
    double sigma_g_sq; ///< Gyro white noise squared
    double sigma_wg; ///< Gyro bias walk
    double sigma_wg_sq; ///< Gyro bias walk squared
    double sigma_a; ///< Acceleration white noise
    double sigma_a_sq; ///< Acceleration white noise squared
    double sigma_wa; ///< Acceleration bias walk
    double sigma_wa_sq; ///< Acceleration bias walk squared

    // Noise values for Initialization
    double sigma_prior_rotation; // 0.1 rad on roll, pitch, yaw
    double sigma_prior_translation; // 30cm std on x, y, z
    double sigma_velocity; // 0.1 m/s
    double sigma_bias; // 0.1 m/s
    double sigma_pose_rotation; // 0.1 rad on roll, pitch, yaw
    double sigma_pose_translation; // 30cm std on x, y, z
};




#endif /* CONFIG_H */
