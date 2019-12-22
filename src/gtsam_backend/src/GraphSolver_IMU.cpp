#include "GraphSolver.h"

bool GraphSolver::set_imu_preintegration(const gtsam::State& prior_state) {

  // Create GTSAM preintegration parameters for use with Foster's version
  boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> params;
  params = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(config->gravity(2));  // Z-up navigation frame: gravity points along negative Z-axis !!!
  
  params->setAccelerometerCovariance(gtsam::I_3x3 * config->sigma_a_sq);  // acc white noise in continuous
  params->setGyroscopeCovariance(gtsam::I_3x3 * config->sigma_g_sq);  // gyro white noise in continuous
  params->biasAccCovariance = config->sigma_wa_sq * gtsam::Matrix33::Identity(3,3);  // acc bias in continuous
  params->biasOmegaCovariance = config->sigma_wg_sq * gtsam::Matrix33::Identity(3,3);  // gyro bias in continuous
  params->setIntegrationCovariance(gtsam::I_3x3 * 0.1);  // error committed in integrating position from velocities
  params->biasAccOmegaInt = 1e-5*gtsam::Matrix66::Identity(6,6); // error in the bias used for preintegration
  
  // Actually create the GTSAM preintegration
  preint_gtsam = new gtsam::PreintegratedCombinedMeasurements(params, prior_state.b());
  return true;
}

/**
 * This function will create a discrete IMU factor using the GTSAM preintegrator class
 * This will integrate from the current state time up to the new update time
 */
gtsam::CombinedImuFactor GraphSolver::create_imu_factor(double updatetime, gtsam::Values& values_initial) {

    int imucompound = 0;

    // TODO: Clean this code, and use the mutex
    while(imu_times.size() > 1 && imu_times.at(1) <= updatetime) {
        double dt = imu_times.at(1) - imu_times.at(0);
        if (dt >= 0) {
            // Our IMU measurement
            Eigen::Vector3d meas_angvel;
            Eigen::Vector3d meas_linaccs;
            meas_angvel = imu_angvel.at(0);
            meas_linaccs = imu_linaccs.at(0);
            // Preintegrate this measurement!
            preint_gtsam->integrateMeasurement(meas_linaccs, meas_angvel, dt);
        }
        //std::cout << "state time = " << updatetime << " | imu0 = " << imu_times.at(0) << " | imu1 = " << imu_times.at(1) << " | dt = " << dt << std::endl;
        //cout << "imu dt = " << dt << " | am = " << imu_linaccs.at(0).transpose() << " | wm = " << imu_angvel.at(0).transpose() << endl;
        imu_angvel.erase(imu_angvel.begin());
        imu_linaccs.erase(imu_linaccs.begin());
        imu_times.erase(imu_times.begin());
        imucompound++;
    }

    // TODO: Clean this code, and use the mutex
    double dt_f = updatetime - imu_times.at(0);
    if (dt_f > 0) {
        // Our IMU measurement
        Eigen::Vector3d meas_angvel;
        Eigen::Vector3d meas_linaccs;
        meas_angvel = imu_angvel.at(0);
        meas_linaccs = imu_linaccs.at(0);
        // Preintegrate this measurement!
        preint_gtsam->integrateMeasurement(meas_linaccs, meas_angvel, dt_f);
        imu_times.at(0) = updatetime;
        imucompound++;
    }
 
    return gtsam::CombinedImuFactor(X(ct_state  ), V(ct_state),
                             X(ct_state+1), V(ct_state+1),  
                             B(ct_state  ), B(ct_state+1),
                             *preint_gtsam);
}


/**
 * This function will get the predicted state based on the IMU measurement
 */

gtsam::State GraphSolver::get_predicted_state(gtsam::Values& values_initial) {

  // Get the current state (t=k)
  gtsam::State stateK = gtsam::State(values_initial.at<gtsam::Pose3>(X(ct_state)),
                                     values_initial.at<gtsam::Vector3>(V(ct_state)),
                                     values_initial.at<gtsam::Bias>(B(ct_state)));
  
  // From this we should predict where we will be at the next time (t=K+1)
  gtsam::NavState stateK1 = preint_gtsam->predict(gtsam::NavState(stateK.pose(), stateK.v()), stateK.b());
  return gtsam::State(stateK1.pose(), stateK1.v(), stateK.b());
}

void GraphSolver::reset_imu_integration() {
  
  // Use the optimized bias to reset integration
  if (values_initial.exists(B(ct_state)))
    preint_gtsam->resetIntegrationAndSetBias(values_initial.at<gtsam::Bias>(B(ct_state)));
    //preint_gtsam_->resetIntegrationAndSetBias(Bias());
  
  return;
}
