#include <math.h>
#include <ros/ros.h>

#include "GraphSolver.h"


void GraphSolver::addmeasurement_imu(double timestamp, Eigen::Vector3d linacc, Eigen::Vector3d angvel, Eigen::Vector4d orientation) {
  // Request access to the imu measurements
  std::unique_lock<std::mutex> lock(imu_mutex);

  // Append this new measurement to the array
  imu_times.push_back(timestamp);
  imu_linaccs.push_back(linacc);
  imu_angvel.push_back(angvel);
  imu_orientation.push_back(orientation);

}

void GraphSolver::addmeasurement_uv(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv) {

  // Return if the node already exists in the graph
  if (ct_state_lookup.find(timestamp) != ct_state_lookup.end())
    return;

  // Return if we don't actually have any plane measurements
  if(leftids.empty())
      return;

  // Return if we don't actually have any IMU measurements
  if(imu_times.size() < 2)
      return;

  // We should try to initialize now
  // Or add the current a new IMU measurement and state!
  if(!systeminitalized) {

      initialize(timestamp);

      // Return if we have not initialized the system yet
      if(!systeminitalized)
          return;

  } else {

      // Forster2 discrete preintegration
      gtsam::CombinedImuFactor imuFactor = create_imu_factor(timestamp, values_initial);
      graph_new->add(imuFactor);
      graph->add(imuFactor);

      // Original models
      gtsam::State newstate = get_predicted_state(values_initial);

      // Move node count forward in time
      ct_state++;

      // Append to our node vectors
      values_new.insert(    X(ct_state), newstate.pose());
      values_new.insert(    V(ct_state), newstate.v());
      values_new.insert(    B(ct_state), newstate.b());
      values_initial.insert(X(ct_state), newstate.pose());
      values_initial.insert(V(ct_state), newstate.v());
      values_initial.insert(B(ct_state), newstate.b());

      // Add ct state to map
      ct_state_lookup[timestamp] = ct_state;
      timestamp_lookup[ct_state] = timestamp;
  }

  // Assert our vectors are equal (note will need to remove top one eventually)
  assert(leftids.size() == leftuv.size());

  // Request access
  std::unique_lock<std::mutex> features_lock(features_mutex);

  // If we are using inverse depth, then lets call on it
  process_feat_smart(timestamp, leftids, leftuv);
}

void GraphSolver::optimize() {

  // Return if not initialized
  if(!systeminitalized && ct_state < 2)
      return;

  // Perform smoothing update
  try {
    gtsam::ISAM2Result result = isam2->update(*graph_new, values_new);
    values_initial = isam2->calculateEstimate();
  } catch(gtsam::IndeterminantLinearSystemException &e) {
      ROS_ERROR("FORSTER2 gtsam indeterminate linear system exception!");
      std::cerr << e.what() << std::endl;
      exit(EXIT_FAILURE);
  }

  // Remove the used up nodes
  values_new.clear();

  // Remove the used up factors
  graph_new->resize(0);

  // reset imu preintegration
  reset_imu_integration();
}

void GraphSolver::initialize(double timestamp) {
    
  // If we have already initialized, then just return
  if (systeminitalized)
    return;
    
  // Wait for enough IMU readings if we initialize from rest
  if (imu_times.size() < config->imuWait)
    return;
  
  // Reading from launch file
  gtsam::Vector4 q_GtoI = config->prior_qGtoI;
  gtsam::Vector3 p_IinG = config->prior_pIinG;
  gtsam::Vector3 v_IinG = config->prior_vIinG;
  gtsam::Vector3 ba = config->prior_ba;
  gtsam::Vector3 bg = config->prior_bg;
    
  // Create prior factor and add it to the graph
  gtsam::State prior_state = gtsam::State(gtsam::Pose3(gtsam::Quaternion(q_GtoI(3), q_GtoI(0), q_GtoI(1), q_GtoI(2)), p_IinG),
                                          v_IinG, gtsam::Bias(ba, bg)); // gtsam::Quaternion(w, x, y, z)
  auto pose_noise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << gtsam::Vector3::Constant(config->sigma_prior_rotation),
                                                         gtsam::Vector3::Constant(config->sigma_prior_translation)).finished());
  auto v_noise = gtsam::noiseModel::Isotropic::Sigma(3, config->sigma_velocity);
  auto b_noise = gtsam::noiseModel::Isotropic::Sigma(6, config->sigma_bias);

  graph_new->add(gtsam::PriorFactor<gtsam::Pose3>(  X(ct_state), prior_state.pose(), pose_noise));
  graph_new->add(gtsam::PriorFactor<gtsam::Vector3>(V(ct_state), prior_state.v(),    v_noise));
  graph_new->add(gtsam::PriorFactor<gtsam::Bias>(   B(ct_state), prior_state.b(),    b_noise));
  graph->add(gtsam::PriorFactor<gtsam::Pose3>(      X(ct_state), prior_state.pose(), pose_noise));
  graph->add(gtsam::PriorFactor<gtsam::Vector3>(    V(ct_state), prior_state.v(),    v_noise));
  graph->add(gtsam::PriorFactor<gtsam::Bias>(       B(ct_state), prior_state.b(),    b_noise));

  // Add initial state to the graph
  values_new.insert(    X(ct_state), prior_state.pose());
  values_new.insert(    V(ct_state), prior_state.v());
  values_new.insert(    B(ct_state), prior_state.b());
  values_initial.insert(X(ct_state), prior_state.pose());
  values_initial.insert(V(ct_state), prior_state.v());
  values_initial.insert(B(ct_state), prior_state.b());

  // Add ct state to map
  ct_state_lookup[timestamp] = ct_state;
  timestamp_lookup[ct_state] = timestamp;

  // Clear all old imu messages (keep the last two)
  imu_times.erase(imu_times.begin(), imu_times.end() - 1);
  imu_linaccs.erase(imu_linaccs.begin(), imu_linaccs.end() - 1);
  imu_angvel.erase(imu_angvel.begin(), imu_angvel.end() - 1);
  imu_orientation.erase(imu_orientation.begin(), imu_orientation.end() - 1);

  if (set_imu_preintegration(prior_state))
    systeminitalized = true;

  // Debug info
  ROS_INFO("\033[0;32m[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\033[0m",q_GtoI(0),q_GtoI(1),q_GtoI(2),q_GtoI(3));
  ROS_INFO("\033[0;32m[INIT]: velocity = %.4f, %.4f, %.4f\033[0m",v_IinG(0),v_IinG(1),v_IinG(2));
  ROS_INFO("\033[0;32m[INIT]: position = %.4f, %.4f, %.4f\033[0m",p_IinG(0),p_IinG(1),p_IinG(2));
  ROS_INFO("\033[0;32m[INIT]: bias accel = %.4f, %.4f, %.4f\033[0m",ba(0),ba(1),ba(2));
  ROS_INFO("\033[0;32m[INIT]: bias gyro = %.4f, %.4f, %.4f\033[0m",bg(0),bg(1),bg(2));
}
