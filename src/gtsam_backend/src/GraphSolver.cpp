/**i
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

#include <math.h>
#include <iomanip>
#include "GraphSolver.h"

using namespace std;
using namespace gtsam;



/**
 * This will get two pose measurements and will initialize the system to the true orientation and pose
 * The velocity is calculated through the difference between the two poses.
 */
void GraphSolver::addtrue_pose(double timestamp, Eigen::Vector4d q_GtoI, Eigen::Vector3d p_IinG) {

    // Request access to the imu measurements
    std::unique_lock<std::mutex> lock(truth_mutex);

    // Append to our pose vector
    true_times.push_back(timestamp);
    true_qGtoI.push_back(q_GtoI);
    true_pIinG.push_back(p_IinG);

}

void GraphSolver::addtrue_state(Eigen::Vector3d v_IinG, Eigen::Vector3d ba, Eigen::Vector3d bg) {
  // Request access to the true velocity
  std::unique_lock<std::mutex> lock(truth_mutex);

  // Append to our velocity vector
  true_vIinG.push_back(v_IinG);
  true_ba.push_back(ba);
  true_bg.push_back(bg);
}


/**
 * This function handles new IMU measurements.
 * We just append this to our IMU vectors which will be used to create a preintegrated measurement later
 */
void GraphSolver::addmeasurement_imu(double timestamp, Eigen::Vector3d linacc, Eigen::Vector3d angvel, Eigen::Vector4d orientation) {

    {
        // Request access to the imu measurements
        std::unique_lock<std::mutex> lock(imu_mutex);

        // Append this new measurement to the array
        imu_times.push_back(timestamp);
        imu_linaccs.push_back(linacc);
        imu_angvel.push_back(angvel);
        imu_orientation.push_back(orientation);
    }

}


/**
 * This function handles new ICP triggers.
 * We need to create a new node and insert into the graph.
 */
void GraphSolver::addtrigger_icp(double timestamp) {

    // Return if the node already exists in the graph
    if (ct_state_lookup.find(timestamp) != ct_state_lookup.end())
      return;

    // Return if we don't actually have any IMU measurements
    if(imu_times.size() < 2)
        return;

    // We should try to initialize now
    // Or add the current a new IMU measurement and state!
    if(!systeminitalized) {

        trytoinitialize(timestamp);

        // Return if we have not initialized the system yet
        if(!systeminitalized)
            return;

    } else {

        //==========================================================================
        // PREINTEGRATION IMU FACTORS
        //==========================================================================

        // Forster2 discrete preintegration
        CombinedImuFactor imuFactorFORSTER2 = create_imu_factor(timestamp, values_initialFORSTER2);
        graph_newFORSTER2->add(imuFactorFORSTER2);
        graphFORSTER2->add(imuFactorFORSTER2);

        //==========================================================================
        // NEW PREDICTED STATE
        //==========================================================================

        // Original models
        State newstateFORSTER2 = get_predicted_state(values_initialFORSTER2);

        // Move node count forward in time
        ct_state++;

        // Append to our node vectors
        values_newFORSTER2.insert(Y(ct_state), newstateFORSTER2.pose());
        values_newFORSTER2.insert(V(ct_state), newstateFORSTER2.v());
        values_newFORSTER2.insert(B(ct_state), newstateFORSTER2.b());

        values_initialFORSTER2.insert(Y(ct_state), newstateFORSTER2.pose());
        values_initialFORSTER2.insert(V(ct_state), newstateFORSTER2.v());
        values_initialFORSTER2.insert(B(ct_state), newstateFORSTER2.b());

        // Add ct state to map
        ct_state_lookup[timestamp] = ct_state;
        timestamp_lookup[ct_state] = timestamp;
    }
    
}


/**
 * This function handles new ICP measurements.
 * We just insert a between factor into the graph.
 */
void GraphSolver::addmeasurement_icp(double timestamp1, double timestamp2, Eigen::Vector4d q_1to2, Eigen::Vector3d p_2in1) {
      
  //std::cout << timestamp1 << " " << timestamp2 << std::endl;
      /*for (auto it = ct_state_lookup.begin(); it != ct_state_lookup.end(); it++) {
        std::cout << it->first << " " << it->second << std::endl;
      }*/
      
  if (ct_state_lookup.find(timestamp1) != ct_state_lookup.end() && ct_state_lookup.find(timestamp2) != ct_state_lookup.end()) {
    gtsam::Pose3 pose_1to2 = gtsam::Pose3(gtsam::Quaternion(q_1to2(3), q_1to2(0), q_1to2(1), q_1to2(2)), p_2in1);
    auto pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(config->sigma_pose_rotation),
                                                                 Vector3::Constant(config->sigma_pose_translation)).finished());
    graph_newFORSTER2->add(BetweenFactor<Pose3>(Y( ct_state_lookup[timestamp1]), 
                                                Y( ct_state_lookup[timestamp2]),
                                                pose_1to2, pose_noise));
    ROS_INFO("Pose factor added between Node %d and Node %d.", ct_state_lookup[timestamp1], ct_state_lookup[timestamp2]);
  }
}


/**
 * This function takes in the calculated plane variables and their covariances.
 * The planes are seen in the LIDAR frame at the given timestep.i
 * We first create a preintegrated measurement up to this time, which we can then insert into the graph.
 */
void GraphSolver::addmeasurement_uv(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv,
                                    std::vector<uint> rightids, std::vector<Eigen::Vector2d> rightuv) {

  //std::cout << std::setprecision (20) << timestamp << std::endl; 
  
  // Return if the node already exists in the graph
    if (ct_state_lookup.find(timestamp) != ct_state_lookup.end())
      return;

    // Return if we don't actually have any plane measurements
    if(leftids.empty() || rightids.empty())
        return;

    // Return if we don't actually have any IMU measurements
    if(imu_times.size() < 2)
        return;

    // We should try to initialize now
    // Or add the current a new IMU measurement and state!
    if(!systeminitalized) {

        trytoinitialize(timestamp);

        // Return if we have not initialized the system yet
        if(!systeminitalized)
            return;

    } else {

        //==========================================================================
        // PREINTEGRATION IMU FACTORS
        //==========================================================================

        // Forster2 discrete preintegration
        CombinedImuFactor imuFactorFORSTER2 = create_imu_factor(timestamp, values_initialFORSTER2);
        graph_newFORSTER2->add(imuFactorFORSTER2);
        graphFORSTER2->add(imuFactorFORSTER2);

        //==========================================================================
        // NEW PREDICTED STATE
        //==========================================================================

        // Original models
        State newstateFORSTER2 = get_predicted_state(values_initialFORSTER2);

        // Move node count forward in time
        ct_state++;

        // Append to our node vectors
     
        // Note: current orientation is assuming using NED coordinate
        if (!config->useENU && config->useOrientationInitValues) {
          
          std::unique_lock<std::mutex> imu_lock(imu_mutex); 
          int indexshift = config->timeshift * config->imuRate;
          Vector4 q_GtoI = imu_orientation.at(imu_orientation.size()-1+indexshift);
          
          gtsam::Pose3 newpose = gtsam::Pose3(gtsam::Quaternion(q_GtoI(3), q_GtoI(0), q_GtoI(1), q_GtoI(2)), newstateFORSTER2.p());
          values_newFORSTER2.insert(Y(ct_state), newpose);
          values_newFORSTER2.insert(V(ct_state), newstateFORSTER2.v());
          values_newFORSTER2.insert(B(ct_state), newstateFORSTER2.b());
          values_initialFORSTER2.insert(Y(ct_state), newpose);
          values_initialFORSTER2.insert(V(ct_state), newstateFORSTER2.v());
          values_initialFORSTER2.insert(B(ct_state), newstateFORSTER2.b());
        } 
        else if (config->useENU && config->useVINSInitValues && !true_times.empty()) {
          std::unique_lock<std::mutex> lock(truth_mutex); 
          Vector4 q_GtoI = true_qGtoI.at(true_qGtoI.size()-1);
          Vector3 p_IinG = true_pIinG.at(true_pIinG.size()-1);
          Vector3 v_IinG = true_vIinG.at(true_vIinG.size()-1);
          Vector3 ba = true_ba.at(true_ba.size()-1);
          Vector3 bg = true_bg.at(true_bg.size()-1);
          
          gtsam::Pose3 newpose = gtsam::Pose3(gtsam::Quaternion(q_GtoI(3), q_GtoI(0), q_GtoI(1), q_GtoI(2)), p_IinG);
          values_newFORSTER2.insert(    Y(ct_state), newpose);
          values_newFORSTER2.insert(    V(ct_state), v_IinG);
          values_newFORSTER2.insert(    B(ct_state), Bias(ba, bg));
          values_initialFORSTER2.insert(Y(ct_state), newpose);
          values_initialFORSTER2.insert(V(ct_state), v_IinG);
          values_initialFORSTER2.insert(B(ct_state), Bias(ba, bg));
        } 
        else {
          values_newFORSTER2.insert(Y(ct_state), newstateFORSTER2.pose());
          values_newFORSTER2.insert(V(ct_state), newstateFORSTER2.v());
          values_newFORSTER2.insert(B(ct_state), newstateFORSTER2.b());
          values_initialFORSTER2.insert(Y(ct_state), newstateFORSTER2.pose());
          values_initialFORSTER2.insert(V(ct_state), newstateFORSTER2.v());
          values_initialFORSTER2.insert(B(ct_state), newstateFORSTER2.b());
        }

        newTimestampsFORSTER[Y(ct_state)] = timestamp;

        // Add ct state to map
        ct_state_lookup[timestamp] = ct_state;
        timestamp_lookup[ct_state] = timestamp;

    
        // Add prior on second pose the recover SCALE
        if (ct_state == config->secondPriorState) {
          auto pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(config->sigma_prior_rotation),
                                                                       Vector3::Constant(config->sigma_prior_translation)).finished());
          if(!config->useENU && config->useOrientationInitValues) {
            std::unique_lock<std::mutex> imu_lock(imu_mutex);
            int indexshift = config->timeshift * config->imuRate;
            Vector4 ori = imu_orientation.at(imu_orientation.size()-1+indexshift);
            gtsam::Pose3 newpose = gtsam::Pose3(gtsam::Quaternion(ori(3), ori(0), ori(1), ori(2)), newstateFORSTER2.p());
            
            // Create gtsam prior factor and add it to FORSTER2 graph
            graph_newFORSTER2->add(PriorFactor<Pose3>(Y(ct_state), newpose, pose_noise));
            graphFORSTER2->add(PriorFactor<Pose3>(    Y(ct_state), newpose, pose_noise));
          } else {
            graph_newFORSTER2->add(PriorFactor<Pose3>(Y(ct_state), newstateFORSTER2.pose(), pose_noise));
            graphFORSTER2->add(PriorFactor<Pose3>(    Y(ct_state), newstateFORSTER2.pose(), pose_noise));
          }
          graph_newFORSTER2->print();
        }
    }


    // Assert our vectors are equal (note will need to remove top one eventually)
    assert(leftids.size() == leftuv.size());
    assert(rightids.size() == rightuv.size());

    // Debug printing
    //cout << "[FEAT]: " << leftids.size() << " left features | " << rightids.size() << " right features" << endl;

    // Request access
    std::unique_lock<std::mutex> features_lock(features_mutex);

    // If we are using inverse depth, then lets call on it
    if(config->useSmartFactor)
      process_feat_smart(timestamp, leftids, leftuv, rightids, rightuv);
    else
      process_feat_norm(timestamp, leftids, leftuv, rightids, rightuv);
}


/**
 * We add matched features in matched frames into graph.
 */
void GraphSolver::add_visual_loop_closure(double curr_timestamp, double prev_timestamp,
                                          std::vector<uint> curr_ids, std::vector<Eigen::Vector2d> curr_uv,
                                          std::vector<uint> prev_ids, std::vector<Eigen::Vector2d> prev_uv) {

    for (auto it = ct_state_lookup.begin(); it != ct_state_lookup.end(); it++) {
        std::cout << it->first << " " << it->second << std::endl;
    }

    Eigen::Vector4d q_1to2;
    q_1to2 << 0, 0, 0, 1;
    Eigen::Vector3d p_2in1;
    p_2in1 << 0, 0, 0;
    this->addmeasurement_icp(prev_timestamp, curr_timestamp, q_1to2, p_2in1);
 
    // Debug printing
    cout << "[LOOP]: " << curr_ids.size() << " current features | " << prev_ids.size() << " previous features" << endl;

    // Return if the node does not exist in the graph
    if (ct_state_lookup.find(curr_timestamp) == ct_state_lookup.end() || ct_state_lookup.find(prev_timestamp) == ct_state_lookup.end())
      return;

    // Assert our vectors are equal (note will need to remove top one eventually)
    assert(curr_ids.size() == curr_uv.size());
    assert(prev_ids.size() == prev_uv.size());

    // Request access
    std::unique_lock<std::mutex> features_lock(features_mutex);

    if(config->useVisualLoopClosure)
        process_feat_loop(curr_timestamp, prev_timestamp, curr_ids, curr_uv, prev_ids, prev_uv);
}


void GraphSolver::optimizeBatch() {

    // Return if not initialized
    if(!systeminitalized && ct_state < 2)
        return;

    // Start our timer
    boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());


    // Perform smoothing update
    try {
        smootherBatch->update(*graph_newFORSTER2, values_newFORSTER2, newTimestampsFORSTER);
        values_initialFORSTER2 = smootherBatch->calculateEstimate();
    } catch(gtsam::IndeterminantLinearSystemException &e) {
        ROS_ERROR("FORSTER gtsam indeterminate linear system exception!");
        cerr << e.what() << endl;
        exit(EXIT_FAILURE);
    }

    // Remove the used up nodes
    values_newFORSTER2.clear();

    // Clear used timestamps
    newTimestampsFORSTER.clear();

    // Remove the used up factors
    graph_newFORSTER2->resize(0);

    // Debug print time
    boost::posix_time::ptime t2(boost::posix_time::microsec_clock::local_time());

}



void GraphSolver::optimizeLM() {

    // Return if not initialized
    if(!systeminitalized)
        return;

    // Start our timer
    boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());
    //graphFORSTER2->print();
    //values_initialFORSTER2.print();
    // Perform smoothing update
    try {
      cout << "initial error = " << graphFORSTER2->error(values_initialFORSTER2) << endl;
      LevenbergMarquardtOptimizer optimizer(*graphFORSTER2, values_initialFORSTER2);
      values_initialFORSTER2 = optimizer.optimize();
      cout << "final error = " << graphFORSTER2->error(values_initialFORSTER2) << endl;
    } catch(gtsam::IndeterminantLinearSystemException &e) {
        ROS_ERROR("FORSTER2 gtsam indeterminate linear system exception!");
        cerr << e.what() << endl;
        exit(EXIT_FAILURE);
    }

    // Debug print time
    boost::posix_time::ptime t2(boost::posix_time::microsec_clock::local_time());
    //ROS_INFO("[GRAPH]: %d states | %d features | %d edges [%.5f seconds to optimize]",(int)ct_state+1,(int)ct_features,(int)graphMODEL1->size(),(t2-t1).total_microseconds()*1e-6);
}


void GraphSolver::optimizeISAM2() {

    // Return if not initialized
    if(!systeminitalized && ct_state < 2)
        return;

    //for (auto it = ct_state_lookup.begin(); it != ct_state_lookup.end(); ++it)
      //std::cout << it->first << " " << it->second << std::endl;

    // Start our timer
    boost::posix_time::ptime t1(boost::posix_time::microsec_clock::local_time());

    //graph_newFORSTER2->print();
    // Perform smoothing update
    try {
      //cout << "initial error = " << graphFORSTER2->error(values_initialFORSTER2) << endl;
      ISAM2Result result = isam2FORSTER2->update(*graph_newFORSTER2, values_newFORSTER2);
      //result.print();
      
      /*cout << "Detailed results:" << endl;
      for (auto keyedStatus : result.detail->variableStatus) {
        const auto& status = keyedStatus.second;
        PrintKey(keyedStatus.first);
        cout << " {" << endl;
        cout << "reeliminated: " << status.isReeliminated << endl;
        cout << "relinearized above thresh: " << status.isAboveRelinThreshold
             << endl;
        cout << "relinearized involved: " << status.isRelinearizeInvolved << endl;
        cout << "relinearized: " << status.isRelinearized << endl;
        cout << "observed: " << status.isObserved << endl;
        cout << "new: " << status.isNew << endl;
        cout << "in the root clique: " << status.inRootClique << endl;
        cout << "}" << endl;
      }*/
        
      values_initialFORSTER2 = isam2FORSTER2->calculateEstimate();
      //cout << "final error = " << graphFORSTER2->error(values_initialFORSTER2) << endl;
    } catch(gtsam::IndeterminantLinearSystemException &e) {
        ROS_ERROR("FORSTER2 gtsam indeterminate linear system exception!");
        cerr << e.what() << endl;
        exit(EXIT_FAILURE);
    }

    // Remove the used up nodes
    values_newFORSTER2.clear();

    // Remove the used up factors
    graph_newFORSTER2->resize(0);

    // Debug print time
    boost::posix_time::ptime t2(boost::posix_time::microsec_clock::local_time());
    //ROS_INFO("[GRAPH]: %d states | %d features | %d edges [%.5f seconds to optimize]",(int)ct_state+1,(int)ct_features,(int)graph_newMODEL1->size(),(t2-t1).total_microseconds()*1e-6);
}


void GraphSolver::trytoinitialize(double timestamp) {

    // If we have already initialized, then just return
    if(systeminitalized)
        return;

    // Wait for enough IMU readings if we initialize from rest
    //if(config->keepRestInit && imu_times.size() < (size_t)config->imuWait)
      //return;
    
    // Wait for VINS state msg if we initialize from their estimate
    //if (config->useVINSInit && true_times.empty())
      //return;
      //

    if (imu_times.size() < 1096)
      return;

    std::cout << imu_times.size() << std::endl;

    //==========================================================================
    // START INITIALIZE!
    //==========================================================================

    // Reading from launch file
    Vector4 q_GtoI = config->prior_qGtoI;
    Vector3 p_IinG = config->prior_pIinG;
    Vector3 v_IinG = config->prior_vIinG;
    Vector3 ba = config->prior_ba;
    Vector3 bg = config->prior_bg;
    
    /**
     * This will try to take the current IMU vector and initalize
     * If there are enough IMU, we should find the current orientation and biases
     * NOTE: This assumes that we are starting from rest!!
     */
      
    // Calculate the mean of the linear acceleration and angular velocity
    /*Eigen::Vector3d linavg = Eigen::Vector3d::Zero();
    Eigen::Vector3d angavg = Eigen::Vector3d::Zero();
    
    if (config->keepRestInit) {
      // Request access to the imu measurements
      std::unique_lock<std::mutex> lock(imu_mutex);

      // Sum up our current accelerations and velocities
      Eigen::Vector3d linsum = Eigen::Vector3d::Zero();
      Eigen::Vector3d angsum = Eigen::Vector3d::Zero();
      for(size_t i=0; i<imu_times.size(); i++) {
          linsum += imu_linaccs.at(i);
          angsum += imu_angvel.at(i);
      }

      // Calculate the mean of the linear acceleration and angular velocity
      linavg = linsum/imu_times.size();
      angavg = angsum/imu_times.size();
     
      // Gyroscope bias can be recovered when starting from rest 
      bg = angavg;

      if (config->useENU) {
        // Note: this is assuming Imu is using ENU coordinate
        // Get z axis, which alines with -g (z_in_G=0,0,1)
        Eigen::Vector3d z_axis = linavg/linavg.norm();

        // Create an x_axis
        Eigen::Vector3d e_1(1,0,0);

        // Make x_axis perpendicular to z
        Eigen::Vector3d x_axis = e_1-z_axis*z_axis.transpose()*e_1;
        x_axis= x_axis/x_axis.norm();

        // Get z from the cross product of these two
        Eigen::Matrix<double,3,1> y_axis = skew_x(z_axis)*x_axis;

        // From these axes get rotation
        Eigen::Matrix<double,3,3> Ro; // Rotation of IMU frame to gravity frame
        Ro.block(0,0,3,1) = x_axis;
        Ro.block(0,1,3,1) = y_axis;
        Ro.block(0,2,3,1) = z_axis;
        gtsam::Quaternion q(Ro);
        q_GtoI = Vector4(q.x(), q.y(), q.z(), q.w());
      
        // Set our biases equal to our noise (subtract our gravity from accelerometer bias)
        //Vector3 ba = linavg - quat_2_Rot(q_GtoI)*config->gravity;
        ba = linavg - Ro*config->gravity;
      }
    }

    // TODO: use this orientation to estimate initial bias when starting from rest
    // Note: the current Imu rotation is assuming NED coordinate
    if (!config->useENU && config->useOrientationInit && !imu_orientation.empty()) {
      std::unique_lock<std::mutex> lock(imu_mutex);
      q_GtoI = imu_orientation.at(imu_orientation.size()-1);
      if (config->keepRestInit) {
        Eigen::Quaterniond q(q_GtoI[3], q_GtoI[0], q_GtoI[1], q_GtoI[2]); // Eigen: w, x, y, z
        Eigen::Matrix3d Ro = q.toRotationMatrix();
        ba = linavg + Ro.transpose() * config->gravity;
      }
    }
 

    // If we have ground truth (like from simulation, then use that)
    // NOTE: VINS mono is assuming ENU coordinate
    if (config->useENU && config->useVINSInit && !true_times.empty()) {
      std::unique_lock<std::mutex> lock(truth_mutex);
      q_GtoI = true_qGtoI.at(true_qGtoI.size()-1);
      p_IinG = true_pIinG.at(true_pIinG.size()-1);
      v_IinG = true_vIinG.at(true_vIinG.size()-1);
      ba = true_ba.at(true_ba.size()-1);
      bg = true_bg.at(true_bg.size()-1);
    }*/

    
    //==========================================================================
    // CREATE PRIOR FACTORS AND INITALIZE GRAPHS
    //==========================================================================

    // Create gtsam prior factor and add it to FORSTER2 graph
    gtsam::State prior_state = gtsam::State(gtsam::Pose3(gtsam::Quaternion(q_GtoI(3), q_GtoI(0), q_GtoI(1), q_GtoI(2)), p_IinG), v_IinG, Bias(ba, bg)); // gtsam::Quaternion(w, x, y, z)
    auto pose_noise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(config->sigma_prior_rotation),
                                                                 Vector3::Constant(config->sigma_prior_translation)).finished());
    auto v_noise = noiseModel::Isotropic::Sigma(3, config->sigma_velocity);
    auto b_noise = noiseModel::Isotropic::Sigma(6, config->sigma_bias);

    graph_newFORSTER2->add(PriorFactor<Pose3>(  Y(ct_state), prior_state.pose(), pose_noise));
    graph_newFORSTER2->add(PriorFactor<Vector3>(V(ct_state), prior_state.v(),    v_noise));
    graph_newFORSTER2->add(PriorFactor<Bias>(   B(ct_state), prior_state.b(),    b_noise));
    
    graphFORSTER2->add(PriorFactor<Pose3>(  Y(ct_state), prior_state.pose(), pose_noise));
    graphFORSTER2->add(PriorFactor<Vector3>(V(ct_state), prior_state.v(),    v_noise));
    graphFORSTER2->add(PriorFactor<Bias>(   B(ct_state), prior_state.b(),    b_noise));

    // Add initial state to FORSTER2 model
    values_newFORSTER2.insert(Y(ct_state), prior_state.pose());
    values_newFORSTER2.insert(V(ct_state), prior_state.v());
    values_newFORSTER2.insert(B(ct_state), prior_state.b());
    values_initialFORSTER2.insert(Y(ct_state), prior_state.pose());
    values_initialFORSTER2.insert(V(ct_state), prior_state.v());
    values_initialFORSTER2.insert(B(ct_state), prior_state.b());

    newTimestampsFORSTER[Y(ct_state)] = timestamp;

    // Add ct state to map
    ct_state_lookup[timestamp] = ct_state;
    timestamp_lookup[ct_state] = timestamp;

    // Clear all old imu messages (keep the last two)
    imu_times.erase(imu_times.begin(), imu_times.end()-1);
    imu_linaccs.erase(imu_linaccs.begin(), imu_linaccs.end()-1);
    imu_angvel.erase(imu_angvel.begin(), imu_angvel.end()-1);
    imu_orientation.erase(imu_orientation.begin(), imu_orientation.end()-1);

    //==========================================================================
    // SET UP IMU PREINTEGRATION
    //==========================================================================
    
    // Create GTSAM preintegration parameters for use with Foster's version
    boost::shared_ptr<PreintegratedCombinedMeasurements::Params> params;
    if (config->useENU)
      params = PreintegratedCombinedMeasurements::Params::MakeSharedU(config->gravity(2));  // Z-up navigation frame: gravity points along negative Z-axis !!!
    else
      params = PreintegratedCombinedMeasurements::Params::MakeSharedD(config->gravity(2));  // Z-down navigation frame: gravity points along positive Z-axis !!!
    
    params->setAccelerometerCovariance(I_3x3 * config->sigma_a_sq);  // acc white noise in continuous
    params->setGyroscopeCovariance(I_3x3 * config->sigma_g_sq);  // gyro white noise in continuous
    params->biasAccCovariance = config->sigma_wa_sq*Matrix33::Identity(3,3);  // acc bias in continuous
    params->biasOmegaCovariance = config->sigma_wg_sq*Matrix33::Identity(3,3);  // gyro bias in continuous
    params->setIntegrationCovariance(I_3x3 * 0.1);  // error committed in integrating position from velocities
    params->biasAccOmegaInt = 1e-5*Matrix66::Identity(6,6); // error in the bias used for preintegration
    
    // Actually create the GTSAM preintegration
    preint_gtsam_ = new PreintegratedCombinedMeasurements(params, prior_state.b());

    // Set our initialized to true!
    systeminitalized = true;

    // Debug info
    ROS_INFO("\033[0;32m[INIT]: orientation = %.4f, %.4f, %.4f, %.4f\033[0m",q_GtoI(0),q_GtoI(1),q_GtoI(2),q_GtoI(3));
    ROS_INFO("\033[0;32m[INIT]: velocity = %.4f, %.4f, %.4f\033[0m",v_IinG(0),v_IinG(1),v_IinG(2));
    ROS_INFO("\033[0;32m[INIT]: position = %.4f, %.4f, %.4f\033[0m",p_IinG(0),p_IinG(1),p_IinG(2));
    ROS_INFO("\033[0;32m[INIT]: bias accel = %.4f, %.4f, %.4f\033[0m",ba(0),ba(1),ba(2));
    ROS_INFO("\033[0;32m[INIT]: bias gyro = %.4f, %.4f, %.4f\033[0m",bg(0),bg(1),bg(2));
}
