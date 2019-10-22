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


#ifndef GRAPHSOLVER_H
#define GRAPHSOLVER_H

#include <mutex>
#include <thread>
#include <deque>
#include <fstream>
#include <unordered_map>

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "utils/State.h"
#include "utils/quat_ops.h"
#include "utils/Config.h"
#include "utils/feature.h"

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std;
using namespace gtsam;


using gtsam::symbol_shorthand::X; // X: our JPL states
using gtsam::symbol_shorthand::F; // F: our feature node

using gtsam::symbol_shorthand::Y; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

typedef std::vector<std::pair<double, gtsam::State>> Trajectory;
typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;

class GraphSolver {
public:


    /**
     * Default constructor
     */
    GraphSolver(Config* config) {
        // Store our config object
        this->config = config;
        // Initalize our graphs
        this->graphFORSTER2 = new gtsam::NonlinearFactorGraph();
        this->graph_newFORSTER2 = new gtsam::NonlinearFactorGraph();
        
        // ISAM2 solver
        ISAM2Params isam_params;
  
        isam_params.relinearizeThreshold = 0.01;
        isam_params.relinearizeSkip = 1;
        isam_params.cacheLinearizedFactors = false;
        isam_params.enableDetailedResults = true;
        isam_params.print();
     
        this->isam2FORSTER2 = new ISAM2(isam_params);

        gtsam::LevenbergMarquardtParams params;
        params.setVerbosity("ERROR"); // SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
        this->smootherBatch = new BatchFixedLagSmoother(3,params,true);
    }

    /// Function that takes in IMU measurements for use in preintegration measurements
    void addtrue_pose(double timestamp, Eigen::Vector4d q_GtoI, Eigen::Vector3d p_IinG);
    void addtrue_state(Eigen::Vector3d v_IinG, Eigen::Vector3d ba, Eigen::Vector3d bg);

    /// Function that takes in IMU measurements for use in preintegration measurements
    void addmeasurement_imu(double timestamp, Eigen::Vector3d linacc, Eigen::Vector3d angvel, Eigen::Vector4d orientation);

    /// Function that takes in UV measurements that will be used as "features" in our graph
    void addmeasurement_uv(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv,
                           std::vector<uint> rightids, std::vector<Eigen::Vector2d> rightuv);

    void addmeasurement_icp(double timestamp1, double timestamp2, Eigen::Vector4d q_1to2, Eigen::Vector3d p_2in1);
    void addtrigger_icp(double timestamp);
    void add_visual_loop_closure(double curr_timestamp, double prev_timestamp,
                                 std::vector<uint> curr_ids, std::vector<Eigen::Vector2d> curr_uv,
                                 std::vector<uint> prev_ids, std::vector<Eigen::Vector2d> prev_uv);

    /// This function will optimize the graph
    void optimizeLM();
    void optimizeISAM2();
    void optimizeBatch();
    
    /// Function will reset preintegrated Imu measurements
    void reset_imu_integration();

    /// Will return true if the system is initialized
    bool is_initialized() {
        return systeminitalized;
    }

    /// This function returns the current state, return origin if we have not initialized yet
    State get_state(size_t ct) {
      // Ensure valid states
      assert (ct <= ct_state);
      if (!values_initialFORSTER2.exists(Y(ct)))
        return State();
      return State(values_initialFORSTER2.at<Pose3>(Y(ct)),
                   values_initialFORSTER2.at<Vector3>(V(ct)),
                   values_initialFORSTER2.at<Bias>(B(ct)));
    }

    State get_current_state() { 
      return get_state(ct_state);
    }


    Trajectory get_trajectory() {
      // Return if we do not have any nodes yet
      if (values_initialFORSTER2.empty()) {
        Trajectory traj;
        traj.push_back(std::make_pair(0.0, gtsam::State()));
        return traj;
      }
      Trajectory trajectory;
      // Else loop through the states and return them
      for (size_t i=1; i<=ct_state; i++) {
        double timestamp = timestamp_lookup[i];
        trajectory.push_back(std::make_pair(timestamp, get_state(i)));
      }
      return trajectory;
    }
    
    /// Returns the currently tracked features
    std::vector<Eigen::Vector3d> get_current_features() {
        // Return if we do not have any nodes yet
        if(values_initialFORSTER2.empty()) {
            return std::vector<Eigen::Vector3d>();
        }
        // Our vector of points in the global
        std::vector<Eigen::Vector3d> features;
        // Else loop through the features and return them
        for (auto element : measurement_smart_lookup_left) {
         boost::optional<Point3> point = element.second->point(values_initialFORSTER2);
         if (point)
           features.push_back(*point);
        }
        return features;
    }


private:

    /// Function which will try to initalize our graph using the current IMU measurements
    void trytoinitialize(double timestamp);

    /// Function that will compound the GTSAM preintegrator to get discrete preintegration measurement
    CombinedImuFactor create_imu_factor(double updatetime, gtsam::Values& values_initial);

    /// Function will get the predicted JPL Navigation State based on this generated measurement
    State get_predicted_state(gtsam::Values& values_initial);

    /// Smart feature measurements
    void process_feat_smart(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv,
                            std::vector<uint> rightids, std::vector<Eigen::Vector2d> rightuv);
    /// Norm feature measurements
    void process_feat_norm(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv,
                            std::vector<uint> rightids, std::vector<Eigen::Vector2d> rightuv);

    /// Loop closure feature measurements
    void process_feat_loop(double timestamp1, double timestamp2,
                           std::vector<uint> curr_ids, std::vector<Eigen::Vector2d> curr_uv,
                           std::vector<uint> prev_ids, std::vector<Eigen::Vector2d> prev_uv);

    /// Imu Preintegration
    gtsam::PreintegratedCombinedMeasurements* preint_gtsam_;

    //==========================================================================
    // OPTIMIZATION OBJECTS
    //==========================================================================

    // Master non-linear GTSAM graph, all created factors
    gtsam::NonlinearFactorGraph* graphFORSTER2;

    // New factors that have not been optimized yet
    gtsam::NonlinearFactorGraph* graph_newFORSTER2;

    // ISAM2 solvers
    ISAM2* isam2FORSTER2;
    BatchFixedLagSmoother* smootherBatch;
    FixedLagSmoother::KeyTimestampMap newTimestampsFORSTER;

    size_t ct_state = 0;
    // Current ID of state and features
    size_t ct_features = 0;

    // All created nodes
    gtsam::Values values_initialFORSTER2;

    // New nodes that have not been optimized
    gtsam::Values values_newFORSTER2;

    //==========================================================================
    // SYSTEM / HOUSEKEEPING VARIABLES
    //==========================================================================

    /// Our config object (has all sensor noise values)
    Config* config;

    /// Boolean that tracks if we have initialized
    bool systeminitalized = false;

    std::unordered_map<double, size_t> ct_state_lookup; // ct state based on timestamp
    std::unordered_map<size_t, double> timestamp_lookup;

    // Our true POSE data
    std::mutex truth_mutex;
    std::deque<double> true_times;
    std::deque<Eigen::Vector4d> true_qGtoI;
    std::deque<Eigen::Vector3d> true_pIinG;
    std::deque<Eigen::Vector3d> true_vIinG;
    std::deque<Eigen::Vector3d> true_ba;
    std::deque<Eigen::Vector3d> true_bg;

    // Our IMU data from the sensor
    std::mutex imu_mutex;
    std::deque<double> imu_times;
    std::deque<Eigen::Vector3d> imu_linaccs;
    std::deque<Eigen::Vector3d> imu_angvel;
    std::deque<Eigen::Vector4d> imu_orientation;

    /// Lookup tables for features and incoming measurements
    std::mutex features_mutex;
    std::unordered_map<int, size_t> measurement_lookup; //< node ID of feature if added into graph
    std::unordered_map<int, size_t> measurement_state_lookup; //< state ID of feature if added into graph
    std::unordered_map<size_t, size_t> measurement_anchor_lookup; //< state ID of anchor pose based on feature ID
    std::unordered_map<int, feature> measurement_queue; //< queue of features that have not been added
    std::unordered_map<int, SmartFactor::shared_ptr> measurement_smart_lookup_left;
};



#endif /* GRAPHSOLVER_H */
