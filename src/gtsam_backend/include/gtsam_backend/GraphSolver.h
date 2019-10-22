#pragma

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
#include "utils/Config.h"

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/SmartProjectionPoseFactor.h>
#include <gtsam/slam/BetweenFactor.h>

using namespace std;
using namespace gtsam;

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

typedef std::vector<std::pair<double, gtsam::State>> Trajectory;
typedef SmartProjectionPoseFactor<Cal3_S2> SmartFactor;

class GraphSolver {
public:
  GraphSolver(Config* config) {
    
    // Set up config
    this->config = config;
    this->graph = new gtsam::NonlinearFactorGraph();
    this->graph_new = new gtsam::NonlinearFactorGraph();
    
    // ISAM2 solver
    ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.01;
    isam_params.relinearizeSkip = 1;
    isam_params.cacheLinearizedFactors = false;
    isam_params.enableDetailedResults = true;
    isam_params.print();
    this->isam2 = new ISAM2(isam_params);
  }

  /// Will return true if the system is initialized
  inline bool is_initialized() { return systeminitalized; }

  /// Function that takes in IMU measurements for use in preintegration measurements
  void addmeasurement_imu(double timestamp, Eigen::Vector3d linacc, Eigen::Vector3d angvel, Eigen::Vector4d orientation);

  /// Function that takes in UV measurements that will be used as "features" in our graph
  void addmeasurement_uv(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv);

  /// This function will optimize the graph
  void optimize();
  
  /// This function returns the current state, return origin if we have not initialized yet
  State get_state(size_t ct) {
    // Ensure valid states
    assert (ct <= ct_state);
    if (!values_initial.exists(X(ct)))
      return State();
    return State(values_initial.at<Pose3>(  X(ct)),
                 values_initial.at<Vector3>(V(ct)),
                 values_initial.at<Bias>(   B(ct)));
  }

  State get_current_state() { 
    return get_state(ct_state);
  }


  Trajectory get_trajectory() {
    // Return if we do not have any nodes yet
    if (values_initial.empty()) {
      Trajectory traj;
      traj.push_back(std::make_pair(0.0, gtsam::State()));
      return traj;
    }
    Trajectory trajectory;
    // Else loop through the states and return them
    for (size_t i = 1; i <= ct_state; ++i) {
      double timestamp = timestamp_lookup[i];
      trajectory.push_back(std::make_pair(timestamp, get_state(i)));
    }
    return trajectory;
  }
  
  /// Returns the currently tracked features
  std::vector<Eigen::Vector3d> get_current_features() {
      // Return if we do not have any nodes yet
      if(values_initial.empty()) {
          return std::vector<Eigen::Vector3d>();
      }
      // Our vector of points in the global
      std::vector<Eigen::Vector3d> features;
      // Else loop through the features and return them
      for (auto element : measurement_smart_lookup_left) {
       boost::optional<Point3> point = element.second->point(values_initial);
       if (point)
         features.push_back(*point);
      }
      return features;
  }

private:

  /// Function which will try to initalize our graph using the current IMU measurements
  void initialize(double timestamp);

  // ******************************* TODO ********************************* //
  bool set_imu_preintegration(const gtsam::State& prior_state);

  /// Function that will compound the GTSAM preintegrator to get discrete preintegration measurement
  CombinedImuFactor create_imu_factor(double updatetime, gtsam::Values& values_initial);

  /// Function will get the predicted Navigation State based on this generated measurement 
  State get_predicted_state(gtsam::Values& values_initial);

  /// Function that will reset imu preintegration
  void reset_imu_integration();

  /// Smart feature measurements
  void process_feat_smart(double timestamp, std::vector<uint> leftids, std::vector<Eigen::Vector2d> leftuv);
  // ******************************* END TODO ********************************* //

  /// Imu Preintegration
  gtsam::PreintegratedCombinedMeasurements* preint_gtsam;

  /// Config object (has all sensor noise values)
  Config* config;

  // ISAM2 solvers
  ISAM2* isam2;

  // New factors that have not been optimized yet
  gtsam::NonlinearFactorGraph* graph_new;
  
  // Master non-linear GTSAM graph, all created factors
  gtsam::NonlinearFactorGraph* graph;

  // New nodes that have not been optimized
  gtsam::Values values_new;
  
  // All created nodes
  gtsam::Values values_initial;

  // Current ID of state and features
  size_t ct_state = 0;
  size_t ct_features = 0;

  /// Boolean that tracks if we have initialized
  bool systeminitalized = false;

  std::unordered_map<double, size_t> ct_state_lookup; // ct state based on timestamp
  std::unordered_map<size_t, double> timestamp_lookup;

  // IMU data from the sensor
  std::mutex imu_mutex;
  std::deque<double> imu_times;
  std::deque<Eigen::Vector3d> imu_linaccs;
  std::deque<Eigen::Vector3d> imu_angvel;
  std::deque<Eigen::Vector4d> imu_orientation;

  /// Lookup tables for features
  std::mutex features_mutex;
  std::unordered_map<int, SmartFactor::shared_ptr> measurement_smart_lookup_left;
};
