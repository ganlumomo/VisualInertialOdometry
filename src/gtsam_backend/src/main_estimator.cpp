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


#include <cmath>
#include <vector>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <unistd.h>

#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <boost/date_time/posix_time/posix_time.hpp>

#include "GraphSolver.h"
#include "utils/Config.h"
#include "utils/ros_utils.h"

string output_filename = "trajectory.csv";
//fprintf(fp_out, "#time(s),x(m),y(m),z(m),qx,qy,qz,qw\n");

// Functions
void setup_config(ros::NodeHandle& nh, Config* config);
void setup_subpub(ros::NodeHandle& nh);
void handle_measurement_imu(sensor_msgs::Imu::Ptr msg);
//void handle_measurement_uv(common::CameraMeasurement::Ptr msg);
void handle_measurement_uv(const sensor_msgs::PointCloudConstPtr& msg);
void optimize_graph(double timestamp);
void publish_state(double timestamp, gtsam::State& state, Eigen::Matrix<double, 6, 6>& covariance,
                   ros::Publisher& pubPath, ros::Publisher& pubPoseIMU, vector<geometry_msgs::PoseStamped>& poses_est);
void publish_trajectory(double timestamp, Trajectory& trajectory, ros::Publisher& pubPath);
void publish_FeatureCloud(double timestamp);


// Subscribers and publishers
ros::Publisher pubFeatureCloudsFORSTER2;
ros::Publisher pubPathFORSTER2;
ros::Publisher pubTrajFORSTER2;
ros::Publisher pubPoseIMUFORSTER2;
ros::Subscriber subUVMeas;
ros::Subscriber subIMUMeas;
ros::Subscriber subPOSETrue;
ros::Subscriber subICPtrigger;
ros::Subscriber subICPMeas;
ros::Subscriber subVLCMeas;

// Master config and graph object
Config* config;
GraphSolver* graphsolver;

// Variables needed for visualization
boost::thread* t_viz;
unsigned int poses_seq = 0;
vector<geometry_msgs::PoseStamped> poses_estFORSTER2;

// Control parameters
int skip = 0;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "estimator");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    // Setup our config object
    config = new Config();
    setup_config(nhPrivate, config);

    // Sleep a bit to let publishers setup
    sleep(2);

    // Next up, setup our subscribers and publishers
    setup_subpub(nh);

    // Create our gtsam estimation class
    graphsolver = new GraphSolver(config);

    // Create our visualization thread
    //t_viz = new boost::thread(boost::bind(&thread_visualization));

    // Let ROS handle callbacks if not doing serial
        ros::spin();

    // Shutdown subscribers
    pubFeatureCloudsFORSTER2.shutdown();
    pubPathFORSTER2.shutdown();
    pubPoseIMUFORSTER2.shutdown();
    subPOSETrue.shutdown();
    subIMUMeas.shutdown();
    subUVMeas.shutdown();

    // Done!
    return EXIT_SUCCESS;
}


/**
 * \brief This function will read in config values from the node handler
 * If the values are not found, then use a default for each one.
 */
void setup_config(ros::NodeHandle& nh, Config* config) {

    // Read sensor timeshift
    nh.param<double>("timeshift", config->timeshift, 0.0);
      
    // Read in our CAMERA noise values
    nh.param<double>("sigma_camera", config->sigma_camera, 1.0/484.1316);
    config->sigma_camera_sq = std::pow(config->sigma_camera,2);

    // Read in our IMU noise values
    nh.param<double>("gyroscope_noise_density", config->sigma_g, 0.005);
    config->sigma_g_sq = std::pow(config->sigma_g,2);
    nh.param<double>("accelerometer_noise_density", config->sigma_a, 0.01);
    config->sigma_a_sq = std::pow(config->sigma_a,2);
    nh.param<double>("gyroscope_random_walk", config->sigma_wg, 4.0e-06);
    config->sigma_wg_sq = std::pow(config->sigma_wg,2);
    nh.param<double>("accelerometer_random_walk", config->sigma_wa, 0.002);
    config->sigma_wa_sq = std::pow(config->sigma_wa,2);

    // Read in our Initialization noise values
    nh.param<double>("sigma_prior_rotation",    config->sigma_prior_rotation, 1.0e-4);
    nh.param<double>("sigma_prior_translation", config->sigma_prior_translation, 1.0e-4);
    nh.param<double>("sigma_velocity",          config->sigma_velocity, 1.0e-4);
    nh.param<double>("sigma_bias",              config->sigma_bias, 1.0e-4);
    nh.param<double>("sigma_pose_rotation",     config->sigma_pose_rotation, 1.0e-4);
    nh.param<double>("sigma_pose_translation",  config->sigma_pose_translation, 1.0e-4);

    // Load priors
    std::vector<double> prior_qGtoI = {0, 0, 0, 1.0};
    nh.param<std::vector<double>>("prior_qGtoI", prior_qGtoI, prior_qGtoI);
    for(size_t i=0;i<4;i++) config->prior_qGtoI(i) = prior_qGtoI.at(i);
    
    std::vector<double> prior_pIinG = {0, 0, 0};
    nh.param<std::vector<double>>("prior_pIinG", prior_pIinG, prior_pIinG);
    for(size_t i=0;i<3;i++) config->prior_pIinG(i) = prior_pIinG.at(i);

    std::vector<double> prior_vIinG = {0, 0, 0};
    nh.param<std::vector<double>>("prior_vIinG", prior_vIinG, prior_vIinG);
    for(size_t i=0;i<3;i++) config->prior_vIinG(i) = prior_vIinG.at(i);

    std::vector<double> prior_ba = {0, 0, 0};
    nh.param<std::vector<double>>("prior_ba", prior_ba, prior_ba);
    for(size_t i=0;i<3;i++) config->prior_ba(i) = prior_ba.at(i);

    std::vector<double> prior_bg = {0, 0, 0};
    nh.param<std::vector<double>>("prior_bg", prior_bg, prior_bg);
    for(size_t i=0;i<3;i++) config->prior_bg(i) = prior_bg.at(i);


    // Load global gravity
    std::vector<double> gravity = {0,0,9.8};
    nh.param<std::vector<double>>("gravity", gravity, gravity);
    for(size_t i=0;i<3;i++) config->gravity(i,0) = gravity.at(i);

    // SIMULATION: Simulation config for the transform
    // TODO: read from launch file!!!
    //config->R_C0toI << 1,0,0,0,1,0,0,0,1;
    //config->R_C0toI << 0,0,1,0,1,0,-1,0,0; //-90 about the y-axis
    //config->p_IinC0 << 0,0,0;
    config->R_C1toI << 1,0,0,0,1,0,0,0,1;
    //config->R_C1toI << 0,0,1,0,1,0,-1,0,0; //-90 about the y-axis
    config->p_IinC1 << 0.1,0,0;
    config->p_IinC1 = -config->R_C1toI.transpose()*config->p_IinC1;

    // Read in Rotation and Translation from camera frame to Imu frame
    std::vector<double> R_C0toI = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    nh.param<std::vector<double>>("R_C0toI", R_C0toI, R_C0toI);
    for(size_t i=0; i<9; i++) config->R_C0toI(i) = R_C0toI.at(i);

    std::vector<double> p_IinC0 = {0, 0, 0};
    nh.param<std::vector<double>>("p_IinC0", p_IinC0, p_IinC0);
    for(size_t i=0; i<3; i++) config->p_IinC0(i) = p_IinC0.at(i);


    // Load if we should do a serial simulation
    nh.param<bool>("useSerialSIM", config->useSerialSIM, true);

    // Load simulation path
    config->simPath = "";
    nh.param<std::string>("simPath", config->simPath, config->simPath);

    config->fixedId = "global";
    nh.param<std::string>("fixedId", config->fixedId, config->fixedId);
    
    nh.param<bool>("useENU", config->useENU, true);

    // Load if we should inialize using the groundtruth pose
    nh.param<bool>("keepRestInit",       config->keepRestInit,       true);
    nh.param<bool>("useOrientationInit", config->useOrientationInit, false);
    nh.param<bool>("useVINSInit",        config->useVINSInit,        false);
    nh.param<bool>("useOrientationInitValues", config->useOrientationInitValues, false);
    nh.param<bool>("useVINSInitValues",   config->useVINSInitValues, false);
    nh.param<bool>("useSmartFactor",           config->useSmartFactor,           true);
    nh.param<bool>("useVisualLoopClosure",     config->useVisualLoopClosure,     true);
    nh.param<int>("secondPriorState",         config->secondPriorState,     -1);

    // Max window we should update uv coordinates by
    nh.param<int>("uvWindowSize", config->uvWindowSize, 25);

    // Number of poses we should see a feature from before we initialize it
    nh.param<int>("minPoseFeatureInit", config->minPoseFeatureInit, 5);

    // Read in IMU rates (hertz) and number of IMU message we should wait to initialize from
    nh.param<int>("imurate", config->imuRate, 100);
    nh.param<int>("camrate", config->camRate, 10);
    nh.param<int>("skipkf",  config->skipkf,  0);
    nh.param<int>("imuwait", config->imuWait, 300);

    // Debug print to screen for the user
    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
    cout << "\t- sensor timeshift: " << config->timeshift << endl;
    
    cout << "Noise Parameters:" << endl;
    cout << "\t- sigma_camera:  " << config->sigma_camera << endl;
    cout << "\t- sigma_g:  " << config->sigma_g << endl;
    cout << "\t- sigma_wg: " << config->sigma_wg << endl;
    cout << "\t- sigma_a:  " << config->sigma_a << endl;
    cout << "\t- sigma_wa: " << config->sigma_wa << endl;
    cout << "\t- sigma_prior_rotation: "    << config->sigma_prior_rotation << endl;
    cout << "\t- sigma_prior_translation: " << config->sigma_prior_translation << endl;
    cout << "\t- sigma_velocity: "          << config->sigma_velocity << endl;
    cout << "\t- sigma_bias: "              << config->sigma_bias << endl;
    cout << "\t- sigma_pose_rotation: "     << config->sigma_pose_rotation << endl;
    cout << "\t- sigma_pose_translation: "  << config->sigma_pose_translation << endl;
 
    cout << "Initialization:" << endl;
    cout << "\t- prior_q_GtoI: " << endl << config->prior_qGtoI.transpose() << endl;
    cout << "\t- prior_p_IinG: " << endl << config->prior_pIinG.transpose() << endl;
    cout << "\t- prior_ba: "     << endl << config->prior_ba.transpose() << endl;
    cout << "\t- prior_bg: "     << endl << config->prior_bg.transpose() << endl;
    
    cout << "World Parameters:" << endl;
    cout << "\t- imuRate: " << config->imuRate << endl;
    cout << "\t- camRate: " << config->camRate << endl;
    cout << "\t- skipkf: "  << config->skipkf << endl;
    cout << "\t- imuWait: " << config->imuWait << endl;
    cout << "\t- gravity: " << config->gravity.format(CommaInitFmt) << endl;
    cout << "Transform Parameters:" << endl;
    cout << "\t- R_C0toI: " << endl << config->R_C0toI << endl;
    cout << "\t- p_IinC0: " << endl << config->p_IinC0.transpose() << endl;
    cout << "\t- R_C1toI: " << endl << config->R_C1toI << endl;
    cout << "\t- p_IinC1: " << endl << config->p_IinC1.transpose() << endl;
    cout << "General Parameters:" << endl;
    cout << "\t- keep rest init: " << std::boolalpha << config->keepRestInit << endl;
    cout << "\t- use orientation init: " << std::boolalpha << config->useOrientationInit << endl;
    cout << "\t- use vins init: " << std::boolalpha << config->useVINSInit << endl;
    cout << "\t- use orientation init values: " << std::boolalpha << config->useOrientationInitValues << endl;
    cout << "\t- use vins init values: " << std::boolalpha << config->useVINSInitValues << endl;
    cout << "\t- smart factor: " << std::boolalpha << config->useSmartFactor << endl;
    cout << "\t- visual loop closure: " << std::boolalpha << config->useVisualLoopClosure << endl;
    cout << "\t- second prior state: " << std::boolalpha << config->secondPriorState << endl;
}



/**
 * \brief This performs the setup for all the ROS subscriber and publishers needed
 */
void setup_subpub(ros::NodeHandle& nh) {

    // Point cloud visualization
    pubFeatureCloudsFORSTER2 = nh.advertise<sensor_msgs::PointCloud2>("cpi_compare/feature_cloud_forster2", 2);
    ROS_INFO("Publishing: %s", pubFeatureCloudsFORSTER2.getTopic().c_str());

    // Path visualization
    pubPathFORSTER2 = nh.advertise<nav_msgs::Path>("cpi_compare/path_imu_forster2", 2);
    ROS_INFO("Publishing: %s", pubPathFORSTER2.getTopic().c_str());

    // Trajectory visualization
    pubTrajFORSTER2 = nh.advertise<nav_msgs::Path>("cpi_compare/traj_imu_forster2", 2);
    ROS_INFO("Publishing: %s", pubPathFORSTER2.getTopic().c_str());

    // IMU pose visualization
    pubPoseIMUFORSTER2 = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("cpi_compare/pose_imu_forster2", 2);
    ROS_INFO("Publishing: %s", pubPoseIMUFORSTER2.getTopic().c_str());

    // Subscribe to our IMU measurements
    subIMUMeas = nh.subscribe("cpi_compare/data_imu", 2000, handle_measurement_imu);
    ROS_INFO("Subscribing: %s", subIMUMeas.getTopic().c_str());

    // Subscribe to the uv measurements
    subUVMeas = nh.subscribe("cpi_compare/data_uv", 500, handle_measurement_uv);
    ROS_INFO("Subscribing: %s", subUVMeas.getTopic().c_str());
    
    // Subscribe to the icp triggers
}




/**
 * \brief Subscription callback for IMU messages from the sensor
 * @param msg IMU messagei
 */
void handle_measurement_imu(sensor_msgs::Imu::Ptr msg) {

    // Convert to eigen format
    Eigen::Vector3d linearacceleration;
    linearacceleration << msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z;
    Eigen::Vector3d angularvelocity;
    angularvelocity << msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z;
    Eigen::Vector4d orientation;
    orientation << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;

    // Send to our graph solver
    graphsolver->addmeasurement_imu(msg->header.stamp.toSec(),linearacceleration,angularvelocity, orientation);

}





/**
 * \brief Subscription callback for UV coordinate measurements
 * @param msg CameraMeasurement message
 */

void handle_measurement_uv(const sensor_msgs::PointCloudConstPtr& msg) {

    if (graphsolver->is_initialized() && skip != config->skipkf) {
      skip++;
      return;
    } else
      skip = 0;

    //==========================================================================
    // PROCESS NEW MEASUREMENTS
    //==========================================================================

    // Our feature measurements
    std::vector<uint> leftids;
    std::vector<Eigen::Vector2d> leftuv;
    std::vector<uint> rightids;
    std::vector<Eigen::Vector2d> rightuv;

    // Loop through LEFT features and append
    for(size_t i=0; i<msg->points.size(); i++) {
        int v = msg->channels[0].values[i] + 0.5;
        int id = v / 1;
        leftids.push_back((uint)id);
        rightids.push_back((uint)id);
        Eigen::Vector2d uv;
        uv << msg->points[i].x, msg->points[i].y;
        leftuv.push_back(uv);
        rightuv.push_back(uv);
    }

    // We have successfully handled all features, lets send them to the optimizer
    graphsolver->addmeasurement_uv(msg->header.stamp.toSec() + config->timeshift, leftids,leftuv,rightids,rightuv);

    //==========================================================================
    // OPTIMIZE GRAPH
    //==========================================================================

    optimize_graph(msg->header.stamp.toSec());
}

/*
void handle_measurement_uv(common::CameraMeasurement::Ptr msg) {

    if (graphsolver->is_initialized() && skip != config->skipkf) {
      skip++;
      return;
    } else
      skip = 0;

    //==========================================================================
    // PROCESS NEW MEASUREMENTS
    //==========================================================================

    // Our feature measurements
    std::vector<uint> leftids;
    std::vector<Eigen::Vector2d> leftuv;
    std::vector<uint> rightids;
    std::vector<Eigen::Vector2d> rightuv;

    // Loop through LEFT features and append
    for(size_t i=0; i<msg->features_left.size(); i++) {
        leftids.push_back((uint)msg->features_left.at(i).id);
        Eigen::Vector2d uv;
        uv << msg->features_left.at(i).u, msg->features_left.at(i).v;
        leftuv.push_back(uv);
    }

    // Loop through RIGHT features and append
    for(size_t i=0; i<msg->features_right.size(); i++) {
        rightids.push_back((uint)msg->features_right.at(i).id);
        Eigen::Vector2d uv;
        uv << msg->features_right.at(i).u, msg->features_right.at(i).v;
        rightuv.push_back(uv);
    }

    // We have successfully handled all features, lets send them to the optimizer
    graphsolver->addmeasurement_uv(msg->header.stamp.toSec() + config->timeshift, leftids,leftuv,rightids,rightuv);

    //==========================================================================
    // OPTIMIZE GRAPH
    //==========================================================================

    optimize_graph(msg->header.stamp.toSec());
}*/



/**
 * \brief Subscription callback for visual loop closure messages
 * @param matched_frame message
 */




void optimize_graph(double timestamp) {
    
    // Optimize the graph! GO GO GO!!!
    //if (graphsolver->ct_state <= 8)
      //graphsolver->optimizeLM();
    //else
    graphsolver->optimizeISAM2();

    // Reset Imu measurements
    graphsolver->reset_imu_integration();

    //==========================================================================
    // ROS VISUALIZATION
    //==========================================================================

    // Temp empty covariance, since this takes a while to compute
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double,6,6>::Zero();

    // FORSTER2 DISCRETE
    gtsam::State state = graphsolver->get_current_state();
    publish_state(timestamp, state, covariance, pubPathFORSTER2, pubPoseIMUFORSTER2, poses_estFORSTER2);
    std::vector<pair<double, gtsam::State>> trajectory = graphsolver->get_trajectory();
    publish_trajectory(timestamp, trajectory, pubTrajFORSTER2);

    // Publish our feature cloud
    publish_FeatureCloud(timestamp);
}


/**
 * Given a State, this should publish it onto ROS for visualization
 */
void publish_state(double timestamp, gtsam::State& state, Eigen::Matrix<double, 6, 6>& covariance,
                   ros::Publisher& pubPath, ros::Publisher& pubPoseIMU, vector<geometry_msgs::PoseStamped>& poses_est) {

    // Return if we have not initialized yet
    if(!graphsolver->is_initialized())
        return;

    // Create our stamped pose with covariance
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.stamp = ros::Time(timestamp);
    pose.header.frame_id = config->fixedId;
    utils::ToPoseWithCovariance(state.pose(), covariance, pose.pose);
    
    // Publish this pose
    pubPoseIMU.publish(pose);

    // Create our stamped pose for path publishing
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time(timestamp);
    poseStamped.header.frame_id = config->fixedId;
    poseStamped.pose = pose.pose.pose;
    poses_est.push_back(poseStamped);

    // Create pose arrays and publish
    nav_msgs::Path patharr;
    patharr.header.stamp = ros::Time(timestamp);
    patharr.header.seq = poses_seq++;
    patharr.header.frame_id = config->fixedId;
    patharr.poses = poses_est;
    pubPath.publish(patharr);

    // Debug print current state
    std::cout << state << endl;
}


void publish_trajectory(double timestamp, Trajectory& trajectory, ros::Publisher& pubPath) {
  
  if (trajectory.empty())
    return;

  vector<geometry_msgs::PoseStamped> traj_est;
  FILE* fp_out = fopen(output_filename.c_str(), "w+");
  for (auto it = trajectory.begin(); it != trajectory.end(); it++) {
    // Create our stamped pose for path publishing
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time(it->first);
    poseStamped.header.frame_id = config->fixedId;
    utils::ToPose(it->second.pose(), poseStamped.pose);
    traj_est.push_back(poseStamped);
       fprintf(fp_out, "%f, %f, %f, %f, %f, %f, %f, %f\n", 
              it->first, poseStamped.pose.position.x, poseStamped.pose.position.y, poseStamped.pose.position.z,
              poseStamped.pose.orientation.x, poseStamped.pose.orientation.y, poseStamped.pose.orientation.z, poseStamped.pose.orientation.w);
  }
  fclose(fp_out);
  
  // Create pose arrays and publish
  nav_msgs::Path patharr;
  patharr.header.stamp = ros::Time(timestamp);
  patharr.header.seq = poses_seq++;
  patharr.header.frame_id = config->fixedId;
  patharr.poses = traj_est;
  pubPath.publish(patharr);
}


/**
 * Given a set of point clouds, this should publish them onto ROS
 * NOTE: POINTS SHOULD ALREADY BE IN THE GLOBAL FRAME OF REFERENCE!!!!@@!#
 */
void publish_FeatureCloud(double timestamp) {

    // Return if we have not initialized yet
    if(!graphsolver->is_initialized())
        return;

    // Loop through and create a cloud
    std::vector<Eigen::Vector3d> points = graphsolver->get_current_features();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
    for(size_t i=0; i<points.size(); i++) {
        pcl::PointXYZ pt;
        pt.x = points.at(i)(0);
        pt.y = points.at(i)(1);
        pt.z = points.at(i)(2);
        cloud->push_back(pt);
    }

    // Publish the downstampled cloud
    sensor_msgs::PointCloud2 msgOut;
    pcl::toROSMsg(*cloud, msgOut);
    msgOut.header.frame_id = config->fixedId;
    msgOut.header.stamp = ros::Time(timestamp);
    pubFeatureCloudsFORSTER2.publish(msgOut);
}
