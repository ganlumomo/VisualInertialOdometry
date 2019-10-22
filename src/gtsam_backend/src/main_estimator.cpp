/* 
 * Lu Gan
 * ganlu@umich.edu
 */

#include <vector>

// Ros related
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl_conversions/pcl_conversions.h>

#include "GraphSolver.h"
#include "utils/Config.h"
#include "utils/Convert.h"

// Functions
void setup_config(ros::NodeHandle& nh, Config* config);
void setup_subpub(ros::NodeHandle& nh);
void handle_measurement_imu(const sensor_msgs::ImuConstPtr& msg);
void handle_measurement_uv(const sensor_msgs::PointCloudConstPtr& msg);
void optimize_graph(double timestamp);
void publish_state(double timestamp, gtsam::State& state);
void publish_trajectory(double timestamp, Trajectory& trajectory);
void publish_cloud(double timestamp);

// Subscribers and publishers
ros::Subscriber subIMUMeas;
ros::Subscriber subUVMeas;
ros::Publisher pubPoseIMU;
ros::Publisher pubPathIMU;
ros::Publisher pubFeatureCloud;

// Master config and graph object
Config* config;
GraphSolver* graphsolver;
int poses_seq = 0;
int skip = 0;

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "vio");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate("~");

  config = new Config();
  setup_config(nhPrivate, config);
  setup_subpub(nh);
  
  sleep(2);
  
  graphsolver = new GraphSolver(config);

  ros::spin();
  return EXIT_SUCCESS;
}

void setup_config(ros::NodeHandle& nh, Config* config) {

    config->fixedId = "global";
    nh.param<std::string>("fixedId", config->fixedId, config->fixedId);
    // Load global gravity
    std::vector<double> gravity = {0, 0, 9.8};
    nh.param<std::vector<double>>("gravity", gravity, gravity);
    for (size_t i = 0; i < 3; ++i) config->gravity(i, 0) = gravity.at(i);
   
    // Read in number of IMU and FEAT message we should wait to initialize from
    nh.param<int>("imuWait", config->imuWait, 300);
    nh.param<int>("featWait", config->featWait,  0);

    // Read in Rotation and Translation from camera frame to Imu frame
    std::vector<double> R_C0toI = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    nh.param<std::vector<double>>("R_C0toI", R_C0toI, R_C0toI);
    for (size_t i = 0; i < 9; ++i) config->R_C0toI(i) = R_C0toI.at(i);

    std::vector<double> p_IinC0 = {0, 0, 0};
    nh.param<std::vector<double>>("p_IinC0", p_IinC0, p_IinC0);
    for (size_t i = 0; i < 3; ++i) config->p_IinC0(i) = p_IinC0.at(i);

    // Load priors
    std::vector<double> prior_qGtoI = {0, 0, 0, 1.0};
    nh.param<std::vector<double>>("prior_qGtoI", prior_qGtoI, prior_qGtoI);
    for (size_t i = 0; i < 4; ++i) config->prior_qGtoI(i) = prior_qGtoI.at(i);
    
    std::vector<double> prior_pIinG = {0, 0, 0};
    nh.param<std::vector<double>>("prior_pIinG", prior_pIinG, prior_pIinG);
    for (size_t i = 0; i < 3; ++i) config->prior_pIinG(i) = prior_pIinG.at(i);

    std::vector<double> prior_vIinG = {0, 0, 0};
    nh.param<std::vector<double>>("prior_vIinG", prior_vIinG, prior_vIinG);
    for (size_t i = 0; i < 3; ++i) config->prior_vIinG(i) = prior_vIinG.at(i);

    std::vector<double> prior_ba = {0, 0, 0};
    nh.param<std::vector<double>>("prior_ba", prior_ba, prior_ba);
    for (size_t i = 0; i < 3; ++i) config->prior_ba(i) = prior_ba.at(i);

    std::vector<double> prior_bg = {0, 0, 0};
    nh.param<std::vector<double>>("prior_bg", prior_bg, prior_bg);
    for (size_t i = 0; i < 3; ++i) config->prior_bg(i) = prior_bg.at(i);

    // Read in our CAMERA noise values
    nh.param<double>("sigma_camera", config->sigma_camera, 1.0/484.1316);
    config->sigma_camera_sq = std::pow(config->sigma_camera, 2);

    // Read in our IMU noise values
    nh.param<double>("accelerometer_noise_density", config->sigma_a, 0.01);
    config->sigma_a_sq = std::pow(config->sigma_a, 2);
    nh.param<double>("gyroscope_noise_density", config->sigma_g, 0.005);
    config->sigma_g_sq = std::pow(config->sigma_g, 2);
    nh.param<double>("accelerometer_random_walk", config->sigma_wa, 0.002);
    config->sigma_wa_sq = std::pow(config->sigma_wa, 2);
    nh.param<double>("gyroscope_random_walk", config->sigma_wg, 4.0e-06);
    config->sigma_wg_sq = std::pow(config->sigma_wg, 2);
    
    // Read in our Initialization noise values
    nh.param<double>("sigma_prior_rotation",    config->sigma_prior_rotation, 1.0e-4);
    nh.param<double>("sigma_prior_translation", config->sigma_prior_translation, 1.0e-4);
    nh.param<double>("sigma_velocity",          config->sigma_velocity, 1.0e-4);
    nh.param<double>("sigma_bias",              config->sigma_bias, 1.0e-4);
    nh.param<double>("sigma_pose_rotation",     config->sigma_pose_rotation, 1.0e-4);
    nh.param<double>("sigma_pose_translation",  config->sigma_pose_translation, 1.0e-4);

    // Debug print to screen for the user
    Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
    std::cout << "\t- fixed ID: "<< config->fixedId << std::endl;
    std::cout << "\t- gravity: " << config->gravity.format(CommaInitFmt) << std::endl;
    std::cout << "\t- imuWait: " << config->imuWait << std::endl;
    std::cout << "\t- featWait: "<< config->featWait << std::endl;
    std::cout << "\t- R_C0toI: " << std::endl << config->R_C0toI << std::endl;
    std::cout << "\t- p_IinC0: " << std::endl << config->p_IinC0.transpose() << std::endl;
 
    std::cout << "Initialization:" << std::endl;
    std::cout << "\t- prior_q_GtoI: " << std::endl << config->prior_qGtoI.transpose() << std::endl;
    std::cout << "\t- prior_p_IinG: " << std::endl << config->prior_pIinG.transpose() << std::endl;
    std::cout << "\t- prior_ba: "     << std::endl << config->prior_ba.transpose() << std::endl;
    std::cout << "\t- prior_bg: "     << std::endl << config->prior_bg.transpose() << std::endl;
   
    std::cout << "Noise Parameters:" << std::endl;
    std::cout << "\t- sigma_camera:  " << config->sigma_camera << std::endl;
    std::cout << "\t- sigma_a:  " << config->sigma_a << std::endl;
    std::cout << "\t- sigma_g:  " << config->sigma_g << std::endl;
    std::cout << "\t- sigma_wa: " << config->sigma_wa << std::endl;
    std::cout << "\t- sigma_wg: " << config->sigma_wg << std::endl;
    std::cout << "\t- sigma_prior_rotation: "    << config->sigma_prior_rotation << std::endl;
    std::cout << "\t- sigma_prior_translation: " << config->sigma_prior_translation << std::endl;
    std::cout << "\t- sigma_velocity: "          << config->sigma_velocity << std::endl;
    std::cout << "\t- sigma_bias: "              << config->sigma_bias << std::endl;
    std::cout << "\t- sigma_pose_rotation: "     << config->sigma_pose_rotation << std::endl;
    std::cout << "\t- sigma_pose_translation: "  << config->sigma_pose_translation << std::endl;
}

void setup_subpub(ros::NodeHandle& nh) {
 
    // Subscribe to IMU measurements
    subIMUMeas = nh.subscribe("vio/data_imu", 2000, handle_measurement_imu);
    ROS_INFO("Subscribing: %s", subIMUMeas.getTopic().c_str());

    // Subscribe to uv measurements
    subUVMeas = nh.subscribe("vio/data_uv", 500, handle_measurement_uv);
    ROS_INFO("Subscribing: %s", subUVMeas.getTopic().c_str());
   
    // IMU pose visualization
    pubPoseIMU = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("vio/pose_imu", 2);
    ROS_INFO("Publishing: %s", pubPoseIMU.getTopic().c_str());

    // IMU path visualization
    pubPathIMU = nh.advertise<nav_msgs::Path>("vio/path_imu", 2);
    ROS_INFO("Publishing: %s", pubPathIMU.getTopic().c_str());

    // Feature cloud visualization
    pubFeatureCloud = nh.advertise<sensor_msgs::PointCloud2>("vio/feature_cloud", 2);
    ROS_INFO("Publishing: %s", pubFeatureCloud.getTopic().c_str());

}

void handle_measurement_imu(const sensor_msgs::ImuConstPtr& msg) {

    // Convert to eigen format
    Eigen::Vector3d linearacceleration;
    linearacceleration << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    Eigen::Vector3d angularvelocity;
    angularvelocity << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    Eigen::Vector4d orientation;
    orientation << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w;

    // Send to graph solver
    graphsolver->addmeasurement_imu(msg->header.stamp.toSec(), linearacceleration, angularvelocity, orientation);
}

void handle_measurement_uv(const sensor_msgs::PointCloudConstPtr& msg) {

    // Skip first keyframes
    if (graphsolver->is_initialized() && skip != config->featWait) {
      skip++;
      return;
    } else
      skip = 0;

    // Feature measurements
    std::vector<uint> leftids;
    std::vector<Eigen::Vector2d> leftuv;

    // Loop through LEFT features and append
    for(size_t i = 0; i < msg->points.size(); ++i) {
        int v = msg->channels[0].values[i] + 0.5;
        int id = v / 1;
        leftids.push_back((uint)id);
        Eigen::Vector2d uv;
        uv << msg->points[i].x, msg->points[i].y;
        leftuv.push_back(uv);
    }

    // We have successfully handled all features, lets send them to the optimizer
    graphsolver->addmeasurement_uv(msg->header.stamp.toSec(), leftids, leftuv);
    optimize_graph(msg->header.stamp.toSec());
}

void optimize_graph(double timestamp) {
    // Optimize graph
    graphsolver->optimize();

    // Ros visualization
    gtsam::State state = graphsolver->get_current_state();
    publish_state(timestamp, state);
    std::vector<std::pair<double, gtsam::State>> trajectory = graphsolver->get_trajectory();
    publish_trajectory(timestamp, trajectory);
    publish_cloud(timestamp);
}

void publish_state(double timestamp, gtsam::State& state) {

    // Return if we have not initialized yet
    if(!graphsolver->is_initialized())
        return;

    // Create our stamped pose with covariance
    geometry_msgs::PoseWithCovarianceStamped pose;
    pose.header.stamp = ros::Time(timestamp);
    pose.header.frame_id = config->fixedId;
    Eigen::Matrix<double, 6, 6> covariance = Eigen::Matrix<double,6,6>::Zero();
    ToPoseWithCovariance(state.pose(), covariance, pose.pose);
    
    // Publish this pose
    pubPoseIMU.publish(pose);
}

void publish_trajectory(double timestamp, Trajectory& trajectory) {

  // Return if trajectory is empty
  if (trajectory.empty())
    return;

  // Create stamped pose for path publishing
  std::vector<geometry_msgs::PoseStamped> traj_est;
  for (auto it = trajectory.begin(); it != trajectory.end(); ++it) {
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time(it->first);
    poseStamped.header.frame_id = config->fixedId;
    ToPose(it->second.pose(), poseStamped.pose);
    traj_est.push_back(poseStamped);
  }
  
  // Create pose arrays and publish
  nav_msgs::Path patharr;
  patharr.header.frame_id = config->fixedId;
  patharr.header.stamp = ros::Time(timestamp);
  patharr.header.seq = poses_seq++;
  patharr.poses = traj_est;
  pubPathIMU.publish(patharr);
}

void publish_cloud(double timestamp) {

    // Return if we have not initialized yet
    if (!graphsolver->is_initialized())
        return;

    // Loop through and create a point cloud
    std::vector<Eigen::Vector3d> points = graphsolver->get_current_features();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZ>);
    for (size_t i = 0; i < points.size(); ++i) {
        pcl::PointXYZ pt;
        pt.x = points.at(i)(0);
        pt.y = points.at(i)(1);
        pt.z = points.at(i)(2);
        cloud->push_back(pt);
    }

    // publish as a ros msg
    sensor_msgs::PointCloud2 msgOut;
    pcl::toROSMsg(*cloud, msgOut);
    msgOut.header.frame_id = config->fixedId;
    msgOut.header.stamp = ros::Time(timestamp);
    pubFeatureCloud.publish(msgOut);
}
