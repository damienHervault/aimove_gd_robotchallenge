#include <jointController.hpp>
#include <cameraReader.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <ros/ros.h>
#include "ros/time.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <math.h>

#include <ur_kinematics/ur_kin.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>



#define USE_KINECT

#include "plane_detection.h"



double pos_wrist[3] = {};
double pos_elbow[3] = {};
double jointAngles[6] = {};
double camera2table[9] = {};




//Convert from ref camera to ref Table
void toTable(const double pos[3], double out[3]){
  out[0] = camera2table[0]*pos[0] + camera2table[3]*pos[1] + camera2table[6]*pos[2];
  out[1] = camera2table[1]*pos[0] + camera2table[4]*pos[1] + camera2table[7]*pos[2];
  out[2] = camera2table[2]*pos[0] + camera2table[5]*pos[1] + camera2table[8]*pos[2];
}

void callbackJointAngles(const control_msgs::JointTrajectoryControllerState& traj ){
  for (int i=0; i<6; i++){
    jointAngles[i] = traj.actual.positions[i];
  }
}

void callbackRosOpenpose(const ros_openpose::Frame msg){
  //ROS_INFO pour communiquer avec classe dans le cmd
  //ROS_INFO("%f", msg.persons[0].bodyParts[4].point.z);
  if (msg.persons.size() > 0) {
    pos_wrist[0] = msg.persons[0].bodyParts[4].point.x;
    pos_wrist[1] = msg.persons[0].bodyParts[4].point.y;
    pos_wrist[2] = msg.persons[0].bodyParts[4].point.z;
    pos_elbow[0] = msg.persons[0].bodyParts[3].point.x;
    pos_elbow[1] = msg.persons[0].bodyParts[3].point.y;
    pos_elbow[2] = msg.persons[0].bodyParts[3].point.z;

  }
  else
    pos_wrist[0] = nanf("0");
  //bodyParts[4] correspond au poignet droit
  //point correspond au coordonnées 3D du bodyPart
}

void calibrate(std::shared_ptr<CameraReader> readers) {

  ros::Rate loopRate(10);
  bool calibrated = false;
  while (ros::ok() && !calibrated)
  {
    auto colorImage = readers->getColorFrame();
    auto depthImage = readers->getDepthFrame();
    if (!depthImage.empty() && !colorImage.empty()) {


      depthImage.convertTo(depthImage, CV_16U, 55535);

      PlaneDetection plane_detection;
      plane_detection.readDepthImage(depthImage);
    	plane_detection.readColorImage(colorImage);
    	plane_detection.runPlaneDetection();
      cv::imshow("depthImage", depthImage);
      cv::imshow("Seg image", plane_detection.seg_img_);

      plane_detection.computePlaneSumStats(false);
      int i, max = -1, idMax = -1;
      for (i=0; i<plane_detection.plane_pixel_nums_.size(); i++) {
        if (plane_detection.plane_pixel_nums_[i] > max) {
          max = plane_detection.plane_pixel_nums_[i];
          idMax = i;
        }
      }

      std::cout<<"number of planes detected : "<<plane_detection.plane_num_<<std::endl;
      if (idMax >= 0) {
        std::cout << "biggest plane index : " << idMax << std::endl;
        std::cout << "biggest plane size  : " << max << std::endl;
        int vidx = plane_detection.plane_vertices_[idMax][0];
        cv::Vec3b c = plane_detection.seg_img_.at<cv::Vec3b>(vidx / kDepthWidth, vidx % kDepthWidth);
        std::cout << "biggest plane color : "
          << int(c.val[2]) << " "
          << int(c.val[1]) << " "
          << int(c.val[0]) << " " << std::endl; // OpenCV uses BGR by default
        std::cout << "biggest plane normal : "
          << plane_detection.plane_filter.extractedPlanes[idMax]->normal[0] << " "
          << plane_detection.plane_filter.extractedPlanes[idMax]->normal[1] << " "
          << plane_detection.plane_filter.extractedPlanes[idMax]->normal[2] << " " << std::endl;
        std::cout << "biggest plane center : "
          << plane_detection.plane_filter.extractedPlanes[idMax]->center[0] << " "
          << plane_detection.plane_filter.extractedPlanes[idMax]->center[1] << " "
          << plane_detection.plane_filter.extractedPlanes[idMax]->center[2] << " " << std::endl;

        //Vecteur Z
        camera2table[2] = plane_detection.plane_filter.extractedPlanes[idMax]->normal[0];
        camera2table[5] = plane_detection.plane_filter.extractedPlanes[idMax]->normal[1];
        camera2table[8] = plane_detection.plane_filter.extractedPlanes[idMax]->normal[2];

        // VEcteur X
        camera2table[0] = camera2table[5];
        camera2table[3] = camera2table[2];
        camera2table[6] = - 2.0*camera2table[0]*camera2table[3]/camera2table[8];
        double norm = sqrtf((powf(camera2table[0],2)+powf(camera2table[3],2)+powf(camera2table[6],2)));
        camera2table[0] /= norm;
        camera2table[3] /= norm;
        camera2table[6] /= norm;

        //Vecteur Y
        camera2table[1] = camera2table[5]*camera2table[6] - camera2table[8]*camera2table[3];
        camera2table[4] = camera2table[8]*camera2table[0] - camera2table[2]*camera2table[6];
        camera2table[7] = camera2table[2]*camera2table[3] - camera2table[5]*camera2table[0];

        calibrated = true;
      }
    }
    else
      // display the error at most once per 10 seconds
      ROS_WARN_THROTTLE(10, "Empty depth image frame detected. Ignoring...");
    int key = cv::waitKey(1) & 255;
    if (key == 27)  // escape key
      break;

    ros::spinOnce();
    loopRate.sleep();
  }
}

/*
  TODO list
  - DONE :
  -- dump des logs dans la console
  -- écouter le node ros_openpose
  -- donner des commandes à moveit
  -- ajouter des objets dans gazebo
  -- calcul des coordonées de la table dans le repère camera
  -
  - SHORT TERM Plan
  -- calculer coordonées de la main dans le referentiel camera
  -- envoyer entre chaque pas de temps l'ordre de movement au robot identique au delta de position de la main
  -- ajout du squelette humain
  -- position realtime des joints du squelette et de la table
  -
  - TODO
  Wk1 -- ajouter objets 3D pour la camera et le HUMAN squelette 3D
  Wk1 -- donner la position d'objets dans gazebo
  Wk1 -- calculer la transformation repère camera <> repère table
  -- add the gripper to the robot
  -- opt. : choisir la logique pour trouver un point précis sur la table
  ? -- calculer la transformation repère camera <> repère robot
  -- optimiser le path planning de moveit
*/

ros::Publisher arm_pub;
ros::Subscriber joint_sub;


int main(int argc, char* argv[]) {

  ros::init(argc, argv, "Joint State Publisher");
  ros::NodeHandle nh;

  std::cout<<"Calibration"<<std::endl;
#ifdef USE_KINECT
  // kinect
  std::string colorTopic   = "/kinect2/sd/image_color_rect";
  std::string depthTopic   = "/kinect2/sd/image_depth";
  std::string camInfoTopic = "/kinect2/sd/camera_info";
#else
  // realsense
  std::string colorTopic   = "/camera/color/image_raw";
  std::string depthTopic   = "/camera/aligned_depth_to_color/image_raw";
  std::string camInfoTopic = "/camera/color/camera_info";
#endif
//std::cout<<depthTopic<<std::endl;
  const auto cameraReader = std::make_shared<CameraReader>(nh, colorTopic, depthTopic, camInfoTopic);
  calibrate(cameraReader);
#if 1
  ros::AsyncSpinner spinner(1);
  spinner.start();


  //frame est le nom du rostopic dans lequelle rosOpenpose publie son msg
  ros::Subscriber counter1_sub = nh.subscribe("frame", 10, callbackRosOpenpose);

  arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command",1);
  joint_sub = nh.subscribe("/arm_controller/state", 10, callbackJointAngles);


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("manipulator");
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();






  trajectory_msgs::JointTrajectory traj;

  traj.header.frame_id = "base_link";
  traj.joint_names.resize(6);
  traj.points.resize(1);

  traj.points[0].positions.resize(6);
  //Time to get the current angle Values
  ros::Duration(2.0).sleep();
  kinematic_state->setJointGroupPositions(joint_model_group, jointAngles);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    traj.joint_names[i] = joint_names[i].c_str();
    traj.points[0].positions[i] = jointAngles[i];
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), traj.points[0].positions[i]);
  }

  //Weights to avoid oscillations
  double weights[6] = {0.15,0.15,0.15,0.05,0.01,0.01};
  double old_pos_wrist[3] = {};
  double diff_wrist[3] = {};
  double tableDiff_wrist[3] = {};
  double wrist_pos[3] = {};
  //double filtered_wrist_pos[3] = {}
  double wrist_pos_error[3] = {};

  double dir_wrist_elbow[3] = {};
  double tableDiff_dir_wrist_elbow[3] = {};



  bool first_step = true;
  while (ros::ok()){

    if (!isnan(pos_wrist[0])) {
      double q[6] = {traj.points[0].positions[0],traj.points[0].positions[1],
                     traj.points[0].positions[2],traj.points[0].positions[3],
                     traj.points[0].positions[4],traj.points[0].positions[5]};
      double* T = new double[16];
      ur_kinematics::forward(q, T);


      //{0.2,0.2,0.5};

      double ee_pos[3] = {T[3],T[7],T[11]};

      //Maybe We'll need to use a Mutex
      diff_wrist[0] = pos_wrist[0] - old_pos_wrist[0];
      diff_wrist[1] = pos_wrist[1] - old_pos_wrist[1];
      diff_wrist[2] = pos_wrist[2] - old_pos_wrist[2];
      toTable(diff_wrist, tableDiff_wrist);


      dir_wrist_elbow[0] = pos_wrist[0] - pos_elbow[0];
      dir_wrist_elbow[1] = pos_wrist[1] - pos_elbow[1];
      dir_wrist_elbow[2] = pos_wrist[2] - pos_elbow[2];
      toTable(dir_wrist_elbow, tableDiff_dir_wrist_elbow);

      double norm_wrist_elbow = sqrtf((powf(tableDiff_dir_wrist_elbow[0],2)+powf(tableDiff_dir_wrist_elbow[1],2)+powf(tableDiff_dir_wrist_elbow[2],2)));

      tableDiff_dir_wrist_elbow[0] = tableDiff_dir_wrist_elbow[0]/norm_wrist_elbow;
      tableDiff_dir_wrist_elbow[1] = tableDiff_dir_wrist_elbow[1]/norm_wrist_elbow;
      tableDiff_dir_wrist_elbow[2] = tableDiff_dir_wrist_elbow[2]/norm_wrist_elbow;
      //std::cout<<"dir_wrist_elbow[0]"<<int(tableDiff_dir_wrist_elbow[0]*100)/100.0<<" ; dir_wrist_elbow[1]"<<int(tableDiff_dir_wrist_elbow[1]*100)/100.<<" ; dir_wrist_elbow[2]"<<int(tableDiff_dir_wrist_elbow[2]*100)/100.<<std::endl;

      //+1 Step
      old_pos_wrist[0] = pos_wrist[0];
      old_pos_wrist[1] = pos_wrist[1];
      old_pos_wrist[2] = pos_wrist[2];





      if (first_step){
        wrist_pos[0] = ee_pos[0];
        wrist_pos[1] = ee_pos[1];
        wrist_pos[2] = ee_pos[2];
        first_step = false;
      }else{
        wrist_pos[0] += tableDiff_wrist[0];
        wrist_pos[1] += tableDiff_wrist[1];
        wrist_pos[2] += tableDiff_wrist[2];
        //Need to implement a PID to erase noise
      }

      wrist_pos_error[0] =  wrist_pos[0] - ee_pos[0];
      wrist_pos_error[1] =  wrist_pos[1] - ee_pos[1];
      wrist_pos_error[2] =  wrist_pos[2] - ee_pos[2];
      //std::cout<<"pos_error[0]"<<pos_error[0]<<"pos_error[1]"<<pos_error[1]<<"pos_error[2]"<<pos_error[2]<<std::endl;




      Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
      Eigen::MatrixXd jacobian;
      kinematic_state->getJacobian(joint_model_group,
                                 kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                 reference_point_position, jacobian);


      double delta_q[6] = {};
      for (int i = 0; i<6; i++){
          delta_q[i] = jacobian(i*6)*wrist_pos_error[0]+jacobian(i*6+1)*wrist_pos_error[1]+jacobian(i*6+2)*wrist_pos_error[2];
      }

      traj.header.stamp = ros::Time::now();
      for (int i=0; i<6; i++){
        traj.points[0].positions[i] += weights[i]*delta_q[i];
      }
      //Maybe Useless but need to check after direct control
      kinematic_state->setJointGroupPositions(joint_model_group, traj.points[0].positions);

      traj.points[0].time_from_start = ros::Duration(1);

      arm_pub.publish(traj);
    }
    ros::spinOnce();

	}
#endif
}
