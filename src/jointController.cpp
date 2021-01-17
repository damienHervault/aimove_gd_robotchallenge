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

#define USE_KINECT

#include "plane_detection.h"



float pos[3] = {};
float camera2table[9] = {};



//Convert from ref camera to ref Table
void toTable(const float pos[3], float out[3]){
  out[0] = camera2table[0]*pos[0] + camera2table[3]*pos[1] + camera2table[6]*pos[2];
  out[1] = camera2table[1]*pos[0] + camera2table[4]*pos[1] + camera2table[7]*pos[2];
  out[2] = camera2table[2]*pos[0] + camera2table[5]*pos[1] + camera2table[8]*pos[2];
}

void callbackRosOpenpose(const ros_openpose::Frame msg){
  //ROS_INFO pour communiquer avec classe dans le cmd
  //ROS_INFO("%f", msg.persons[0].bodyParts[4].point.z);
  pos[0] = msg.persons[0].bodyParts[4].point.x;
  pos[1] = msg.persons[0].bodyParts[4].point.y;
  pos[2] = msg.persons[0].bodyParts[4].point.z;
  //bodyParts[4] correspond au poignet droit
  //point correspond au coordonnées 3D du bodyPart
}

void calibrate(std::shared_ptr<CameraReader> readers) {

  ros::Rate loopRate(10);
  bool calibrated = false;
  while (ros::ok()  && !calibrated)
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
        float norm = sqrtf((powf(camera2table[0],2)+powf(camera2table[3],2)+powf(camera2table[6],2)));
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

int setValeurPoint(trajectory_msgs::JointTrajectory* trajectoire,int pos_tab, int val);

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
std::cout<<depthTopic<<std::endl;
  const auto cameraReader = std::make_shared<CameraReader>(nh, colorTopic, depthTopic, camInfoTopic);
  calibrate(cameraReader);
#if 1
  //ros::AsyncSpinner spinner(1);
  //spinner.start();


  //frame est le nom du rostopic dans lequelle rosOpenpose publie son msg
  ros::Subscriber counter1_sub = nh.subscribe("frame", 10, callbackRosOpenpose);

  arm_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command",1);

  ros::Rate loop_rate(10);



  trajectory_msgs::JointTrajectory traj;

  traj.header.frame_id = "base_link";
  traj.joint_names.resize(6);
  traj.points.resize(1);

  traj.points[0].positions.resize(6);

  traj.joint_names[0] = "shoulder_pan_joint";
  traj.joint_names[1] = "shoulder_lift_joint";
  traj.joint_names[2] = "elbow_joint";
  traj.joint_names[3] = "wrist_1_joint";
  traj.joint_names[4] = "wrist_2_joint";
  traj.joint_names[5] = "wrist_3_joint";


  while (ros::ok()){
    traj.header.stamp = ros::Time::now();
    traj.points[0].positions[0] += 0.05;
    traj.points[0].positions[1] = 0.0;
    traj.points[0].positions[2] = 0.0;
    traj.points[0].positions[3] = 0.0;
    traj.points[0].positions[4] = 0.0;
    traj.points[0].positions[5] = 0.0;

    traj.points[0].time_from_start = ros::Duration(1);

    arm_pub.publish(traj);
    ros::spinOnce();

    loop_rate.sleep();
	}
#endif
}
