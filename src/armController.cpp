#include <armController.hpp>
#include <cameraReader.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

//#define USE_KINECT

#include "plane_detection.h"

float posX, posY, posZ;

void callbackRosOpenpose(const ros_openpose::Frame msg){
  //ROS_INFO pour communiquer avec classe dans le cmd
  //ROS_INFO("%f", msg.persons[0].bodyParts[4].point.z);
  posX = msg.persons[0].bodyParts[4].point.x;
  posY = msg.persons[0].bodyParts[4].point.y;
  posZ = msg.persons[0].bodyParts[4].point.z;
  //bodyParts[4] correspond au poignet droit
  //point correspond au coordonnées 3D du bodyPart
}

void calibrate(std::shared_ptr<CameraReader> readers) {

  ros::Rate loopRate(10);
  bool calibrated = false;
  while (ros::ok() /* && !calibrated*/)
  {
    auto colorImage = readers->getColorFrame();
    auto depthImage = readers->getDepthFrame();
    if (!depthImage.empty() && !colorImage.empty()) {
      calibrated = true;

      depthImage.convertTo(depthImage, CV_16U, 55535);

      PlaneDetection plane_detection;
      plane_detection.readDepthImage(depthImage);
    	plane_detection.readColorImage(colorImage);
    	plane_detection.runPlaneDetection();
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
  - TODO
  -- ajouter objets 3D pour la camera et le squelette 3D
  -- donner la position d'objets dans gazebo
  -- calculer la transformation repère camera <> repère table
  -- choisir la logique pour trouver un point précis sur la table
  -- calculer la transformation repère camera <> repère robot
  -- optimiser le path planning de moveit
*/

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "Starting robot challenge");
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
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //frame est le nom du rostopic dans lequelle rosOpenpose publie son msg
  ros::Subscriber counter1_sub = nh.subscribe("frame", 10, callbackRosOpenpose);

  //http://docs.ros.org/en/jade/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroup.html
  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  //std::cout << move_group.getCurrentPose();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  move_group.setGoalOrientationTolerance(0.05);
  move_group.setGoalJointTolerance(0.05);
  move_group.setGoalPositionTolerance(0.05);
  move_group.setPlanningTime(2.0); //in seconds, default 5
  //move_group.setPoseReferenceFrame(world..); By default the ref frame is the one of the robot model

  geometry_msgs::Pose target_pose1;
  while (ros::ok()){
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = 0.2;
    target_pose1.position.z = posZ-1.3;
    std::cout<<"posZ = "<<posZ<<std::endl;
    move_group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
  move_group.execute(my_plan);
  //move_group.move();
	}
#endif
}
