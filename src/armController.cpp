#include <armController.hpp>
#include <cameraReader.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

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

PlaneDetection plane_detection;

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

      //cv::imshow("depth image", depthImage);
      std::cout<<"depthSize"<<depthImage.size()<<std::endl;

      plane_detection.readDepthImage(depthImage);
    	plane_detection.readColorImage(colorImage);
    	plane_detection.runPlaneDetection();
      std::cout<<"plane_detection.seg_img_"<<plane_detection.seg_img_.size()<<std::endl;
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
  - Basics :
  -- trouver comment dupper les logs dans la console: done
  -- trouver comment écouter le node ros_openpose: done
  -- trouver comment écrire dans moveit
  - Sujet :
  -- trouver comment faire la transformation repère camera > repère table
*/

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "Starting robot challenge");
  ros::NodeHandle nh;

  std::cout<<"Calibration"<<std::endl;
#if 0
  // realsense
  std::string colorTopic   = "/camera/color/image_raw";
  std::string depthTopic   = "/camera/aligned_depth_to_color/image_raw";
  std::string camInfoTopic = "/camera/color/camera_info";
#else
  // kinect
  std::string colorTopic   = "/kinect2/sd/image_color_rect";
  std::string depthTopic   = "/kinect2/sd/image_depth";
  std::string camInfoTopic = "/kinect2/sd/camera_info";
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
