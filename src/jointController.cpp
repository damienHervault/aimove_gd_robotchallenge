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
#include <ctime>

#include <ur_kinematics/ur_kin.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>

#include <chrono>

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
  bool planeCalibrated = false;
  bool done = false;
  time_t calibrationTime;
  PlaneDetection plane_detection;
  while (ros::ok() && !done)
  {
    if (planeCalibrated) {
      cv::imshow("Seg image", plane_detection.seg_img_);
      float left = 5 - (time(NULL) - calibrationTime); // Wait 10 seconds before start
      if (left < 0.) {
        double arm_axis[3] = {};
        arm_axis[0] = pos_wrist[0] - pos_elbow[0];
        arm_axis[1] = pos_wrist[1] - pos_elbow[1];
        arm_axis[2] = pos_wrist[2] - pos_elbow[2];

        /*std::cout << "camera2table : " << camera2table[2] << " " << camera2table[5] << " " << camera2table[8] << " " << std::endl;
        std::cout << "pos_wrist : " << pos_wrist[0] << " " << pos_wrist[1] << " " << pos_wrist[2] << " " << std::endl;
        std::cout << "pos_elbow : " << pos_elbow[0] << " " << pos_elbow[1] << " " << pos_elbow[2] << " " << std::endl;
        std::cout << "arm_axis : " << arm_axis[0] << " " << arm_axis[1] << " " << arm_axis[2] << " " << std::endl;*/

        // project arm axis on plane
        double proj = arm_axis[0] * camera2table[2] + arm_axis[1] * camera2table[5] + arm_axis[2] * camera2table[8];
        arm_axis[0] -= proj * camera2table[2];
        arm_axis[1] -= proj * camera2table[5];
        arm_axis[2] -= proj * camera2table[6];
        double norm = sqrtf(arm_axis[0] * arm_axis[0] + arm_axis[1] * arm_axis[1] + arm_axis[2] * arm_axis[2]);

        // Table X vector
        camera2table[0] = arm_axis[0] / norm;
        camera2table[3] = arm_axis[1] / norm;
        camera2table[6] = arm_axis[2] / norm;
        //std::cout << "arm_axis : " << camera2table[0] << " " << camera2table[3] << " " << camera2table[6] << " " << std::endl;

        // Table Y vector
        camera2table[1] = camera2table[5]*camera2table[6] - camera2table[8]*camera2table[3];
        camera2table[4] = camera2table[8]*camera2table[0] - camera2table[2]*camera2table[6];
        camera2table[7] = camera2table[2]*camera2table[3] - camera2table[5]*camera2table[0];

        done = true;
      }
      else
        std::cout << "Your arm's position will be scanned in " << left << " seconds, say in init position" << std::endl;
    }
    else {
      auto colorImage = readers->getColorFrame();
      auto depthImage = readers->getDepthFrame();
      if (!depthImage.empty() && !colorImage.empty()) {
        depthImage.convertTo(depthImage, CV_16U, 55535);

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
          /*std::cout << "biggest plane index : " << idMax << std::endl;
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
            << plane_detection.plane_filter.extractedPlanes[idMax]->center[2] << " " << std::endl;*/

          //Vecteur Z
          camera2table[2] = plane_detection.plane_filter.extractedPlanes[idMax]->normal[0];
          camera2table[5] = plane_detection.plane_filter.extractedPlanes[idMax]->normal[1];
          camera2table[8] = plane_detection.plane_filter.extractedPlanes[idMax]->normal[2];

          planeCalibrated = true;
          calibrationTime = time(NULL);
        }
      }
      else
        // display the error at most once per 10 seconds
        ROS_WARN_THROTTLE(10, "Empty depth image frame detected. Ignoring...");
    }

  /*int key = cv::waitKey(1) & 255;
    if (key == 27)  // escape key
      break;
      */
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

  using std::chrono::high_resolution_clock;
  using std::chrono::duration;

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
  //frame est le nom du rostopic dans lequelle rosOpenpose publie son msg
  ros::Subscriber counter1_sub = nh.subscribe("frame", 10, callbackRosOpenpose);

  const auto cameraReader = std::make_shared<CameraReader>(nh, colorTopic, depthTopic, camInfoTopic);
  calibrate(cameraReader);
#if 1
  ros::AsyncSpinner spinner(1);
  spinner.start();



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
  Eigen::VectorXd weights(6);
  weights << 0.15,0.15,0.15,0.05,0.01,0.01;
  double old_pos_wrist[3] = {};
  double diff_wrist[3] = {};
  double tableDiff_wrist[3] = {};
  double wrist_pos[3] = {};


  int latest_in_pos = 0;
  Eigen::Vector3d wrist_pos_error;

  double dir_wrist_elbow[3] = {};
  double tableDiff_dir_wrist_elbow[3] = {};
  Eigen::Vector3d unit_wrist_elbow;


  bool first_step = true;
  auto t1 = high_resolution_clock::now();

  // Hand point pose manager
  ros::ServiceClient set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState wrist_state_srv_msg;
  wrist_state_srv_msg.request.model_state.model_name = "wrist_point";
  wrist_state_srv_msg.request.model_state.pose.position.x = 0.0;
  wrist_state_srv_msg.request.model_state.pose.position.y = 0.0;
  wrist_state_srv_msg.request.model_state.pose.position.z = 0.0;
  wrist_state_srv_msg.request.model_state.pose.orientation.x = 0.0;
  wrist_state_srv_msg.request.model_state.pose.orientation.y = 0.0;
  wrist_state_srv_msg.request.model_state.pose.orientation.z = 0.0;
  wrist_state_srv_msg.request.model_state.pose.orientation.w = 1.0;
  wrist_state_srv_msg.request.model_state.reference_frame = "world";
  gazebo_msgs::SetModelState elbow_state_srv_msg;
  elbow_state_srv_msg.request.model_state.model_name = "elbow_point";
  elbow_state_srv_msg.request.model_state.pose.position.x = 0.0;
  elbow_state_srv_msg.request.model_state.pose.position.y = 0.0;
  elbow_state_srv_msg.request.model_state.pose.position.z = 0.0;
  elbow_state_srv_msg.request.model_state.pose.orientation.x = 0.0;
  elbow_state_srv_msg.request.model_state.pose.orientation.y = 0.0;
  elbow_state_srv_msg.request.model_state.pose.orientation.z = 0.0;
  elbow_state_srv_msg.request.model_state.pose.orientation.w = 1.0;
  elbow_state_srv_msg.request.model_state.reference_frame = "world";

  while (ros::ok()){

    if (!isnan(pos_wrist[0]) && !isnan(pos_elbow[0])) {
      // Set helper points pose
      wrist_state_srv_msg.request.model_state.pose.position.x = pos_wrist[0];
      wrist_state_srv_msg.request.model_state.pose.position.y = pos_wrist[1];
      wrist_state_srv_msg.request.model_state.pose.position.z = pos_wrist[2];
      set_model_state_client.call(wrist_state_srv_msg);
      elbow_state_srv_msg.request.model_state.pose.position.x = pos_elbow[0];
      elbow_state_srv_msg.request.model_state.pose.position.y = pos_elbow[1];
      elbow_state_srv_msg.request.model_state.pose.position.z = pos_elbow[2];
      set_model_state_client.call(elbow_state_srv_msg);

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

      unit_wrist_elbow(0) = tableDiff_dir_wrist_elbow[0]/norm_wrist_elbow;
      unit_wrist_elbow(1) = tableDiff_dir_wrist_elbow[1]/norm_wrist_elbow;
      unit_wrist_elbow(2) = tableDiff_dir_wrist_elbow[2]/norm_wrist_elbow;
      //std::cout<<"dir_wrist_elbow[0]"<<int(unit_wrist_elbow(0)*100)/100.0<<" ; dir_wrist_elbow[1]"<<int(unit_wrist_elbow(1)*100)/100.<<" ; dir_wrist_elbow[2]"<<int(unit_wrist_elbow(2)*100)/100.<<std::endl;

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

      wrist_pos_error(0) =  wrist_pos[0] - ee_pos[0];
      wrist_pos_error(1) =  wrist_pos[1] - ee_pos[1];
      wrist_pos_error(2) =  wrist_pos[2] - ee_pos[2];
      //std::cout<<"pos_error[0]"<<pos_error[0]<<"pos_error[1]"<<pos_error[1]<<"pos_error[2]"<<pos_error[2]<<std::endl;




      Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
      Eigen::MatrixXd jacobian;
      kinematic_state->getJacobian(joint_model_group,
                                 kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                 reference_point_position, jacobian);




      Eigen::MatrixXd JPosition(3,6);//There is a function to do that best using Eigen
      Eigen::MatrixXd JOrientation(3,6);
      for (int i = 0; i<3; i++){
        for (int j = 0; j<6; j++){
          JPosition(i,j) = jacobian(i,j);
          JOrientation(i,j) = jacobian(i+3,j);
        }
      }
      Eigen::MatrixXd JJTPosition = JPosition * JPosition.transpose();
      Eigen::MatrixXd pinvJPosition = JPosition.transpose() * JJTPosition.inverse();
      Eigen::MatrixXd JJTOrientation = JOrientation * JOrientation.transpose();
      Eigen::MatrixXd pinvJOrientation = JOrientation.transpose() * JJTOrientation.inverse();


      Eigen::Matrix3d ee_orientation(3,3);
      for (int i = 0; i<3; i++){
        for (int j = 0; j<3; j++){
          ee_orientation(i,j) = T[j +i*4];

        }
      }
      //std::cout<<"ee_orientation : "<<ee_orientation<<std::endl;
      Eigen::Vector3d u;
      u(0) = T[0];
      u(1) = T[4];
      u(2) = T[8];
      //std::cout<<"u[0]"<<int(u(0)*100)/100.0<<" ; u[1]"<<int(u(1)*100)/100.<<" ; u[2]"<<int(u(2)*100)/100.<<std::endl;



      Eigen::Vector3d n = u.cross(unit_wrist_elbow);
      double alpha = acos(u.dot(unit_wrist_elbow));
      //std::cout<<"u : \n"<<u<<std::endl;
      //std::cout<<"v : \n"<<unit_wrist_elbow<<std::endl;
      //std::cout<<"alpha : \n"<<alpha<<std::endl;
      //std::cout<<"n : \n"<<n<<std::endl;

      Eigen::Quaterniond quat_inter(cos(alpha/2), n(0)*sin(alpha/2), n(1)*sin(alpha/2), n(2)*sin(alpha/2));
      Eigen::Quaterniond quat_robot;
      quat_robot = ee_orientation;
      Eigen::Matrix4d quat_inter_matrix;
      quat_inter_matrix << quat_inter.w(), -quat_inter.vec()(0), -quat_inter.vec()(1), -quat_inter.vec()(2),
                           quat_inter.vec()(0), quat_inter.w(), -quat_inter.vec()(2), quat_inter.vec()(1),
                           quat_inter.vec()(1), quat_inter.vec()(2), quat_inter.w(), -quat_inter.vec()(0),
                           quat_inter.vec()(2), -quat_inter.vec()(1), quat_inter.vec()(0), quat_inter.w();

      Eigen::Vector4d quat_robot_mat(quat_robot.w(), quat_robot.vec()(0),quat_robot.vec()(1),quat_robot.vec()(2));
      Eigen::VectorXd quat_des_mat = quat_inter_matrix*quat_robot_mat;
      Eigen::Quaterniond quat_des;
      quat_des.w() = quat_des_mat(0);
      quat_des.vec() <<quat_des_mat(1),quat_des_mat(2),quat_des_mat(3);
      //std::cout<<"quat_robot : \n"<<quat_robot.w()<<" \n"<<quat_robot.vec()<<std::endl;
      //std::cout<<"quat_inter : \n"<<quat_inter.w()<<" \n"<<quat_inter.vec()<<std::endl;
      //std::cout<<"quat_des : \n"<<quat_des.w()<<" \n"<<quat_des.vec()<<std::endl;
      //Eigen::Quaterniond quat_robot_conj;
      //quat_robot_conj.w() = quat_robot.w();=
      //quat_robot_conj.vec() = quat_robot.vec();
      //std::cout<<"quat_robot_conj : \n"<<quat_robot_conj.w()<<" \n"<<quat_robot_conj.vec()<<std::endl;

      //Now we go back to the Euler space to find the orientation error
      Eigen::Matrix4d quat_des_matrix;

      Eigen::Vector3d angular_error;
      Eigen::Vector4d quat_robot_mat_conj(quat_robot.w(), -quat_robot.vec()(0),-quat_robot.vec()(1),-quat_robot.vec()(2));
      Eigen::VectorXd quat_for_log = quat_des_matrix.transpose()*quat_robot_mat_conj;
      quat_for_log = quat_for_log / quat_for_log.norm();

      if(quat_des.vec().norm()<1e-6){
        quat_des_matrix.setIdentity(4,4);
      }else{
        quat_des_matrix << quat_des.w(), -quat_des.vec()(0), -quat_des.vec()(1), -quat_des.vec()(2),
                           quat_des.vec()(0), quat_des.w(), -quat_des.vec()(2), quat_des.vec()(1),
                           quat_des.vec()(1), quat_des.vec()(2), quat_des.w(), -quat_des.vec()(0),
                           quat_des.vec()(2), -quat_des.vec()(1), quat_des.vec()(0), quat_des.w();
      }
      if(abs(1.0 - quat_for_log(0))<1e-16){
        angular_error << 0.0,0.0,0.0;
      }
      else{
        double acosx;
        if(quat_for_log(0)>=1.0){
          quat_for_log(0) = 1.0;
        }
        if(quat_for_log(0)<=-1.0){
          quat_for_log(0) = -1.0;
        }
        if(quat_for_log(0)>=-1.0 and quat_for_log(0)<0.0){
           acosx = acos(quat_for_log(0)) - M_PI;
        }else{
          acosx = acos(quat_for_log(0));
        }

        double scale = acosx/sqrt(1.0-quat_for_log(0)*quat_for_log(0));
        angular_error << 2.0*quat_for_log(1)*scale, 2.0*quat_for_log(2)*scale, 2.0*quat_for_log(3)*scale;
        std::cout<<"angular_error: "<<angular_error(0)<<" ; "<<angular_error(1)<<" ; "<<angular_error(2)<<std::endl;
        //angular_error << 0.0,0.0,0.0;
      }
      //std::cout<<"angular_error : \n"<<angular_error<<std::endl;

      //Eigen::VectorXd dir_angle_error =



      // std::cout<<"T "<<std::endl;
      // for(int i=0;i<4;i++) {
      //   for(int j=i*4;j<(i+1)*4;j++)
      //     printf("%1.3f ", T[j]);
      //   printf("\n");
      //   }





      Eigen::MatrixXd identity6;
      identity6.setIdentity(6,6);
      Eigen::MatrixXd N = identity6 - pinvJPosition * JPosition;

      //std::cout<<"N : \n"<<N<<std::endl;
      auto t2 = t1;
      t1 = high_resolution_clock::now();
      duration<double, std::micro> ms_double = t1 - t2;
      double dt = ms_double.count()*1e-6;

      //std::cout<<"pinvJOrientation*angular_error : \n"<<pinvJOrientation*angular_error*dt<<std::endl;
      //std::cout<<"pinvJOrientation*angular_error : \n"dt<<std::endl;
      Eigen::VectorXd nullQ(6);
      Eigen::VectorXd phill = pinvJOrientation*angular_error;
      for (int i = 0; i < 6; i++) {
        nullQ(i) = 0.0 - traj.points[0].positions[i]; //replace 0.0 by the angle of interest
      }                                               //Could try with by default angles


      Eigen::VectorXd delta_q;
      if(isnan(angular_error(0))  or isnan(angular_error(1)) or isnan(angular_error(2))){//Why does NaN happend?
        delta_q = pinvJPosition*wrist_pos_error*dt;
      }else{
        //delta_q = pinvJPosition*wrist_pos_error*dt + N*nullQ;//To keep in second track the initial joint angles
        delta_q = pinvJPosition*wrist_pos_error*dt + N*pinvJOrientation*angular_error*dt*10.0;
      }

      traj.header.stamp = ros::Time::now();
      for (int i=0; i<6; i++){
        traj.points[0].positions[i] += delta_q(i); //weights(i)*delta_q(i);
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
