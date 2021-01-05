#include <armController.hpp>
#include <cameraReader.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "settings.h"
#include "quadtree_t.h"
#include "kernel_t.h"
#include "plane_t.h"
#include "accumulatorball_t.h"
#include "hough.h"

void callbackRosOpenpose(const ros_openpose::Frame msg){
  //ROS_INFO pour communiquer avec classe dans le cmd
  ROS_INFO("%f", msg.persons[0].bodyParts[4].point.z);
  //bodyParts[4] correspond au poignet droit
  //point correspond au coordonnées 3D du bodyPart
}

void calibrate(std::shared_ptr<CameraReader> readers) {
  ros::Rate loopRate(10);
  bool calibrated = false;
  while (!calibrated)
  {
    auto depthImage = readers->getDepthFrame();
    if (!depthImage.empty()) {
      calibrated = true;
      cv::imshow("depth image", depthImage);

      // Plane computation
      hough_settings settings;
      quadtree_t father;
      //load_input(settings, father);
      //father.compute_centroid();
      std::vector<plane_t> planes_out;
      std::vector<kernel_t> used_kernels;
      //accumulatorball_t *accum = kht3d(planes_out, father, settings, used_kernels);
      std::cout << planes_out.size() << " PLANES FOUND." << endl;;
    }
    else
      ROS_WARN_THROTTLE(10, "Empty depth image frame detected. Waiting...");
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
  ros::init(argc, argv, "armController : au boulot les glandeurs !!");
  ros::NodeHandle nh;

  std::cout<<"Calibration"<<std::endl;
#if TRUE
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
  const auto cameraReader = std::make_shared<CameraReader>(nh, colorTopic, depthTopic, camInfoTopic);
  calibrate(cameraReader);

  //frame est le nom du rostopic dans lequelle rosOpenpose publie son msg
  ros::Subscriber counter1_sub = nh.subscribe("frame", 10, callbackRosOpenpose);

  //lets all the callbacks get called for your subscribers
  ros::spin();
}
