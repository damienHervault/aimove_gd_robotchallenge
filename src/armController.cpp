#include <armController.hpp>



void callbackRosOpenpose(const ros_openpose::Frame msg){
  //ROS_INFO pour communiquer avec classe dans le cmd
  ROS_INFO("%f", msg.persons[0].bodyParts[4].point.z);
  //bodyParts[4] correspond au poignet droit
  //point correspond au coordonnées 3D du bodyPart
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "armController : au boulot les glandeurs !!");
  ros::NodeHandle nh;

  //frame est le nom du rostopic dans lequelle rosOpenpose publie son msg
  ros::Subscriber counter1_sub = nh.subscribe("frame", 10, callbackRosOpenpose);
  //std::cout<<"hello"<<std::endl;

  //lets all the callbacks get called for your subscribers
  ros::spin();

  /*
    TODO list
    - Basics :
    -- trouver comment dupper les logs dans la console: done
    -- trouver comment écouter le node ros_openpose: done
    -- trouver comment écrire dans moveit
    - Sujet :
    -- trouver comment faire la transformation repère camera > repère table
  */
}
