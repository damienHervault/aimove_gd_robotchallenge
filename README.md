# aimove_gd_robotchallenge
Robotic arm realtime control from depth camera


* https://github.com/CMU-Perceptual-Computing-Lab/openpose
* http://wiki.ros.org/melodic/Installation/Ubuntu
* https://github.com/ros-industrial/universal_robot
* https://github.com/ravijo/ros_openpose
* http://docs.ros.org/en/melodic/api/moveit_tutorials/html/index.html

* https://www.visgraf.impa.br/Data/RefBib/PS_PDF/prl2018/Vera2018.pdf
* https://github.com/eduardovera/D-KHT


* https://hal-mines-paristech.archives-ouvertes.fr/hal-02276236/document
* https://medium.com/cse-468-568-robotic-algorithms/project-proposal-31aa5f095857

* https://github.com/msr-peng/openpose_ros_visualization
* https://github.com/Seri-Lee/robotis_op3_following_motion
* https://www.universal-robots.com/articles/ur/parameters-for-calculations-of-kinematics-and-dynamics/
* https://github.com/robotology/human-gazebo


roslaunch ros_openpose run.launch
roslaunch ur3_moveit_config ur3_moveit_planning_execution.launch sim:=true
roslaunch ur_gazebo ur3.launch
roslaunch aimove_gd_robotchallenge joint_run.launch

replace in universal_robot/ur_gazebo/launch/ur3.launch
'''
<arg name="world_name" default="$(find aimove_gd_robotchallenge)/worlds/simulation.world"/>
'''
