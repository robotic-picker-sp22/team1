function setrobot() {
  if [ "$1" = "sim" ]; then
    export ROS_HOSTNAME=localhost;
    export ROS_MASTER_HOST=localhost;
    export ROS_MASTER_URI=http://localhost:11311;
    export ROBOT=sim;
    export CATKIN_SOURCE_PATH=/home/capstone/gazebo_ws/devel/setup.bash
    source $CATKIN_SOURCE_PATH
  else
    unset ROBOT;
    unset ROS_HOSTNAME;
    export ROS_MASTER_HOST=$1;
    export ROS_MASTER_URI=http://$1.cs.washington.edu:11311;
    export ROS_IP=10.158.134.71;
    export CATKIN_SOURCE_PATH=/home/capstone/catkin_ws/devel/setup.bash
    source $CATKIN_SOURCE_PATH
  fi
}

setrobot astro-1
rosrun perception save_cloud tags-7
mv tags-7.bag ~/data/
setrobot sim
rosrun applications publish_saved_cloud.py /home/capstone/data/tags-7.bag