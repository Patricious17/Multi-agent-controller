if [ -d "$HOME/dev/CWS" ]; then 
  echo "yo"
  sudo rm -r $HOME/dev/CWS
fi

source /opt/ros/kinetic/setup.bash
mkdir -p  $HOME/dev/CWS/src && cd CWS && catkin_make

