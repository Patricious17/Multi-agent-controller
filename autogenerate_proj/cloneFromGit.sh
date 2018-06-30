#create local git repo
rm -r ~/dev/gitRepo
mkdir ~/dev/gitRepo
cd ~/dev/gitRepo

#clone UTARI demo
git clone -b master --single-branch git@gitlab.com:patrik17/Cooperative_UAV_formation.git
cp -a ~/dev/gitRepo/consensus_control/* ~/dev/CWS/src/

#clone joystick drivers and copy joy package 
cd ~/dev/gitRepo
git clone https://github.com/ros-drivers/joystick_drivers.git
cp -a ~/dev/gitRepo/joystick_drivers/joy ~/dev/CWS/src

#clone vicon
cd ~/dev/CWS/src
git clone https://github.com/ethz-asl/vicon_bridge.git

