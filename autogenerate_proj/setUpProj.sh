cd ~/dev
chmod 700 createCWS.sh
chmod 700 cloneFromGit.sh

./createCWS.sh
./cloneFromGit.sh

cd CWS
catkin_make

