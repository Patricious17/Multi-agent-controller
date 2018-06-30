#include <ros/ros.h>
#include <ros/console.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
//#include <Vector3.h> for some god damn reason it works without including this
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>
#include <ros/timer_options.h>
#include <vector>
#include <iostream>

#include "CommSocket.hpp"

class Coop
{
public:
	Coop(const std::string&,
    const std::vector<std::string>&,
    const std::vector<std::string>&,
    const ros::NodeHandle&,
    CommSocket&
		);
  void findAgentErrorsInGlobalFrame();
  void calculateGlobalError();

	CommSocket& CS;

private:

	void stampAndPublishErrorMsgs(std::vector<tf::Transform>&);
	void initConsensusController();
	inline void initDisplacements();

	inline void initLeader();
  	inline void initLinks();

  	enum State {
  	Init = 0,
  	Automatic = 1,
  	Emergency = 2,
  	Hover = 3,
  	Idle = 4,
    };

	            std::string  m_worldFrame;
  std::vector<std::string>     m_frames;
  std::vector<std::string> m_namespaces;
    
  //tf::TransformListener m_listener;

  std::vector <geometry_msgs::PoseStamped>  m_goal;
  std::vector <geometry_msgs::PoseStamped> m_error;

  std::vector    <ros::Publisher>       m_pubGoal;
  std::vector    <ros::Publisher>      m_pubError;

  std::vector<tf::Vector3> m_D;
  tf::Vector3 m_leader;
  std::vector<std::vector<tfScalar>> m_gMat;
  std::vector<tfScalar> m_leader2agent;
  std::vector<tf::StampedTransform> m_transforms;
  std::vector<tf::Vector3> m_e_xyz;

};