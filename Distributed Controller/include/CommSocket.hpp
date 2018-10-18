/*

Patrik Kolaric patrikkolaric92@gmail.com

EXAMPLE USAGE:

    geometry_msgs::PoseStamped dummyMsg;
    CS.CFP["crazyflie1"].pub["crazyflie1/goal"].publish(dummyMsg);
*/
#ifndef _COMM_SOCKET_
#define _COMM_SOCKET_

#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tuple>
#include <vector>
#include <map>
#include <algorithm>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class... T_CLS>
class CFPublishers
{

public:
  std::map<std::string, ros::Publisher> pub;

private:
  //ros::NodeHandle m_nh;//std::tuple<T_CLS...> clMsgs;// TODO: implement nh member instead of having it locally in every method
  std::size_t numMsgs;
  int m_upCount;
  std::vector<std::string> m_topicNames;


  template<typename T, typename... TS>
  void parsePublishers(T arg, TS... args)
  {
    ros::NodeHandle nh;
    ros::Publisher newPub; std::string newTop;
    newTop = m_topicNames[m_upCount++];
    newPub = nh.advertise<T>(newTop.c_str(), 2);
    pub.insert(std::pair<std::string, 
                         ros::Publisher> 
                                        (newTop, newPub));
    parsePublishers(args...);
    return;
  }

  template<typename T>
  void parsePublishers(T final)
  {
    ros::NodeHandle nh;// TODO: implement nh member instead of having it locally in every method
    ros::Publisher newPub; std::string newTop;
    newTop = m_topicNames[m_upCount++];
    newPub = nh.advertise<T>(newTop.c_str(), 2);
    pub.insert(std::pair<std::string, 
                         ros::Publisher> 
                                        (newTop, newPub));
    return;
  }
  void parsePublishers(){
    ROS_WARN("Never should've reached --> error in void parsePublishers in CommSocket.cpp");
  }
  // TODO: make service client connections persistent; implement custom connection/disconnection logic

public:  
  CFPublishers(std::vector<std::string>& topicNames, T_CLS... Args)
  : m_topicNames(topicNames)
  , m_upCount(0)
  , numMsgs(0)
  {   
    ros::NodeHandle nh;
    this->numMsgs = sizeof...(T_CLS);

    if(this->numMsgs != topicNames.size())
    {
      ROS_WARN("Number of topic strings doesn't match number of message types in CFPublishers, in CommSocket.hpp");
      //throw ObjectCouldNotBeCreatedException();
    }
    else
    {
      parsePublishers(Args...);
    }
  }

  CFPublishers(){} // default constructor
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <class... T_CLS>
class CFClients
{
public:
  std::map<std::string, ros::ServiceClient> cl;

private:
  //ros::NodeHandle m_nh;//std::tuple<T_CLS...> clMsgs;// TODO: implement nh member instead of having it locally in every method
  std::size_t numMsgs;
  int m_upCount;
  std::vector<std::string> m_topicNames;  


  template<typename T, typename... TS>
  void parseClients(T arg, TS... args)
  {
    ros::NodeHandle nh;
    ros::ServiceClient newCl; std::string newTop;
    newTop = m_topicNames[m_upCount++];
    newCl  = nh.serviceClient<T>(newTop.c_str());
    cl.insert(std::pair<       std::string, 
                        ros::ServiceClient> 
                                            (newTop, newCl));
    parseClients(args...);
    return;
  }

  template<typename T>
  void parseClients(T final)
  {
    ros::NodeHandle nh;
    ros::ServiceClient newCl; std::string newTop;
    newTop = m_topicNames[m_upCount++];
    newCl  = nh.serviceClient<T>(newTop.c_str());
    cl.insert(std::pair<       std::string, 
                        ros::ServiceClient> 
                                            (newTop, newCl));
    return;
  }
  void parseClients(){
    ROS_WARN("Never should've reached --> error in void pasrseClients in CommSocekt.cpp");
  }

  // TODO: make service client connections persistent; implement custom connection/disconnection logic
public:  
  CFClients(std::vector<std::string>& topicNames, T_CLS... Args)
  : m_topicNames(topicNames)
  , m_upCount(0)
  , numMsgs(0)
  {   
    ros::NodeHandle nh;
    this->numMsgs = sizeof...(T_CLS);

    if(this->numMsgs != topicNames.size())
    {
      ROS_WARN("Number of topic strings doesn't match number of message types in CFClients, in CommSocket.hpp");
      //throw ObjectCouldNotBeCreatedException();
    }
    else
    {
      parseClients(Args...);
    }
  }

  CFClients(){};

  bool waitForClients()
  {
    for(auto& tpc : m_topicNames)
    {
      ROS_INFO("CFClients::waitForServices: Waiting for service %s", tpc.c_str());
      this->cl[tpc].waitForExistence();
      ROS_INFO("CFClients::waitForServices: cl-srv %s comm established", tpc.c_str());
    }    
    return true;    
  }
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class CommSocket
{

public:
  CommSocket();
	CommSocket(std::string, std::vector<std::string>, std::vector<std::string>);
  void waitClients();
  void callClients();

public:

  // TODO: typedef those ugly types
  std::map<
            std::string
            , CFClients<
                std_srvs::Empty,
                std_srvs::Empty, 
                std_srvs::SetBool>
          > CFC;

  std::map<
            std::string
            , CFPublishers<
                geometry_msgs::PoseStamped, 
                geometry_msgs::PoseStamped>
          > CFP;

	//ros::ServiceServer m_serviceJoy;
  std::string dummyStr;
  std::vector<std::string> m_srvStr;
  std::vector<std::string> m_pubStr;
  tf::TransformListener m_listener;

private:
  std::string  m_worldFrame;
  std::vector<std::string>     m_frames;
  std::vector<std::string> m_namespaces;


private:
	void initComm();
  void testComm();
	//void testCom();
	//bool joy2master(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

public:
  template<typename T>
  void callClients(std::string& tpc)
  {
    T srv;
    for(auto& nmsp : m_namespaces)
      this->CFC[nmsp].cl[nmsp + tpc].call(srv);
  }

  template<typename T>
  void callClients(std::string& tpc, T& srv)
  {
    for(auto& nmsp : m_namespaces)
      this->CFC[nmsp].cl[nmsp + tpc].call(srv);
  }

  template<typename T>
  void callSingleClient(std::string& tpc, std::string& nmsp, T& srv)
  {
    this->CFC[nmsp].cl[nmsp + tpc].call(srv);
  }

  template<typename T>
  void callSingleClient(std::string& tpc, std::string& nmsp)
  {
    T srv;
    this->CFC[nmsp].cl[nmsp + tpc].call(srv);
  }

  template<typename T>
  void callPublishers(std::string& tpc, std::vector<T>& pubs)
  {
    auto itPubs = pubs.begin();
    for(auto& nmsp : m_namespaces){
      this->CFP[nmsp].pub[nmsp + tpc].publish(*itPubs);
      ++itPubs;
    }
  }
};

/*
CFClinet:

*/

#endif