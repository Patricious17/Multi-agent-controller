#include "coop.hpp"

#ifndef TEST_COMM

class CoopSM 
{

public:

	CoopSM(
    const std::string& worldFrame,
    const std::vector<std::string>& frames,
    const std::vector<std::string>& namespaces,
    const ros::NodeHandle& n,
    CommSocket& CSarg,
    Coop& cooparg,
    bool isCoop)
    : m_worldFrame(worldFrame)
    , m_frames(frames)
    , m_namespaces(namespaces)
    , m_state(Init)
    , m_goal()
    , m_serviceJoy()
    , m_time(0.0)
    , m_avgFreq(0.0)
    , m_goCoop(false)
    , CS(CSarg)
    , coop(cooparg)
    , m_tOffSeq(0)
    , m_tOffTime(2.0)
    , m_coopTime(20.0)
    , coopIsInit(false)
  {
    ros::NodeHandle nh;
    this->initGoal();

    if (isCoop) m_autoMode = ConsensusFormation;
    else m_autoMode = GoalTracking;
	};

  void run(double& frequency){
    ros::NodeHandle nh1, nh3;
    ros::CallbackQueue queue1, queue3;

    ros::Duration(5.0).sleep();
    m_state = Init;

    nh1.setCallbackQueue(&queue1);
    nh3.setCallbackQueue(&queue3);
    
    ros::TimerCallback tCbk1; /*typedef boost::function<void(const TimerEvent &)>*/
    ros::TimerCallback tCbk3;

    tCbk1 = boost::bind(&CoopSM::controllerIteration, this, _1);
    tCbk3 = boost::bind(&CoopSM::         updateGoal, this, _1);

    ros::TimerOptions tOpt1(ros::Duration(1.0 /frequency),
                            tCbk1, 
                            &queue1);
    
    ros::TimerOptions tOpt3(ros::Duration(3.0 /frequency), 
                            tCbk3, 
                            &queue3);

    ros::Timer timer1 = nh1.createTimer(tOpt1);
    ros::Timer timer3 = nh3.createTimer(tOpt3);

    ros::AsyncSpinner spinner1(0, &queue1); // 0 -> number of threads = number of CPU cores
    ros::AsyncSpinner spinner3(1, &queue3);

    spinner3.start();
    spinner1.start();
    
    ROS_INFO("CoopSM: all threads running, m_time is: %f", m_time);

    ros::waitForShutdown();
  }

private:
  void controllerIteration(const ros::TimerEvent& e){
    if(!coopIsInit) this->initCoop();
    else
    {
      float dt = e.current_real.toSec() - e.last_real.toSec();
      m_time+=dt;
    }

    switch(m_state){

      case Init:
      {
        if (this->takeOffSequence(float(2.0))){
          this->toAutomaticState();
        }        
      }
      break;

      case Automatic:
      {
        switch(m_autoMode){
          case GoalTracking:
          {

          }
          break;

          case ConsensusFormation:
          {
            this->enterCoop();
            coop.findAgentErrorsInGlobalFrame();
            coop.calculateGlobalError();
          }
        }
      }        
      break;

      case Emergency:
      {
        this->emergencyRoutine();
      }
      break;

      case Idle:
      {
        ROS_INFO("MASTER: in idle state...");
      }
      break;
    }   
  }

  void updateGoal(const ros::TimerEvent& e){
    // use this function for defining positions of waypoints(goals) and graph leader
    float dt = e.current_real.toSec() - e.last_real.toSec();
    
    m_goal[0].header.seq += 1;
    m_goal[0].header.frame_id = m_worldFrame;
    m_goal[0].pose.position.x = 0.5;
    m_goal[0].pose.position.y = 0.5;
    m_goal[0].pose.position.z = 1;
    m_goal[0].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);    
    m_goal[0].header.stamp = ros::Time::now();
    //m_pubGoal[0].publish(m_goal[0]);

    m_goal[1].header.seq += 1;
    m_goal[1].header.frame_id = m_worldFrame;
    m_goal[1].pose.position.x = -0.5;
    m_goal[1].pose.position.y = -0.5;
    m_goal[1].pose.position.z =  1;
    m_goal[1].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);    
    m_goal[1].header.stamp = ros::Time::now();
    //m_pubGoal[1].publish(m_goal[1]);

    m_goal[2].header.seq += 1;
    m_goal[2].header.frame_id = m_worldFrame;
    m_goal[2].pose.position.x = -0.5;
    m_goal[2].pose.position.y =  0.5;
    m_goal[2].pose.position.z =  1;
    m_goal[2].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);    
    m_goal[2].header.stamp = ros::Time::now();
    //m_pubGoal[2].publish(m_goal[2]);

    // m_goal[3].header.seq += 1;
    // m_goal[3].header.frame_id = m_worldFrame;
    // m_goal[3].pose.position.x =  0.5;
    // m_goal[3].pose.position.y = -0.5;
    // m_goal[3].pose.position.z =  0.7;
    // m_goal[3].pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);    
    // m_goal[3].header.stamp = ros::Time::now();

    // m_pubGoal[3].publish(m_goal[3]);

    std::string g("/goal");
    CS.callPublishers<geometry_msgs::PoseStamped>(g, m_goal);
  }

  bool initCoop()
  {
    std_srvs::SetBool srvMd; srvMd.request.data = true; std::string md("/mode");
    CS.callClients<std_srvs::SetBool>(md, srvMd);// switch mode to TrGoal
    coopIsInit = true;
  }

  bool takeOffSequence(float interval)
  {
    if (m_time > m_tOffTime + m_tOffSeq*interval)
    {
      std::string tOff("/takeoff");
      CS.callSingleClient<std_srvs::Empty>(tOff, m_namespaces[m_tOffSeq++]);
    }

    if (m_tOffSeq == (int) m_namespaces.size()) return true;

    return false;    
  }

  bool toAutomaticState(float delay = 10.0)
  {
    m_state = Automatic;
    m_coopTime = m_time + delay;
    ROS_INFO("CoopSM: master initialized... going to CoopSM::Automatic state ");
    ROS_INFO("m_time: %f \nm_coopTime: %f", m_time, m_coopTime);
  }

  void enterCoop()
  {
    ros::NodeHandle nh;
    if ((m_time >= m_coopTime) && (!m_goCoop)){
      m_goCoop = true;
      std_srvs::SetBool srvMd; srvMd.request.data = false; std::string md("/mode");
      CS.callClients<std_srvs::SetBool>(md, srvMd);// switch mode to MinError
    }
  }

  void emergencyRoutine()
  {
    std::string ld("/land");
    CS.callClients<std_srvs::Empty>(ld);
    m_state = Idle;
  }

  void initGoal(){
    geometry_msgs::PoseStamped goal;

    goal.header.seq = 0;
    goal.header.frame_id = m_worldFrame;
    goal.pose.position.x = 0.5;
    goal.pose.position.y = 0.5;
    goal.pose.position.z = 0.7;
    goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);    
    goal.header.stamp = ros::Time::now();

    m_goal.push_back(goal);

    goal.header.seq = 0;
    goal.header.frame_id = m_worldFrame;
    goal.pose.position.x = -0.5;
    goal.pose.position.y = -0.5;
    goal.pose.position.z =  0.7;
    goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);    
    goal.header.stamp = ros::Time::now();

    m_goal.push_back(goal);

    goal.header.seq = 0;
    goal.header.frame_id = m_worldFrame;
    goal.pose.position.x = -0.5;
    goal.pose.position.y =  0.5;
    goal.pose.position.z =  0.7;
    goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);    
    goal.header.stamp = ros::Time::now();

    m_goal.push_back(goal);

    // goal.header.seq = 0;
    // goal.header.frame_id = m_worldFrame;
    // goal.pose.position.x =  0.5;
    // goal.pose.position.y = -0.5;
    // goal.pose.position.z =  0.7;
    // goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);    
    // goal.header.stamp = ros::Time::now();

    // m_goal.push_back(goal);

    //m_target
  }
  
private:
  enum State {
  Init = 0,
  Automatic = 1,
  Emergency = 2,
  Idle = 3,
};

private:
  enum AutoMode {
  GoalTracking = 0,
  ConsensusFormation = 1,
};

public:
  CommSocket& CS;
  Coop& coop;

private:
  State m_state;
  AutoMode m_autoMode;

	            std::string  m_worldFrame;
  std::vector<std::string>     m_frames;
  std::vector<std::string> m_namespaces;

  std::vector <geometry_msgs::PoseStamped> m_goal;
  ros::ServiceServer m_serviceJoy;

  bool m_goCoop, coopIsInit;
  float m_time, m_avgFreq;
  int m_numIter, m_tOffSeq;
  float m_tOffTime, m_coopTime;


  void printV(tf::Vector3 v){
    ROS_INFO("vector x: %f  y: %f  z: %f\n", v.getX(), v.getY(), v.getZ());
  }

  void printQ(tf::Quaternion q){
    ROS_INFO("Quaternion: angle : %f\n", q.getAngle());
    printV(q.getAxis());
  }

  template<class T>
  void printT(T t){
    ROS_INFO("Transform:\n");
    printV(t.getOrigin());
    printQ(t.getRotation());
  }
};

int main(int argc, char **argv)
{

  ros::init(argc, argv, "coopSM");
  ros::NodeHandle n("~");
  std::string worldFrame;
  n.param<std::string>("worldFrame", worldFrame, "/world");
  std::vector<std::string> frames;
  n.getParam("cf/frames", frames);
  double frequency;  
  n.param("frequency", frequency, 100.0);
  std::vector<std::string> namespaces;
  n.getParam("cf/namespaces", namespaces);
  bool isCoop;
  n.getParam("isCoop", isCoop);

  CommSocket CS(worldFrame, frames, namespaces);
  Coop coop(worldFrame, frames, namespaces, n, CS);
  CoopSM coopSM(worldFrame, frames, namespaces, n, CS, coop, isCoop);
  coopSM.run(frequency);  
  return 0;
}
#endif



































#ifdef TEST_COMM

int main(int argc, char **argv)
{

  ros::init(argc, argv, "coopSM");
  ros::NodeHandle n("~");
  std::string worldFrame;
  n.param<std::string>("worldFrame", worldFrame, "/world");
  std::vector<std::string> frames;
  n.getParam("cf/frames", frames);
  double frequency;  
  n.param("frequency", frequency, 100.0);
  std::vector<std::string> namespaces;
  n.getParam("cf/namespaces", namespaces);

  CommSocket CS(worldFrame, frames, namespaces);
  Coop coop(worldFrame, frames, namespaces, n, CS);
  CoopSM coopSM(worldFrame, frames, namespaces, n, CS, coop);
  coopSM.run(frequency);
  return 0;
}

#endif

/*int main(int argc, char **argv)
{
  // main is used to initialize node and to read parameters
  // by using node handle
  ros::init(argc, argv, "coop");
  ros::NodeHandle n("~");
  std::string worldFrame;
  n.param<std::string>("worldFrame", worldFrame, "/world"); // there is only one; vicon
  std::vector<std::string> frames;
  n.getParam("cf/frames", frames); // add them in launch file and yaml file too
  double frequency;  

  n.param("frequency", frequency, 250.0);
  std::vector<std::string> namespaces;
  n.getParam("cf/namespaces", namespaces);

  Coop coop(worldFrame, frames, namespaces, n);
  coop.run(frequency);

  return 0;
}*/



/*
  bool joy2master(std_srvs::Empty::Request& req,
                  std_srvs::Empty::Response& res)
  {
    ROS_INFO("Patrik: MASTER: Servicing button press:\n");
    
    if (m_state == Idle) 
    {
      ROS_INFO("Patrik: MASTER: Idle --> Init");
      m_state = Init;
    }

    else if ((m_state != Idle) && (m_state != Init))
    {
      ROS_INFO("Patrik: MASTER: emergency landing");
      m_state = Emergency;
    }

    return true;
  }*/



/*

    //std_srvs::Empty srv;
    //CS.CFC[nmsp].cl[nmsp + "/takeoff"].call(srv);
    //std::string tOff("/takeoff"); std::string md("/mode");
    //CS.callClients<std_srvs::Empty>(tOff);
    //std_srvs::SetBool srvMd; srvMd.request.data = true;
    //CS.callClients<std_srvs::SetBool>(md, srvMd);
    //while(true){CS.CFP[nmsp].pub[nmsp + "/goal"].publish(dummyMsg);}*/
