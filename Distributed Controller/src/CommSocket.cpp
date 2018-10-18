#include "CommSocket.hpp"


CommSocket::CommSocket() 
  {
    //default constructor
  }

  
	CommSocket::CommSocket(std::string  worldFrame,
                         std::vector<std::string>     frames,
                         std::vector<std::string> namespaces)
  : m_worldFrame(worldFrame)
  , m_frames(frames)
  , m_namespaces(namespaces)
  , m_listener()
	{
    dummyStr = "dummyString";
		this->initComm();
    this->testComm();
	}

	void CommSocket::initComm()
	{
    ros::NodeHandle nh;

    m_srvStr.push_back("/land");
    m_srvStr.push_back("/takeoff");
    m_srvStr.push_back("/mode");
    m_pubStr.push_back("/goal");
    m_pubStr.push_back("/errorTopic");

    for(auto& nsp : m_namespaces)
    {
      std::vector<std::string> tpcNamesSrv;
      tpcNamesSrv.push_back(nsp + m_srvStr[0]);
      tpcNamesSrv.push_back(nsp + m_srvStr[1]);
      tpcNamesSrv.push_back(nsp + m_srvStr[2]);
      std_srvs::Empty   ph1srv;
      std_srvs::Empty   ph2srv;
      std_srvs::SetBool ph3srv;

      CFC.insert( std::pair<std::string
                  , 
                  CFClients<
                    std_srvs::Empty,
                    std_srvs::Empty, 
                    std_srvs::SetBool>>(nsp, 
                                        CFClients<
                                          std_srvs::Empty,
                                          std_srvs::Empty,
                                          std_srvs::SetBool>(tpcNamesSrv,
                                                             ph1srv,
                                                             ph2srv,
                                                             ph3srv)));

      std::vector<std::string> tpcNamesPub;
      tpcNamesPub.push_back(nsp + m_pubStr[0]);
      tpcNamesPub.push_back(nsp + m_pubStr[1]);
      geometry_msgs::PoseStamped ph1pub;
      geometry_msgs::PoseStamped ph2pub;

      CFP.insert( std::pair<std::string
                  , 
                  CFPublishers<
                    geometry_msgs::PoseStamped,
                    geometry_msgs::PoseStamped>>(nsp, 
                                                 CFPublishers<
                                                   geometry_msgs::PoseStamped,
                                                   geometry_msgs::PoseStamped>(tpcNamesPub,
                                                                               ph1pub,
                                                                               ph2pub)));
    }
    //m_serviceJoy = nh.advertiseService("/joy2master", &CommSocket::joy2master, this);
  }

  void CommSocket::waitClients()
  {
      std::map<
            std::string
            , CFClients<
                std_srvs::Empty,
                std_srvs::Empty, 
                std_srvs::SetBool>>::iterator it;

      it = CFC.begin(); 
      for( ; it != CFC.end(); ++it)
        (it->second).waitForClients();
  }

  void CommSocket::testComm()
  {
    this->waitClients();

    tf::StampedTransform transform;
    for(auto& elem : m_frames)
    {      
      m_listener.waitForTransform(m_worldFrame, elem, ros::Time(0), ros::Duration(10.0));
    }

  }


/*      ROS_INFO("Patrik: no confirmation if tf are actually available (checking process has timer)");
      m_listener.lookupTransform(m_worldFrame, elem, ros::Time(0), transform);
      ROS_INFO("Hi i am x - %f", transform.getOrigin().getX());*/


  /*bool CommSocket::joy2master(std_srvs::Empty::Request&  req,
                              std_srvs::Empty::Response& res)
  {  
    return true;
  }*/  


  /*	void CommSocket::testCom()
	{
    ROS_INFO("Patrik: now testing communication...");

    for(auto& elem : m_frames){      
      m_listener.waitForTransform(m_worldFrame, elem, ros::Time(0), ros::Duration(10000.0));
      ROS_INFO("Patrik: no confirmation if tf are actually available (checking process has timer)");
    }

    ROS_INFO("PATRIK: No service and publisher testing");

    std_srvs::Empty srv;
    testClients<std_srvs::Empty>(   m_clientLand, ros::Duration(-1),  srv);
    testClients<std_srvs::Empty>(m_clientTakeoff, ros::Duration(-1),  srv);
    std_srvs::SetBool srv1;
    srv1.request.data = true;
    testClients<std_srvs::SetBool>( m_clientMode, ros::Duration(-1), srv1);

    // publishers testing is missing

    ROS_INFO("Patrik: All com test successful!");
  }*/




/* 

some cool ways of for_each-ing
  

C++11 bind solution:

std::for_each(this->begin(), this->end(),
      std::bind(&C::transformation, this, std::placeholders::_1));

C++11 lambda solution:

std::for_each(this->begin(), this->end(),
      [this] (T& i) { transformation(i); });

C++14 generic lambda solution:

std::for_each(this->begin(), this->end(),
      [this] (auto&& i) { transformation(std::forward<decltype(i)>(i)); });*/