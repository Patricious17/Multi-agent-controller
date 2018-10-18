#include "coop.hpp"


Coop::Coop(
    const std::string& worldFrame,
    const std::vector<std::string>& frames,
    const std::vector<std::string>& namespaces,
    const ros::NodeHandle& n,
    	  CommSocket& CSarg)
    : m_worldFrame(worldFrame)
    , m_frames(frames)
    , m_namespaces(namespaces)
    //, m_listener()
    , CS(CSarg)
  {
    ros::NodeHandle nh;

#ifndef VICON_OFF
    this->initConsensusController();
#endif
	}

  void Coop::findAgentErrorsInGlobalFrame(){

    tf::StampedTransform transform;
    int i = 0;
    for(auto& elem : m_frames){
      CS.m_listener.lookupTransform(m_worldFrame, elem, ros::Time(0), transform);
      m_transforms[i++] = transform;
    }

    std::vector<tf::Vector3> p_xyz;

    for(auto& elem : m_transforms ){
      p_xyz.push_back(elem.getOrigin());
    }
      
    std::vector<tf::Vector3>::iterator it1 = p_xyz.begin();
    std::vector<tf::Vector3>::iterator it2 =   m_D.begin();

    for(; (it2 != m_D.end()) && (it1 != p_xyz.end()); ++it1, ++it2)
      (*it1) -= (*it2);

    m_e_xyz = p_xyz;
  }

  void Coop::calculateGlobalError(){

    std::vector<tf::Vector3> gErrors;

    tf::Vector3 tfError1 =                                            m_gMat[0][1] * (m_e_xyz[1] - m_e_xyz[0]) + m_gMat[0][2] * (m_e_xyz[2] - m_e_xyz[0]);// + m_gMat[0][3] * (m_e_xyz[3] - m_e_xyz[0]);
    tf::Vector3 tfError2 = m_gMat[1][0] * (m_e_xyz[0] - m_e_xyz[1])                                            + m_gMat[1][2] * (m_e_xyz[2] - m_e_xyz[1]);// + m_gMat[1][3] * (m_e_xyz[3] - m_e_xyz[1]);
    tf::Vector3 tfError3 = m_gMat[2][0] * (m_e_xyz[0] - m_e_xyz[2]) + m_gMat[2][1] * (m_e_xyz[1] - m_e_xyz[2])                                           ;// + m_gMat[2][3] * (m_e_xyz[3] - m_e_xyz[2]);
    //tf::Vector3 tfError4 = m_gMat[3][0] * (m_e_xyz[0] - m_e_xyz[3]) + m_gMat[3][1] * (m_e_xyz[1] - m_e_xyz[3]) + m_gMat[3][2] * (m_e_xyz[2] - m_e_xyz[3]);

    tfError1 += 1.0*(m_leader - m_e_xyz[0]); tfError2 += 1.0*(m_leader - m_e_xyz[1]); tfError3 += 1.0*(m_leader - m_e_xyz[2]);

    gErrors.push_back(tfError1); gErrors.push_back(tfError2); gErrors.push_back(tfError3);
    //gErrors.push_back(tfError4);

    tf::Transform  invTF;
    tf::Transform  tf2local;
    tf::Transform  errorTF;
    std::vector<tf::Transform> errorTFVec;
    tf::Vector3 localError;
    tf::Quaternion invQuat;

    tf::Vector3 idleVec(0, 0, 0);
     tf2local.setOrigin(idleVec);

    std::vector<tf::StampedTransform>::iterator itTF = m_transforms.begin();
    std::vector<std::string>::iterator      itFrames = m_frames.begin();
    std::vector<tf::Vector3>::iterator     itgErrors =  gErrors.begin();

    for( ; (itTF != m_transforms.end()) && (itFrames != m_frames.end()) && (itgErrors != gErrors.end()); ++itTF, ++itFrames, ++itgErrors)
    {
      invTF = (*itTF).inverse();
      invQuat  = invTF.getRotation();
      tf2local.setRotation(invQuat);
      localError = tf2local*(*itgErrors);
      errorTF.setOrigin(localError);
      errorTF.setRotation(invQuat);
      errorTFVec.push_back(errorTF);
      //drone yaw should be the same as world's      
      //use transformPose to calculate yaw, something like --> m_listener.transformPose(m_frame, targetWorld, targetDrone);
    }
    stampAndPublishErrorMsgs(errorTFVec);
  }

  void Coop::stampAndPublishErrorMsgs(std::vector<tf::Transform>& TFvec){
    ros::NodeHandle nh;
    geometry_msgs::PoseStamped errorMsg;
    std::vector<geometry_msgs::PoseStamped> errorMsgVec;
    std::vector<tf::Transform>::iterator TFvecIt; TFvecIt = TFvec.begin();

    for(auto& fr :  m_frames){
      tf::Stamped<tf::Transform> tf2send(*TFvecIt, ros::Time::now(), fr);
      poseStampedTFToMsg(tf2send, errorMsg);
      errorMsgVec.push_back(errorMsg);
      ++TFvecIt;
    }
    std::string er("/errorTopic");
    CS.callPublishers<geometry_msgs::PoseStamped>(er, errorMsgVec);
  }

  void Coop::initConsensusController(){
    initDisplacements();
    initLeader();
    initLinks();

    tf::StampedTransform transform;
    for(auto& elem : m_frames){
      CS.m_listener.lookupTransform(m_worldFrame, elem, ros::Time(0), transform);
      m_transforms.push_back(transform);
    }
  }

  inline void Coop::initDisplacements(){

    tf::Vector3 elem1( 0.5,  0.5, 0.0);
    tf::Vector3 elem2(-0.5, -0.5, 0.0);
    tf::Vector3 elem3(-0.5,  0.5, 0.0);
    //tf::Vector3 elem4( 0.5, -0.5, 0.0);

    m_D.push_back(elem1);
    m_D.push_back(elem2);
    m_D.push_back(elem3);
    //m_D.push_back(elem4);
  }

  inline void Coop::initLeader(){
    tf::Vector3 elem(0.0, 0.0, 0.7);
    m_leader = elem;
  }

  inline void Coop::initLinks(){
    std::vector<tfScalar> row1 = {0.0, 3.0, 3.0};//, 1.0};
    std::vector<tfScalar> row2 = {3.0, 0.0, 3.0};//, 1.0};
    std::vector<tfScalar> row3 = {3.0, 3.0, 0.0};//, 1.0};
    //std::vector<tfScalar> row4 = {1.0, 1.0, 1.0, 0.0};

    m_gMat.push_back(row1);
    m_gMat.push_back(row2);
    m_gMat.push_back(row3);
    //m_gMat.push_back(row4);

    std::vector<tfScalar> elem = {1.0, 0.0, 0.0, 0.0};
    m_leader2agent = elem;
  }
