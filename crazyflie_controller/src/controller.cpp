#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include "pid.hpp"

double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

#ifdef ERROR_INPUT

#include <std_srvs/SetBool.h>

class Controller
{
public:

    Controller(
        const std::string& worldFrame,
        const std::string& frame,
        const ros::NodeHandle& n)
        : m_worldFrame(worldFrame)
        , m_frame(frame)
        , m_pubNav()
        , m_listener()
        , m_pidX(
            get(n, "PIDs/X/kp"),
            get(n, "PIDs/X/kd"),
            get(n, "PIDs/X/ki"),
            get(n, "PIDs/X/minOutput"),
            get(n, "PIDs/X/maxOutput"),
            get(n, "PIDs/X/integratorMin"),
            get(n, "PIDs/X/integratorMax"),
            "x")
        , m_pidY(
            get(n, "PIDs/Y/kp"),
            get(n, "PIDs/Y/kd"),
            get(n, "PIDs/Y/ki"),
            get(n, "PIDs/Y/minOutput"),
            get(n, "PIDs/Y/maxOutput"),
            get(n, "PIDs/Y/integratorMin"),
            get(n, "PIDs/Y/integratorMax"),
            "y")
        , m_pidZ(
            get(n, "PIDs/Z/kp"),
            get(n, "PIDs/Z/kd"),
            get(n, "PIDs/Z/ki"),
            get(n, "PIDs/Z/minOutput"),
            get(n, "PIDs/Z/maxOutput"),
            get(n, "PIDs/Z/integratorMin"),
            get(n, "PIDs/Z/integratorMax"),
            "z")
        , m_pidYaw(
            get(n, "PIDs/Yaw/kp"),
            get(n, "PIDs/Yaw/kd"),
            get(n, "PIDs/Yaw/ki"),
            get(n, "PIDs/Yaw/minOutput"),
            get(n, "PIDs/Yaw/maxOutput"),
            get(n, "PIDs/Yaw/integratorMin"),
            get(n, "PIDs/Yaw/integratorMax"),
            "yaw")
        , m_state(Idle)
        , m_autMode(MinError)
        , m_goal()
        , m_error()
        , m_subscribeGoal()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_avgFreq(0.0)
        , m_numIter(0)
        , m_thrust(0)
        , m_startZ(0.003)
        , m_subscribeError()
        , m_serviceMode()
    {
        ros::NodeHandle nh;
        #ifdef VICON_OFF

        #else
        m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
        #endif
        // create publishers:
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        // create subscribers:
        m_subscribeGoal  = nh.subscribe("goal", 1, &Controller::goalChanged, this);
        m_subscribeError = nh.subscribe("errorTopic", 1, &Controller::errorChanged, this);
        // create services used to control crazyflie
        m_serviceTakeoff = nh.advertiseService("takeoff", &Controller::takeoff, this);
        m_serviceLand =    nh.advertiseService("land", &Controller::land, this);
        m_serviceMode =    nh.advertiseService("mode", &Controller::mode, this);
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        ros::spin();
    }

private:
    void goalChanged (const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_goal =  *msg;
    }

    void errorChanged(const geometry_msgs::PoseStamped::ConstPtr& msg){
        m_error = *msg;
    }

    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;


        tf::StampedTransform transform;
        #ifdef VICON_OFF

        #else
        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
        #endif
        m_startZ = transform.getOrigin().z();

        return true;
    }

    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;
        m_autMode = TrGoal; // landing is basically goal following, where goal is z = ground

        return true;
    }

    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
    }

    void pidReset()
    {
        m_pidX.reset();
        m_pidZ.reset();
        m_pidZ.reset();
        m_pidYaw.reset();
    }

    void goAutomatic()
    {        
        if (m_autMode == TrGoal)
            ROS_INFO("Controller: goAutomatic: %s switch state to AUTOMATIC (mode in TrGoal)", m_frame.c_str());
        else if (m_autMode == MinError)
            ROS_INFO("Controller: goAutomatic: %s switch state to AUTOMATIC (mode in MinError)", m_frame.c_str());        
        m_state = Automatic;
    }

        bool mode(std_srvs::SetBool::Request&  req,
                  std_srvs::SetBool::Response& res){
        bool mod = req.data;
        
            switch(mod){
            case true:
            {
                if(m_state != TakingOff && m_state != Landing && m_state != Idle){
                    m_state = Automatic;
                    m_autMode = TrGoal;
                    res.success = true;
                    ROS_INFO("Controller: mode: %s already AUTOMATIC, switch mode TrGoal", m_frame.c_str());
                    res.message = "Controller: mode: " + m_frame + "already AUTOMATIC (mode TrGoal)";
                    return true;

                }
                else{
                    m_autMode = TrGoal;
                    res.success = true;
                    ROS_INFO("Controller: mode: %s switch to AUTOMATIC (mode TrGoal)", m_frame.c_str());
                    res.message = "Controller: mode: " + m_frame + "switch to AUTOMATIC (mode TrGoal)";
                    return true;
                }
            }
            break;

            case false:
            {                

                if(m_state != TakingOff && m_state != Landing && m_state != Idle){
                    m_state = Automatic;
                    m_autMode = MinError;
                    res.success = true;
                    ROS_INFO("Controller: mode: %s already AUTOMATIC (mode MinError)", m_frame.c_str());
                    res.message = "Controller: mode: " + m_frame + "already AUTOMATIC (mode MinError)";
                    return true;                   
                }
                else{
                    m_autMode = MinError;
                    res.success = true;
                    ROS_INFO("Controller: mode: %s switch to AUTOMATIC (mode MinError)", m_frame.c_str());
                    res.message = "Controller: mode: " + m_frame + "switch to AUTOMATIC (mode MinError)";
                    return true;
                }
            }
            break;           

            return true;
            }
    }

    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();
        //++m_numIter; m_avgFreq = (m_avgFreq*(m_numIter - 1) + 1/dt)/m_numIter;

        switch(m_state)
        {

        case Idle:
            {
                //ROS_INFO("Idle");
                geometry_msgs::Twist msg;
                m_pubNav.publish(msg);
            }
        break;

        case TakingOff:
            {
                    //ROS_INFO("Patrik: taking off sequence started... \n");


                tf::StampedTransform transform;

                #ifdef VICON_OFF

                #else
                    m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                #endif

                
                if (transform.getOrigin().z() > m_startZ + 0.05 || m_thrust > 50000)
                {                        
                    pidReset();
                    m_pidZ.setIntegral(m_thrust / m_pidZ.ki());
                    //ROS_INFO("Patrik: taking off procedure finished with m_thrust: %f, z: %f\n", m_thrust, transform.getOrigin().z());
                    ROS_INFO("Controller: call goAutomatic and setting m_thrust to 0 \n\n");
                    goAutomatic();
                    m_thrust = 0;
                }
                else
                {
                    m_thrust += 10000 * dt;
                    //ROS_INFO("Patrik: taking off... m_thrust: %f, z: %f \n", m_thrust, transform.getOrigin().z());
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pubNav.publish(msg);
                }                          
             
            }
        break;
        case Landing:
            {

                m_goal.pose.position.z = m_startZ + 0.05;
                tf::StampedTransform transform;
                #ifdef VICON_OFF

                #else
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                #endif
                
                if (transform.getOrigin().z() <= m_startZ + 0.15) {
                    m_state = Idle;
                    geometry_msgs::Twist msg; // initializes all elements to 0
                    m_pubNav.publish(msg);
                }
            }
            // intentional fall-thru
        case Automatic:
            {
                switch(m_autMode)
                {
                case TrGoal:
                    {
                        //ROS_INFO("AUTOMATIC(TrGoal)"); //ROS_INFO("avgFreqPos: %f", m_avgFreq);
                        tf::StampedTransform transform;
                        #ifdef VICON_OFF

                        #else
                        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                        #endif                        

                        geometry_msgs::PoseStamped targetWorld;
                        targetWorld.header.stamp = transform.stamp_;
                        targetWorld.header.frame_id = m_worldFrame;
                        targetWorld.pose = m_goal.pose;

                        geometry_msgs::PoseStamped targetDrone;
                        #ifdef VICON_OFF

                        #else
                        m_listener.transformPose(m_frame, targetWorld, targetDrone);
                        #endif                        

                        tfScalar roll, pitch, yaw;
                        tf::Matrix3x3(
                            tf::Quaternion(
                                targetDrone.pose.orientation.x,
                                targetDrone.pose.orientation.y,
                                targetDrone.pose.orientation.z,
                                targetDrone.pose.orientation.w
                            )).getRPY(roll, pitch, yaw);

                        geometry_msgs::Twist msg;
                        msg.linear.x = m_pidX.update(0.0, targetDrone.pose.position.x);
                        msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
                        msg.linear.z = m_pidZ.update(0.0, targetDrone.pose.position.z);
                        msg.angular.z = m_pidYaw.update(0.0, yaw);
                        m_pubNav.publish(msg);
                    }
                    break;

                case MinError:
                    {
                        //ROS_INFO("AUTOMATIC(MinError)");
                        geometry_msgs::Twist msg;
                
                        tfScalar roll, pitch, yaw;
                        tf::Matrix3x3(
                            tf::Quaternion(
                                m_error.pose.orientation.x,
                                m_error.pose.orientation.y,
                                m_error.pose.orientation.z,
                                m_error.pose.orientation.w
                        )).getRPY(roll, pitch, yaw);

                        /*#################################################################################*/
                        /*#################################################################################*/
                        /*#################################################################################*/


                        // this whole mess is just for adding z component, needs fix!!!


                        tf::StampedTransform transform;
                        #ifdef VICON_OFF

                        #else
                        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                        #endif

                        geometry_msgs::PoseStamped targetWorld;
                        targetWorld.header.stamp = transform.stamp_;
                        targetWorld.header.frame_id = m_worldFrame;
                        targetWorld.pose = m_goal.pose;

                        geometry_msgs::PoseStamped targetDrone;

                        #ifdef VICON_OFF

                        #else
                        m_listener.transformPose(m_frame, targetWorld, targetDrone);
                        #endif                        

                        msg.linear.z = m_pidZ.update(0.0, targetDrone.pose.position.z);

                        /*#################################################################################*/
                        /*#################################################################################*/
                        /*#################################################################################*/

                        msg.linear.x =  m_pidX.  update(0.0, m_error.pose.position.x);
                        msg.linear.y =  m_pidY.  update(0.0, m_error.pose.position.y);
                        msg.angular.z = m_pidYaw.update(0.0, yaw);
                        m_pubNav.publish(msg);
                    }
                    break;
                }
            }
        break;
        }
    }

protected:

 enum State
    {
        Idle = 0,
        Automatic = 1, 
        TakingOff = 2,
        Landing = 3,
    };

enum Mode
    {
        TrGoal = 0,
        MinError = 1,
    };

protected:
    std::string m_worldFrame;
    std::string m_frame;
    ros::Publisher m_pubNav;
    tf::TransformListener m_listener;
    PID m_pidX;
    PID m_pidY;
    PID m_pidZ;
    PID m_pidYaw;
    geometry_msgs::PoseStamped m_goal;
    ros::Subscriber m_subscribeGoal;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    float m_thrust;
    float m_startZ;
    ros::Subscriber m_subscribeError;
    State m_state;
    Mode  m_autMode;
    geometry_msgs::PoseStamped m_error;
    ros::ServiceServer m_serviceMode;
    float m_avgFreq;
    int m_numIter;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  // Read parameters
  ros::NodeHandle n("~");
  std::string worldFrame;
  n.param<std::string>("worldFrame", worldFrame, "/world");
  std::string frame;
  n.getParam("frame", frame);
  double frequency;
  n.param("frequency", frequency, 150.0);

  Controller controller(worldFrame, frame, n);
  controller.run(frequency);

  return 0;
}



///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
#else

double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}
class Controller
{
public:

    Controller(
        const std::string& worldFrame,
        const std::string& frame,
        const ros::NodeHandle& n)
        : m_worldFrame(worldFrame)
        , m_frame(frame)
        , m_pubNav()
        , m_listener()
        , m_pidX(
            get(n, "PIDs/X/kp"),
            get(n, "PIDs/X/kd"),
            get(n, "PIDs/X/ki"),
            get(n, "PIDs/X/minOutput"),
            get(n, "PIDs/X/maxOutput"),
            get(n, "PIDs/X/integratorMin"),
            get(n, "PIDs/X/integratorMax"),
            "x")
        , m_pidY(
            get(n, "PIDs/Y/kp"),
            get(n, "PIDs/Y/kd"),
            get(n, "PIDs/Y/ki"),
            get(n, "PIDs/Y/minOutput"),
            get(n, "PIDs/Y/maxOutput"),
            get(n, "PIDs/Y/integratorMin"),
            get(n, "PIDs/Y/integratorMax"),
            "y")
        , m_pidZ(
            get(n, "PIDs/Z/kp"),
            get(n, "PIDs/Z/kd"),
            get(n, "PIDs/Z/ki"),
            get(n, "PIDs/Z/minOutput"),
            get(n, "PIDs/Z/maxOutput"),
            get(n, "PIDs/Z/integratorMin"),
            get(n, "PIDs/Z/integratorMax"),
            "z")
        , m_pidYaw(
            get(n, "PIDs/Yaw/kp"),
            get(n, "PIDs/Yaw/kd"),
            get(n, "PIDs/Yaw/ki"),
            get(n, "PIDs/Yaw/minOutput"),
            get(n, "PIDs/Yaw/maxOutput"),
            get(n, "PIDs/Yaw/integratorMin"),
            get(n, "PIDs/Yaw/integratorMax"),
            "yaw")
        , m_state(Idle)
        , m_goal()
        , m_subscribeGoal()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_thrust(0)
        , m_startZ(0)
    {
        ros::NodeHandle nh;
        m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);
        m_serviceTakeoff = nh.advertiseService("takeoff", &Controller::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &Controller::land, this);
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);
        ros::spin();
    }

private:
    void goalChanged(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_goal = *msg;
    }

    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;

        tf::StampedTransform transform;
        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
        m_startZ = transform.getOrigin().z();

        return true;
    }

    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;

        return true;
    }

    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
    }

    void pidReset()
    {
        m_pidX.reset();
        m_pidZ.reset();
        m_pidZ.reset();
        m_pidYaw.reset();
    }

    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch(m_state)
        {
        case TakingOff:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() > m_startZ + 0.05 || m_thrust > 50000)
                {
                    pidReset();
                    m_pidZ.setIntegral(m_thrust / m_pidZ.ki());
                    m_state = Automatic;
                    m_thrust = 0;
                }
                else
                {
                    m_thrust += 10000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pubNav.publish(msg);
                }

            }
            break;
        case Landing:
            {
                m_goal.pose.position.z = m_startZ + 0.05;
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= m_startZ + 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    m_pubNav.publish(msg);
                }
            }
            // intentional fall-thru
        case Automatic:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);

                geometry_msgs::PoseStamped targetWorld;
                targetWorld.header.stamp = transform.stamp_;
                targetWorld.header.frame_id = m_worldFrame;
                targetWorld.pose = m_goal.pose;

                geometry_msgs::PoseStamped targetDrone;
                m_listener.transformPose(m_frame, targetWorld, targetDrone);

                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(
                    tf::Quaternion(
                        targetDrone.pose.orientation.x,
                        targetDrone.pose.orientation.y,
                        targetDrone.pose.orientation.z,
                        targetDrone.pose.orientation.w
                    )).getRPY(roll, pitch, yaw);

                geometry_msgs::Twist msg;
                msg.linear.x = m_pidX.update(0, targetDrone.pose.position.x);
                msg.linear.y = m_pidY.update(0.0, targetDrone.pose.position.y);
                msg.linear.z = m_pidZ.update(0.0, targetDrone.pose.position.z);
                msg.angular.z = m_pidYaw.update(0.0, yaw);
                m_pubNav.publish(msg);


            }
            break;
        case Idle:
            {
                geometry_msgs::Twist msg;
                m_pubNav.publish(msg);
            }
            break;
        }
    }

private:

    enum State
    {
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 3,
    };

private:
    std::string m_worldFrame;
    std::string m_frame;
    ros::Publisher m_pubNav;
    tf::TransformListener m_listener;
    PID m_pidX;
    PID m_pidY;
    PID m_pidZ;
    PID m_pidYaw;
    State m_state;
    geometry_msgs::PoseStamped m_goal;
    ros::Subscriber m_subscribeGoal;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    float m_thrust;
    float m_startZ;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");

  // Read parameters
  ros::NodeHandle n("~");
  std::string worldFrame;
  n.param<std::string>("worldFrame", worldFrame, "/world");
  std::string frame;
  n.getParam("frame", frame);
  double frequency;
  n.param("frequency", frequency, 150.0);

  //ControllerCoop controller(worldFrame, frame, n);
  Controller     controller(worldFrame, frame, n);
  controller.run(frequency);

  return 0;
}

#endif



/*class ControllerCoop : public Controller {

public:

    ControllerCoop(const std::string& worldFrame,
                   const std::string& frame,
                   const ros::NodeHandle& n)
                   : Controller(worldFrame, frame, n) 

    {
        ros::NodeHandle nh;
        m_subscribeError = nh.subscribe("errorTopic", 1, &ControllerCoop::errorChanged, this);
        m_serviceMode = nh.advertiseService("mode", &ControllerCoop::mode, this);
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &ControllerCoop::iteration, this);
        ros::spin();
    }

};*/


