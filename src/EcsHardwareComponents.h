/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
     this list of conditions and the following disclaimer in the documentation
     and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#ifndef AFF_ECSHARDWARECOMPONENTS_H
#define AFF_ECSHARDWARECOMPONENTS_H

#include "ComponentBase.h"
#include "EntityBase.h"

#include <Rcs_graph.h>
#include <Rcs_cmdLine.h>
#include <Rcs_macros.h>
#include <Rcs_filters.h>

#if defined (USE_ROS)
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#endif

#if defined (USE_FRI)
#include <FriComponent.h>
#endif

#if defined (USE_SDH)
#include <SDHComponent.h>
#endif

#if defined (USE_LWA)
#include <LWAComponent.h>
#endif

#if defined (USE_CANOPENCONTROL)
#include <IEFComponent.h>
#include <IMEComponent.h>
#include <FTSComponent.h>
#endif

#if defined (USE_VICON)
#include <ViconComponent.h>
#endif

#if defined (USE_JACO)
#include <JacoComponent.h>
#endif

#if defined (USE_ROBOCOMPONENTS)
#include <JacoShmComponent.h>
#else
namespace Rcs
{
class JacoShmComponent : public aff::ComponentBase
{
public:
  enum JacoType
  {
    Jaco6 = 0,
    Jaco7_left,
    Jaco7_right
  };
};
}
#endif

#if !defined (USE_JACO) || defined (_MSC_VER)

namespace Rcs
{
class JacoComponent : public aff::ComponentBase
{
public:
  enum JacoType
  {
    Jaco6 = 0,
    Jaco7_left,
    Jaco7_right
  };
};


}
#endif

#if defined (USE_OPENCV_ARUCO)
#include <ArucoComponent.h>
#endif

#include <Rcs_typedef.h>
#include <Rcs_math.h>

#include <exception>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>



namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
class ComponentFactory
{
public:

  static void registerComponent(std::string name)
  {
    compiledComponents.push_back(name);
  }

  static void print()
  {
    std::cout << "Found " << compiledComponents.size()
              << " components:"  << std::endl;

    for (size_t i=0; i<compiledComponents.size(); ++i)
    {
      std::cout << compiledComponents[i] << std::endl;
    }
  }

  static std::vector<std::string> compiledComponents;
};

std::vector<std::string> ComponentFactory::compiledComponents;

class ComponentFactoryRegistrar
{
public:

  ComponentFactoryRegistrar(std::string name)
  {
    ComponentFactory::registerComponent(name);
  }

};



/*******************************************************************************
 *
 ******************************************************************************/
#if defined (USE_ROS)
static ComponentFactoryRegistrar regROS("ROS enabled");
void initROS()
{
  static bool rosInitialized = false;

  if (rosInitialized)
  {
    RLOG(1, "ROS already initialized - doing nothing");
    return;
  }

  rosInitialized = true;

  Rcs::CmdLineParser argP;
  int argc = 0;
  char** argv = NULL;

  argc = argP.getArgs(&argv);

  if (argc==0)
  {
    RCHECK(argv == NULL);
  }

  RMSG("Calling ros::init()");
  ros::init(argc, argv, "RcsROS", ros::init_options::NoSigintHandler);
}
#else
void initROS()
{
  RFATAL("You are trying to initialize ROS, but it has not been compiled in");
}
#endif



/*******************************************************************************
 *
 ******************************************************************************/
#if defined (USE_ROS)
static ComponentFactoryRegistrar regPTU("RoboPtuComponent");
class PtuComponent : public ComponentBase
{
public:

  static PtuComponent* create(EntityBase* parent)
  {
    return new PtuComponent(parent);
  }

  PtuComponent(EntityBase* parent, std::string topic="/ptu/state") :
    ComponentBase(parent), topicName(topic),
    panJointName("ptu_pan_joint"), tiltJointName("ptu_tilt_joint"),
    panJointAngle(0.0), tiltJointAngle(0.0),
    panFilt(NULL), tiltFilt(NULL),
    panJointIdx(-1), tiltJointIdx(-1), desiredValuesInitialized(false)
  {
    subscribe("Start", &PtuComponent::onStart);
    subscribe("Stop", &PtuComponent::onStop);
    subscribe("UpdateGraph", &PtuComponent::onUpdateGraph);
    subscribe("SetJointCommand", &PtuComponent::setJointPosition);
  }

private:

  void rosCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    bool panUpdated = false, tiltUpdated = false;

    for (size_t i=0; i<msg->name.size(); ++i)
    {

      if (msg->name[i]==panJointName)
      {
        NLOG(0, "Joint %zu is %s %f", i, msg->name[i].c_str(), msg->position[i]);
        mtx.lock();
        panJointAngle = msg->position[i];
        mtx.unlock();
        panUpdated = true;
      }
      else if (msg->name[i]==tiltJointName)
      {
        NLOG(0, "Joint %zu is %s %f", i, msg->name[i].c_str(), msg->position[i]);
        mtx.lock();
        tiltJointAngle = msg->position[i];
        mtx.unlock();
        tiltUpdated = true;
      }
      else
      {
        RLOG(0, "Joint %zu \"%s\" is not found", i, msg->name[i].c_str());
      }

    }

    // Initialize the desired values after the first ROS joint state message
    if (!this->desiredValuesInitialized)
    {
      if (panUpdated && tiltUpdated)
      {
        this->desiredValuesInitialized = true;
        const double tmc = 0.1;
        const double vmax = RCS_DEG2RAD(30.0);
        const double dt = 1.0/50.0;//getEntity()->getDt()
        this->panFilt = new Rcs::RampFilter1D(panJointAngle, tmc, vmax, dt);
        this->tiltFilt = new Rcs::RampFilter1D(tiltJointAngle, tmc, vmax, dt);
      }
      return;
    }

    // Send commands only after the desired values have been initialized
    panFilt->iterate();
    tiltFilt->iterate();

    sensor_msgs::JointState cmd;

    if (panJointIdx!=-1)
    {
      const double gain = 0.5;
      double vel = panFilt->getVelocity() + gain*(panFilt->getPosition()-panJointAngle);
      cmd.name.push_back(panJointName);
      cmd.position.push_back(0.0);
      cmd.velocity.push_back(vel);
      //RLOG(0, "Pan: %f", RCS_RAD2DEG(vel));
    }

    if (tiltJointIdx!=-1)
    {
      const double gain = 0.5;
      double vel = tiltFilt->getVelocity() + gain*(tiltFilt->getPosition()-tiltJointAngle);
      cmd.name.push_back(tiltJointName);
      cmd.position.push_back(0.0);
      cmd.velocity.push_back(vel);
      //RLOG(0, "Tilt: %f", RCS_RAD2DEG(vel));
    }

    if (!cmd.name.empty())
    {
      ptu_pub.publish(cmd);
    }

  }

  void onUpdateGraph(RcsGraph* graph)
  {

    RCSGRAPH_TRAVERSE_JOINTS(graph)
    {
      if (STREQ(panJointName.c_str(), JNT->name))
      {
        NLOG(0, "q[%zu] = %f", JNT->jointIndex, panJointAngle);
        mtx.lock();
        MatNd_set(graph->q, JNT->jointIndex, 0, panJointAngle);
        panJointIdx = JNT->jointIndex;
        mtx.unlock();
      }

      if (STREQ(tiltJointName.c_str(), JNT->name))
      {
        NLOG(0, "q[%zu] = %f", JNT->jointIndex, panJointAngle);
        mtx.lock();
        MatNd_set(graph->q, JNT->jointIndex, 0, tiltJointAngle);
        tiltJointIdx = JNT->jointIndex;
        mtx.unlock();
      }

    }

  }

  void setJointPosition(const MatNd* q_des)
  {

    mtx.lock();
    if ((panJointIdx!=-1) && (panFilt))
    {
      panFilt->setTarget(MatNd_get(q_des, panJointIdx, 0));
    }

    if ((tiltJointIdx!=-1)&& (tiltFilt))
    {
      tiltFilt->setTarget(MatNd_get(q_des, tiltJointIdx, 0));
    }
    mtx.unlock();


    // sensor_msgs::JointState cmd;

    // if (panJointIdx!=-1)
    // {
    //   cmd.name.push_back(panJointName);
    //   double ang = MatNd_get(q_des, panJointIdx, 0);
    //   // cmd.position.push_back(ang);
    //   // cmd.velocity.push_back(RCS_DEG2RAD(40.0));
    //   cmd.position.push_back(0.0);
    //   cmd.velocity.push_back(vel);
    //   RLOG(0, "Pan: %f", ang);
    // }

    // if (tiltJointIdx!=-1)
    // {
    //   cmd.name.push_back(tiltJointName);
    //   double ang = MatNd_get(q_des, tiltJointIdx, 0);
    //   // cmd.position.push_back(ang);
    //   // cmd.velocity.push_back(RCS_DEG2RAD(40.0));
    //   cmd.position.push_back(0.0);
    //   cmd.velocity.push_back(vel);
    //   RLOG(0, "Tilt: %f", ang);
    // }

    // if (count%10==0)
    // {
    //   if (!cmd.name.empty())
    //   {
    //     ptu_pub.publish(cmd);
    //   }
    // }
  }

public:

  void onStart()
  {
    RLOG_CPP(5, "PtuComponent::onStart():: Subscribing to " << topicName);
    if (!nh)
    {
      RLOG(0, "Creating node handle");
      nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
    }

    RLOG_CPP(5, "Subscribing node handle " << topicName);
    this->jntSubscriber = nh->subscribe(topicName, 1,
                                        &PtuComponent::rosCallback, this);
    RLOG(5, "Done subscribing");

    waitForInitialization();

    // From here on, pan and tilt targets are initialized
    RLOG(0, "PTU initialized");
    this->ptu_pub = nh->advertise<sensor_msgs::JointState>("/ptu/velocity_command", 1);
  }

protected:

  virtual void waitForInitialization()
  {
    while (!desiredValuesInitialized)
    {
      usleep(10000);
      RLOG(0, "Waiting for PTU initialization from ROS");
    }
  }

  void onStop()
  {
    RLOG(0, "PtuComponent::onStop()");
    if (nh)
    {
      this->jntSubscriber.shutdown();
    }

    this->desiredValuesInitialized = false;
  }

  std::string topicName;
  std::string panJointName;
  std::string tiltJointName;
  double panJointAngle;
  double tiltJointAngle;
  Rcs::RampFilter1D* panFilt;
  Rcs::RampFilter1D* tiltFilt;
  int panJointIdx;
  int tiltJointIdx;
  ros::Subscriber jntSubscriber;
  ros::Publisher ptu_pub;
  std::unique_ptr<ros::NodeHandle> nh;
  std::mutex mtx;
  bool desiredValuesInitialized;
};

#else
class PtuComponent : public ComponentBase
{
public:
  static PtuComponent* create(EntityBase* parent)
  {
    RFATAL("You are trying to use the PTU, but it has not been compiled in");
    return NULL;
  }
};
#endif


/*******************************************************************************
 *
 ******************************************************************************/
#if defined (USE_FRI)
static ComponentFactoryRegistrar regFRI("RoboFriComponent");
class RoboFriComponent : public ComponentBase, public Rcs::FriComponent
{
public:

  static RoboFriComponent* create(EntityBase* parent, const RcsGraph* graph,
                                  const std::string& robo)
  {
    return new RoboFriComponent(parent, graph, robo);
  }

  RoboFriComponent(EntityBase* parent, const RcsGraph* graph,
                   const std::string& robo) :
    ComponentBase(parent), Rcs::FriComponent(RcsGraph_clone(graph), robo),
    emergencyFlagPrev(false), eStop(false)
  {
    subscribe("Start", &RoboFriComponent::start);
    subscribe("Stop", &RoboFriComponent::stop);
    subscribe("UpdateGraph", &RoboFriComponent::onUpdateGraph);
    subscribe("SetJointCommand", &FriComponent::setJointPosition);
    subscribe("EnableCommands", &RoboFriComponent::enableCommands);
    subscribe<std::string, std::vector<double> >("SetCompliance", &RoboFriComponent::setCompliance);
    subscribe<std::vector<double> >("SetComplianceCommand", &RoboFriComponent::setComplianceCommand);
    subscribe("EmergencyStop", &RoboFriComponent::onEmergencyStop);
    subscribe("EmergencyRecover", &RoboFriComponent::onEmergencyRecover);
  }

  void start()
  {
    RLOG(1, "RoboFriComponent::start()");
    Rcs::FriComponent::start();
  }

  void enableCommands()
  {
    if (this->eStop == false)
    {
      RLOG(1, "RoboFriComponent::enableCommands()");
      setCommandsEnabled(true);
    }
    else
    {
      RLOG(0, "Can't enable commands during emergency stop");
    }
  }

  void setCompliance(std::string side, std::vector<double> comp)
  {

    if (side == this->sideName)
    {
      //      RLOG(0, "Called setCompliance with side: %s", side.c_str());
      //      RLOG(0, "%5.2f, %5.2f, %5.2f,    %5.2f, %5.2f, %5.2f", comp[0], comp[1], comp[2], comp[3], comp[4], comp[5]);

      double compliance[6];
      compliance[0] = comp[0];
      compliance[1] = comp[1];
      compliance[2] = comp[2];
      compliance[3] = comp[3];
      compliance[4] = comp[4];
      compliance[5] = comp[5];


      Rcs::FriComponent::setCompliance(compliance);
    }
  }

  void setComplianceCommand(std::vector<double> comp)
  {
    setCompliance("right", comp);

    comp.erase(comp.begin(), comp.begin()+6);
    setCompliance("left", comp);
  }

  void onUpdateGraph(RcsGraph* graph)
  {
    bool emergencyFlag = checkEmergencyCondition();

    if ((emergencyFlag==true) && (emergencyFlagPrev==false))
    {
      getEntity()->call("EmergencyStop");
    }

    Rcs::FriComponent::updateGraph(graph);

    emergencyFlagPrev = emergencyFlag;
  }

  void onEmergencyStop()
  {
    if (this->eStop == false)
    {
      RLOG(0, "RoboFriComponent::EmergencyStop");
      Rcs::FriComponent::onEmergencyStop(NULL);
      setCommandsEnabled(false);
    }

    this->eStop = true;
  }

  void onEmergencyRecover()
  {
    RLOG(0, "RoboFriComponent::EmergencyRecover");
    Rcs::FriComponent::onEmergencyRecovery();
    this->emergencyFlagPrev = false;
    setCommandsEnabled(true);
    this->eStop = false;
  }



private:
  bool emergencyFlagPrev;
  bool eStop;
};

#else
class RoboFriComponent : public ComponentBase
{
public:

  static RoboFriComponent* create(EntityBase* parent, const RcsGraph* graph,
                                  const std::string& robo)
  {
    RFATAL("You are trying to use the FRI, but it has not been compiled in");
    return NULL;
  }

};

#endif   // USE_FRI

/*******************************************************************************
 *
 ******************************************************************************/
#if defined (USE_SDH)
static ComponentFactoryRegistrar regSDH("RoboSDHComponent");
class RoboSDHComponent : public ComponentBase, public Rcs::SDHComponent
{
public:

  static RoboSDHComponent* create(EntityBase* parent, const RcsGraph* graph,
                                  const char* tcpAddress, const char* suffix)
  {
    return new RoboSDHComponent(parent, graph, tcpAddress, suffix);
  }

  RoboSDHComponent(EntityBase* parent, const RcsGraph* graph,
                   const char* tcpAddress, const char* suffix) :
    ComponentBase(parent), Rcs::SDHComponent(tcpAddress), eStop(false)
  {
    RLOG(0, "Creating RoboSDHComponent");
    setUpdateFrequency(10.0);

    // Create joint map for SDH
    for (int i=0; i<SDH_DOF_ALL; i++)
    {
      char jName[32];
      strcpy(jName, getJointName(i));
      if (suffix != NULL)
      {
        strcat(jName, suffix);
      }

      const RcsJoint* jnt = RcsGraph_getJointByName(graph, jName);

      if (jnt==NULL)
      {
        RLOG(1, "Couldn't find all SDH joints in graph: %s", jName);
        throw (std::string("Couldn't find all SDH joints in graph"));
      }

      this->jointMap[i] = jnt->jointIndex;
    }

    subscribe("Start", &RoboSDHComponent::start);
    subscribe("Stop", &RoboSDHComponent::stop);
    subscribe("UpdateGraph", &RoboSDHComponent::updateGraph);
    subscribe("SetJointCommand", &RoboSDHComponent::setJointPosition);
    subscribe("InitFromState", &RoboSDHComponent::onInitFromState);
    subscribe("EmergencyStop", &RoboSDHComponent::onEmergencyStop);
    subscribe("EmergencyRecover", &RoboSDHComponent::onEmergencyRecover);
    subscribe("EnableCommands", &RoboSDHComponent::enableCommands);
  }

private:

  void start()
  {
    RLOG(0, "RoboSDHComponent::start()");
    Rcs::SDHComponent::start();
  }

  void stop()
  {
    RLOG(0, "RoboSDHComponent::stop()");
    Rcs::SDHComponent::stop();
  }

  void onInitFromState(const RcsGraph* target)
  {
    RLOG(0, "RoboSDHComponent::onInitFromState()");
    setJointPosition(target->q);
  }

  void onEmergencyStop()
  {
    if (this->eStop == false)
    {
      RLOG(0, "RoboSDHComponent::EmergencyStop");
      setFingersFrozen(true);
    }

    this->eStop = true;
  }

  void onEmergencyRecover()
  {
    RLOG(0, "RoboSDHComponent::EmergencyRecover");
    setFingersEnabled(true);
    this->eStop = false;
  }

  void enableCommands()
  {
    RLOG(1, "RoboSDHComponent::enableCommands()");
    setFingersEnabled(true);
  }

  void toggleHandStiffness()
  {
    RLOG(1, "RoboSDHComponent::toggleHandStiffness()");
    setFingersEnabled(false);
  }

  bool eStop;
};
#else
class RoboSDHComponent : public ComponentBase
{
public:

  static RoboSDHComponent* create(EntityBase* parent, const RcsGraph* graph,
                                  const char* tcpAddress, const char* suffix)
  {
    RFATAL("RoboSDHComponent has not been compiled in");
    return NULL;
  }
};
#endif

/*******************************************************************************
 *
 ******************************************************************************/
#if defined (USE_LWA)
static ComponentFactoryRegistrar regLWA("RoboLWAComponent");
class RoboLWAComponent : public ComponentBase, public Rcs::LWAComponent
{
public:

  static RoboLWAComponent* create(EntityBase* parent, const RcsGraph* graph,
                                  const char* acInitString)
  {
    return new RoboLWAComponent(parent, graph, acInitString);
  }

  RoboLWAComponent(EntityBase* parent, const RcsGraph* graph,
                   const char* acInitString) :
    ComponentBase(parent), Rcs::LWAComponent(graph, acInitString), eStop(false)
  {
    RLOG(0, "Creating RoboLWAComponent");
    setUpdateFrequency(100.0);

    subscribe("Start", &RoboLWAComponent::start);
    subscribe("Stop", &RoboLWAComponent::stop);
    subscribe("UpdateGraph", &RoboLWAComponent::updateGraph);
    subscribe("SetJointCommand", &RoboLWAComponent::setJointPosition);
    subscribe("InitFromState", &RoboLWAComponent::onInitFromState);
    subscribe("EmergencyStop", &RoboLWAComponent::onEmergencyStop);
    subscribe("EmergencyRecover", &RoboLWAComponent::onEmergencyRecover);
    subscribe("EnableCommands", &RoboLWAComponent::enableCommands);
  }

private:

  void start()
  {
    RLOG(0, "RoboLWAComponent::start()");
    Rcs::LWAComponent::startThread();
  }

  void stop()
  {
    RLOG(0, "RoboLWAComponent::stop()");
    Rcs::LWAComponent::stopThread();
  }

  void onInitFromState(const RcsGraph* target)
  {
    RLOG(0, "RoboSDHComponent::onInitFromState()");
    double q_lwa[LWA_DOF];

    for (size_t i=0; i<LWA_DOF; ++i)
    {
      const RcsJoint* ji = RcsGraph_getJointByName(target, getJointName(i));
      RCHECK_MSG(ji, "Failed to find joint \"%s\" in graph", getJointName(i));
      q_lwa[i] = MatNd_get(target->q, ji->jointIndex, 0);
    }

    setDesiredJointAngles(q_lwa);
  }

  void onEmergencyStop()
  {
    if (this->eStop == false)
    {
      RLOG(0, "RoboLWAComponent::EmergencyStop");
    }

    this->eStop = true;
  }

  void onEmergencyRecover()
  {
    RLOG(0, "RoboSDHComponent::EmergencyRecover");
    this->eStop = false;
  }

  void enableCommands()
  {
    RLOG(1, "RoboSDHComponent::enableCommands()");
    setWriteCommands(true);
  }

  void setJointPosition(const MatNd* q_des)
  {
    setCommand(q_des, NULL, NULL);
  }

  bool eStop;
};
#else
class RoboLWAComponent : public ComponentBase
{
public:

  static RoboLWAComponent* create(EntityBase* parent, const RcsGraph* graph,
                                  const char* acInitString)
  {
    RFATAL("RoboLWAComponent has not been compiled in");
    return NULL;
  }
};
#endif

/*******************************************************************************
 *
 ******************************************************************************/
#if defined (USE_CANOPENCONTROL)
static ComponentFactoryRegistrar regIME("RoboIMEComponent");
class RoboIMEComponent : public ComponentBase, public Rcs::IMEComponent
{
public:

  static RoboIMEComponent* create(EntityBase* parent, const RcsGraph* graph,
                                  bool useKalmanFilter)
  {
    RoboIMEComponent* ime = NULL;

    try
    {
      ime = new RoboIMEComponent(parent, graph, useKalmanFilter);
    }
    catch (const char* str)
    {
      RLOG(1, "Caught exception from IME: %s", str);
    }
    catch (...)
    {
      RLOG(1, "Caught unknown exception from IME");
    }

    return ime;
  }

  RoboIMEComponent(EntityBase* parent, const RcsGraph* graph,
                   bool useKalmanFilter) :
    ComponentBase(parent), Rcs::IMEComponent(graph, parent->getDt()),
    useViconKalmanFilter(useKalmanFilter),eStop(false), eStopPublished(false)
  {

    if (useKalmanFilter==true)
    {
      subscribe("UpdateGraph", &IMEComponent::updateGraph);
      subscribe<const MatNd*>("SetJointCommand", &RoboIMEComponent::onSetCommand);
    }
    else
    {
      subscribe("UpdateGraph", &RoboIMEComponent::updateGraphFeedForward);
      subscribe<const MatNd*>("SetJointCommand", &RoboIMEComponent::setCommandFeedForward);
    }

    subscribe("ResetErrorFlag", &RoboIMEComponent::onResetErrorFlag);
    subscribe("InitFromState", &IMEComponent::onInitFromState);
    subscribe("EnableCommands", &RoboIMEComponent::onEnableCommands);
    subscribe("EmergencyStop", &RoboIMEComponent::onEmergencyStop);
    subscribe("EmergencyRecover", &RoboIMEComponent::onEmergencyRecover);
    //subscribe("Render", &RoboIMEComponent::onRender);
    subscribe("ComputeKinematics", &RoboIMEComponent::onCheckEmergencyStop);
  }

  // \todo(MG, FT): Type deduction fails here for some reason.
  void onSetCommand(const MatNd* q_des)
  {
    if (eStop==false)
    {
      Rcs::IMEComponent::setCommand(q_des);
    }
  }

  void onResetErrorFlag()
  {
    RLOG(0, "Resetting IME error flag.");
    resetErrorFlag();
  }

  void onEnableCommands()
  {
    if (eStop==false)
    {
      RLOG(0, "RoboIMEComponent::enableCommands()");
      this->enableCommands = true;
    }
    else
    {
      RLOG(0, "Can't enable commands during emergency stop");
    }
  }

  void onEmergencyStop()
  {
    if (eStop==false)
    {
      RLOG(0, "RoboIMEComponent::EmergencyStop");
      IMEComponent::onEmergencyStop(NULL);
    }

    this->eStop = true;
  }

  void onEmergencyRecover()
  {
    RLOG(0, "RoboIMEComponent::EmergencyRecover");
    this->enableCommands = true;
    this->velocityLimitExceeded = false;
    this->eStop = false;
    this->eStopPublished = false;
  }

  void onCheckEmergencyStop(RcsGraph* current)
  {
    if (this->eStopPublished == true)
    {
      NLOG(0, "this->eStopPublished == true");
      return;
    }

    const RcsBody* viconIME = RcsGraph_getBodyByName(current, "Vicon IME");

    if (useViconKalmanFilter && (viconIME==NULL))
    {
      RLOG(0, "Couldn't find VICON markers for IME - calling emergency stop");
      getEntity()->call("EmergencyStop");
      this->eStopPublished = true;
    }

    // A confidence of 0 means: occluded, or not contained in the most
    // recent frame
    if (useViconKalmanFilter && (viconIME->confidence==0.0))
    {
      RLOG(0, "IME Vicon marker tracking failed");
      getEntity()->call("EmergencyStop");
      this->eStopPublished = true;
    }

    if (velocityLimitExceeded==true)
    {
      RLOG(0, "IME velocity limit exceeded");
      getEntity()->call("EmergencyStop");
      this->eStopPublished = true;
    }

    ImeControlState state;
    ImeControl_getState(this->ime, &state);

    if (ImeControlState_isSafetyScannerStopArea(&state))
    {
      RLOG(0, "Laser stop area intruded");
      getEntity()->call("EmergencyStop");
      this->eStopPublished = true;
    }

  }

  void onRender()
  {
    if (this->cmdCount % 10 == 0)
    {
      ImeControlState state;
      ImeControl_getState(this->ime, &state);
      std::stringstream ss;
      ss << "IME status: " << std::endl;

      if (ImeControlState_isSafetyScannerWarningArea(&state))
      {
        ss << "IME status: ";
      }

      if (ImeControlState_isSafetyScannerStopArea(&state))
      {
        ss << "Safety stop, ";
      }

      if (ImeControlState_isStandby(&state))
      {
        ss << "Standby, ";
      }

      if (ImeControlState_isWaitingForTargetValues(&state))
      {
        ss << "Waiting for target, ";
      }

      if (ImeControlState_isWaitingForEnable(&state))
      {
        ss << "Waiting for enable, ";
      }

      if (ImeControlState_isDriving(&state))
      {
        ss << "Driving, ";
      }

      if (ImeControlState_isError(&state))
      {
        ss << "Error ";
      }

      ss << std::endl;
      getEntity()->publish("SetTextLine", ss.str(), 4);
    }
  }

private:

  bool useViconKalmanFilter;
  bool eStop;
  bool eStopPublished;
};

#else

class RoboIMEComponent : public ComponentBase
{
public:

  static RoboIMEComponent* create(EntityBase* parent, const RcsGraph* graph,
                                  bool useKalmanFilter=false)
  {
    RFATAL("You are trying to use the IME, but it has not been compiled in");
    return NULL;
  }

  void setKalmanGain(double gain)
  {
  }

  double getKalmanGain() const
  {
    return 0.0;
  }


};

#endif // USE_CANOPENCONTROL

/*******************************************************************************
 *
 ******************************************************************************/
#if defined (USE_CANOPENCONTROL)
static ComponentFactoryRegistrar regIEF("RoboIEFComponent");
class RoboIEFComponent : public ComponentBase, public Rcs::IEFComponent
{
public:

  static RoboIEFComponent* create(EntityBase* parent, const RcsGraph* graph)
  {
    return new RoboIEFComponent(parent, graph);
  }

  RoboIEFComponent(EntityBase* parent, const RcsGraph* graph) :
    ComponentBase(parent), Rcs::IEFComponent(graph), eStop(false)
  {
    subscribe("UpdateGraph", &IEFComponent::updateGraph);
    subscribe<const MatNd*>("SetJointCommand", &RoboIEFComponent::onSetCommand);
    subscribe<>("EmergencyStop", &RoboIEFComponent::onEmergencyStop);
    subscribe<>("EmergencyRecover", &RoboIEFComponent::onEmergencyRecover);
    subscribe("EnableCommands", &RoboIEFComponent::enableCommands);
  }

  void onEmergencyStop()
  {
    if (eStop==false)
    {
      RLOG(0, "RoboIEFComponent::EmergencyStop");
      Rcs::IEFComponent::onEmergencyStop(NULL);
    }

    this->eStop = true;
  }

  void onEmergencyRecover()
  {
    RLOG(0, "RoboIEFComponent::EmergencyRecover");
    this->commandsEnabled = true;
    this->eStop = false;
  }

  void enableCommands()
  {
    if (this->eStop == false)
    {
      RLOG(0, "RoboIEFComponent::EnableCommands");
      this->commandsEnabled = true;
    }
    else
    {
      RLOG(0, "Can't enable commands during emergency stop");
    }
  }

  // \todo(MG, FT): Type deduction fails here for some reason.
  void onSetCommand(const MatNd* q_des)
  {
    Rcs::IEFComponent::setCommand(q_des);
  }

  bool eStop;
};

#else

class RoboIEFComponent : public ComponentBase
{
public:

  static RoboIEFComponent* create(EntityBase* parent, const RcsGraph* graph)
  {
    RFATAL("You are trying to use the IEF, but it has not been compiled in");
    return NULL;
  }

};

#endif // USE_CANOPENCONTROL

/*******************************************************************************
 *
 ******************************************************************************/
#if defined (USE_CANOPENCONTROL)
static ComponentFactoryRegistrar regFTS("RoboFTSComponent");
class RoboFTSComponent : public ComponentBase, public Rcs::FTSComponent
{
public:

  static RoboFTSComponent* create(EntityBase* parent, const RcsGraph* graph,
                                  std::string deviceStr)
  {
    return new RoboFTSComponent(parent, graph, deviceStr);
  }

  RoboFTSComponent(EntityBase* parent, const RcsGraph* graph,
                   std::string deviceStr) :
    ComponentBase(parent), Rcs::FTSComponent(RcsGraph_clone(graph), deviceStr)
  {
    setUpdateFrequency(100.0);
    subscribe<>("Start", &RoboFTSComponent::onStart);
    subscribe<>("Stop", &RoboFTSComponent::onStop);
    subscribe("UpdateGraph", &FTSComponent::updateGraph);
    subscribe<>("Tare", &RoboFTSComponent::onTare);
    subscribe<>("Print", &RoboFTSComponent::onPrint);
    subscribe("ComputeKinematics", &RoboFTSComponent::updateSensorPose);
  }

  void onStart()
  {
    RLOG(0, "RoboFTSComponent::start()");
    startThread();
  }

  void onStop()
  {
    stopThread();
  }

  void onPrint()
  {
    MatNd_printCommentDigits("ft", getSensor()->rawData, 6);
  }

  void onTare()
  {
    RLOG(0, "Taring FT sensor '%s'", getSensorName());
    calibrate();
  }

  // Here we update the sensor's transformation according to the real state
  // of the system (coming through the ComputeKinematics event). To do it
  // perfectly, we should set the state from the other graph, and compute
  // the forward kinematics. This is a bit inefficient, therefore we only
  // copy over the sensor's transformation, assuming that in this class there
  // is no call to the forward kinematics anywhere else.
  // The copied transformation might be 1-2 iterations old. This is in the
  // order of magnitued of about 10-20 msec, so that we ignore it for now.
  void updateSensorPose(RcsGraph* other)
  {
    RcsBody* mountBdy = RCSBODY_BY_ID(currentGraph, getSensor()->bodyId);
    const char* mountBdyName = mountBdy->name;
    const RcsBody* otherFTS = RcsGraph_getBodyByName(other, mountBdyName);
    if (otherFTS==NULL)
    {
      RLOG(1, "Couldn't find body %s in graph - skipping FTS pose update", mountBdyName);
      return;
    }

    HTr_copy(&mountBdy->A_BI, &otherFTS->A_BI);
  }

};

#else

class RoboFTSComponent : public ComponentBase
{
public:

  static RoboFTSComponent* create(EntityBase* parent, const RcsGraph* graph,
                                  std::string deviceStr)
  {
    RFATAL("You are trying to use the FTS, but it has not been compiled in");
    return NULL;
  }

};

#endif // USE_CANOPENCONTROL

/*******************************************************************************
 *
 ******************************************************************************/
#if defined (USE_VICON)
static ComponentFactoryRegistrar regVICON("RoboViconComponent");
class RoboViconComponent : public ComponentBase, public Rcs::ViconComponent
{
public:

  static RoboViconComponent* create(EntityBase* parent, double quality)
  {
    RoboViconComponent* vicon = NULL;

    try
    {
      vicon = new RoboViconComponent(parent, quality);
    }
    catch (const char* str)
    {
      RLOG(1, "Caught exception from VICON: %s", str);
    }
    catch (...)
    {
      RLOG(1, "Caught unknown exception from VICON");
    }

    return vicon;
  }

  RoboViconComponent(EntityBase* parent, double quality=0.0) :
    ComponentBase(parent), Rcs::ViconComponent(quality)
  {
    setUpdateFrequency(100.0);
    subscribe("UpdateGraph", &ViconComponent::updateGraph);
    subscribe("UpdateGraph", &RoboViconComponent::printText);
    subscribe("Start", &RoboViconComponent::onStart);
    subscribe("Stop", &ViconComponent::stop);
    subscribe("Print", &ViconComponent::print);
  }

  void onStart()
  {
    RLOG(0, "RoboViconComponent::start()");
    start(getUpdateFrequency(), getThreadPriority());
  }

  void printText(RcsGraph* graph)
  {
    static size_t count = 0;
    count++;

    if (count%10==0)
    {
      mtx.lock();
      auto viconMapCopy = viconMap;
      mtx.unlock();

      std::stringstream ss;
      ss << std::setprecision(5);
      ss << viconMap.size() << " objects tracked" << std::endl;

      for (auto it=viconMapCopy.begin(); it!=viconMapCopy.end(); ++it)
      {
        ss << "Object \"" << it->first << "\": rejected: "
           << it->second.rejectedFrames << " quality: "
           << it->second.quality << " position: "
           << it->second.transform.org[0] << " "
           << it->second.transform.org[1] << " "
           << it->second.transform.org[2] << " " << std::endl;
      }

      getEntity()->publish("SetTextLine", ss.str(), 3);
    }
  }

};
#else

class RoboViconComponent : public ComponentBase
{
public:

  static RoboViconComponent* create(EntityBase* parent, double quality=0.0)
  {
    RFATAL("You are adding a ViconComponent, but it has not been compiled in");
    return NULL;
  }

};

#endif

/*******************************************************************************
 *
 ******************************************************************************/
#if defined (USE_JACO)
static ComponentFactoryRegistrar regJaco("RoboJacoComponent");
class RoboJacoComponent : public ComponentBase, public Rcs::JacoComponent
{
public:

  static RoboJacoComponent* create(EntityBase* parent, Rcs::JacoComponent::JacoType jt=Jaco6)
  {
    return new RoboJacoComponent(parent, jt);
  }

  RoboJacoComponent(EntityBase* parent, Rcs::JacoComponent::JacoType jt) :
    ComponentBase(parent), Rcs::JacoComponent(jt), eStop(false)
  {
    RLOG(0, "Creating RoboJacoComponent");
    subscribe("Start", &RoboJacoComponent::start);
    subscribe("Stop", &RoboJacoComponent::stop);
    subscribe("UpdateGraph", &RoboJacoComponent::onUpdateGraph);
    subscribe("SetJointCommand", &RoboJacoComponent::onSetJointPosition);
    subscribe("InitFromState", &RoboJacoComponent::onInitFromState);
    subscribe("EmergencyStop", &RoboJacoComponent::onEmergencyStop);
    subscribe("EmergencyRecover", &RoboJacoComponent::onEmergencyRecover);
    subscribe("EnableCommands", &RoboJacoComponent::enableCommands);
  }

private:

  void start()
  {
    RLOG(0, "RoboJacoComponent::start()");
    Rcs::JacoComponent::startThread();
  }

  void stop()
  {
    RLOG(0, "RoboJacoComponent::stop()");
    Rcs::JacoComponent::stopThread();
  }

  void onInitFromState(const RcsGraph* target)
  {
    RLOG(0, "RoboJacoComponent::onInitFromState()");
    onSetJointPosition(target->q);
  }

  void onEmergencyStop()
  {
    if (this->eStop == false)
    {
      RLOG(0, "RoboJacoComponent::EmergencyStop");
    }

    this->eStop = true;
  }

  void onEmergencyRecover()
  {
    RLOG(0, "RoboJacoComponent::EmergencyRecover");
    this->eStop = false;
  }

  void onSetJointPosition(const MatNd* q_des)
  {
    Rcs::JacoComponent::setCommand(q_des);
  }

  void onUpdateGraph(RcsGraph* graph)
  {
    Rcs::JacoComponent::updateSensors(graph);

    char text[256];
    snprintf(text, 256, "dt_filt: %.2f msec   dt_max: %.2f msec   force: %.3f",
             1000.0*getDtJacoFilt(), 1000.0*getDtJacoMax(), getHandForce());
    getEntity()->publish("SetTextLine", std::string(text), 2);

    const double handForce = getHandForce();

    if (handForce > 3.0)
    {
      getEntity()->publish("JacoHandForce", handForce);
    }
  }

  bool eStop;
};
#else
class RoboJacoComponent : public Rcs::JacoComponent
{
public:

  static RoboJacoComponent* create(EntityBase* parent,
                                   Rcs::JacoComponent::JacoType jt=Rcs::JacoComponent::Jaco6)
  {
    RFATAL("RoboJacoComponent has not been compiled in");
    return NULL;
  }
};
#endif

/*******************************************************************************
 *
 ******************************************************************************/
#if defined (USE_ROBOCOMPONENTS)
static ComponentFactoryRegistrar regJacoShm("RoboJacoShmComponent");
class RoboJacoShmComponent : public ComponentBase, public Rcs::JacoShmComponent
{
public:

  static RoboJacoShmComponent* create(EntityBase* parent,
                                      const RcsGraph* graph,
                                      Rcs::JacoShmComponent::JacoType jt)
  {
    return new RoboJacoShmComponent(parent, graph, jt);
  }

  RoboJacoShmComponent(EntityBase* parent,
                       const RcsGraph* graph,
                       Rcs::JacoShmComponent::JacoType jt) :
    ComponentBase(parent), Rcs::JacoShmComponent(graph, jt), eStop(false), enableCommands(false)
  {
    RLOG(0, "Creating RoboJacoShmComponent");
    subscribe("UpdateGraph", &RoboJacoShmComponent::onUpdateGraph);
    subscribe("SetJointCommand", &RoboJacoShmComponent::onSetJointPosition);
    subscribe("InitFromState", &RoboJacoShmComponent::onInitFromState);
    subscribe("EmergencyStop", &RoboJacoShmComponent::onEmergencyStop);
    subscribe("EmergencyRecover", &RoboJacoShmComponent::onEmergencyRecover);
    subscribe("EnableCommands", &RoboJacoShmComponent::onEnableCommands);
  }

private:

  void onInitFromState(const RcsGraph* target)
  {
    RLOG(0, "RoboJacoComponent::onInitFromState()");
    onSetJointPosition(target->q);
  }

  void onEmergencyStop()
  {
    if (this->eStop == false)
    {
      RLOG(0, "RoboJacoComponent::EmergencyStop");
    }

    this->eStop = true;
    enableCommands = false;
  }

  void onEmergencyRecover()
  {
    RLOG(0, "RoboJacoComponent::EmergencyRecover");
    this->eStop = false;
    enableCommands = true;
  }

  void onSetJointPosition(const MatNd* q_des)
  {
    if ((enableCommands) && (!eStop))
    {
      Rcs::JacoShmComponent::setCommand(q_des);
    }
  }

  void onUpdateGraph(RcsGraph* graph)
  {
    Rcs::JacoShmComponent::updateSensors(graph);
  }

  void onEnableCommands()
  {
    enableCommands = true;
  }

  bool eStop;
  bool enableCommands;
};
#else
class RoboJacoShmComponent : public Rcs::JacoShmComponent
{
public:

  static RoboJacoShmComponent* create(EntityBase* parent,
                                      const RcsGraph* graph,
                                      Rcs::JacoShmComponent::JacoType jt)
  {
    RFATAL("RoboJacoShmComponent has not been compiled in");
    return NULL;
  }
};
#endif

/*******************************************************************************
 *
 ******************************************************************************/
#if defined (USE_OPENCV_ARUCO)
static ComponentFactoryRegistrar regAruco("RoboArucoComponent");
class RoboArucoComponent : public ComponentBase, public Rcs::ArucoComponent
{
public:

  static RoboArucoComponent* create(EntityBase* parent)
  {
    return new RoboArucoComponent(parent);
  }

  RoboArucoComponent(EntityBase* parent) :
    ComponentBase(parent), Rcs::ArucoComponent()
  {
    RLOG(0, "Creating RoboArucoComponent");
    subscribe("Start", &RoboArucoComponent::start);
    subscribe("Stop", &RoboArucoComponent::stop);
    subscribe("InitFromState", &RoboArucoComponent::onInitFromState);
    subscribe("UpdateGraph", &RoboArucoComponent::onUpdateGraph);
    subscribe("RenderCommand", &RoboArucoComponent::onRenderCommand);
  }

private:

  void start()
  {
    RLOG(0, "RoboArucoComponent::start()");
    Rcs::ArucoComponent::startThread();
  }

  void stop()
  {
    RLOG(0, "RoboArucoComponent::stop()");
    Rcs::ArucoComponent::stopThread();
  }

  void onInitFromState(const RcsGraph* target)
  {
    RLOG(0, "RoboArucoComponent::onInitFromState()");
  }


  static bool HTr_lpf1d(HTr* A_filt, const HTr* A_new, double tmc)
  {

    // Filter position with simple first order low pass filter
    for (int i=0; i<3; i++)
    {
      A_filt->org[i] = tmc*A_new->org[i] + (1.0-tmc)*A_filt->org[i];
    }

    return Mat3d_firstOrderLPF((double (*)[3]) A_filt->rot, (double (*)[3]) A_new->rot, tmc);
  }

  void onUpdateGraph(RcsGraph* graph)
  {
    // Thread-safe copy of all detected markers
    grabberMtx.lock();
    auto markerMap = this->transformMap;
    grabberMtx.unlock();

    for (auto it=markerMap.begin(); it!=markerMap.end(); ++it)
    {
      RLOG_CPP(1, "Map contains " << it->first);
    }


    RCSGRAPH_TRAVERSE_BODIES(graph)
    {
      if (!BODY->rigid_body_joints)
      {
        continue;
      }



      if (STRCASEEQ(BODY->name, "Marker_0"))
      {
        auto it = markerMap.find(BODY->name);

        if (it != markerMap.end())
        {
          static bool filterInit = false;
          RCHECK_MSG(BODY->rigid_body_joints, "Marker %s has no rigid_body_joints", BODY->name);
          const HTr* A_MI = &it->second;

          HTr A_IM;
          HTr_transpose(&A_IM, A_MI);

          const RcsBody* cam = RcsGraph_getBodyByName(graph, "Camera");
          const RcsJoint* camJnt = RCSJOINT_BY_ID(graph, cam->jntId);
          double* q_rb = &graph->q->ele[camJnt->jointIndex];

          HTr A_CI_filt;
          const double tmc = 0.001;
          if (!filterInit)
          {
            HTr_transpose(&A_CI_filt, A_MI);
            filterInit = true;
          }
          else
          {
            HTr_from6DVector(&A_CI_filt, q_rb);
            HTr_lpf1d(&A_CI_filt, &A_IM, tmc);
          }

          const RcsBody* camParent = RCSBODY_BY_ID(graph, cam->parentId);
          RcsGraph_relativeRigidBodyDoFs(graph, cam, &A_CI_filt,   // &A_IM
                                         camParent ? &camParent->A_BI : NULL,
                                         q_rb);


          /* static int cnt = 0; */
          /* if (cnt++==250) */
          /* { */
          /*   osg::ref_ptr<COSNode> cn = new COSNode(0.8); */
          /*   cn->setName("CameraFrame"); */
          /*   cn->setTransformation(&A_CI_filt); */
          /*   getEntity()->publish<osg::ref_ptr<osg::Node>>("AddNode", cn); */
          /* } */
        }

      }




#if 1
      auto it = markerMap.find(BODY->name);

      if (it != markerMap.end())
      {

        if (STREQ(BODY->name, "Marker_0"))
        {
          continue;
        }


        static bool filterInit2 = false;

        RLOG(1, "Updating body %s", BODY->name);
        RCHECK_MSG(BODY->rigid_body_joints, "Marker %s has no rigid_body_joints", BODY->name);
        const RcsJoint* bdyJnt = RCSJOINT_BY_ID(graph, BODY->jntId);
        double* q_rb = &graph->q->ele[bdyJnt->jointIndex];
        const HTr* A_MI = &it->second;

        HTr A_MI_filt;
        const double tmc = 0.01;
        if (!filterInit2)
        {
          HTr_copy(&A_MI_filt, A_MI);
          filterInit2 = true;
        }
        else
        {
          HTr_from6DVector(&A_MI_filt, q_rb);
          HTr_lpf1d(&A_MI_filt, A_MI, tmc);
        }

        HTr_to6DVector(q_rb, &A_MI_filt);
      }
      else
      {
        RLOG_CPP(1, "Not found: " << BODY->name);
      }
#endif

    }






  }

  void onRenderCommand(std::string graphId, std::string command)
  {
    if (!STRCASEEQ("Aruco", graphId.c_str()))
    {
      return;
    }

    if (STRCASEEQ("hide", command.c_str()))
    {
      setEnableCameraWindow(false);
    }
    else if (STRCASEEQ("show", command.c_str()))
    {
      setEnableCameraWindow(true);
    }

  }

};
#else
class RoboArucoComponent : public ComponentBase
{
public:

  static RoboArucoComponent* create(EntityBase* parent)
  {
    RFATAL("RoboArucoComponent has not been compiled in");
    return NULL;
  }
};
#endif

/*******************************************************************************
 *
 ******************************************************************************/
void printHardwareComponents()
{
  ComponentFactory::print();
}

/*******************************************************************************
 *
 ******************************************************************************/
std::vector<ComponentBase*> getHardwareComponents(EntityBase& entity,
                                                  const RcsGraph* graph,
                                                  bool dryRun=false)
{
  Rcs::CmdLineParser argP;
  std::vector<ComponentBase*> hwc;



  if (argP.hasArgument("-ros", "Initialize ROS before any component") && (!dryRun))
  {
    initROS();
  }

  if (argP.hasArgument("-lwa", "Start with LWA") && (!dryRun))
  {
    std::string acInitStr = "ESD:3, 1000";
    argP.getArgument("-acInitString", &acInitStr, "LWA initialization string"
                     "(default is \"%s\")", acInitStr.c_str());
    auto c = RoboLWAComponent::create(&entity, graph, acInitStr.c_str());
    hwc.push_back(c);
  }

  if (argP.hasArgument("-sdh1", "Start with SDH1") && (!dryRun))
  {
    auto c = RoboSDHComponent::create(&entity, graph, "10.107.145.101", "_R");
    hwc.push_back(c);
  }

  if (argP.hasArgument("-sdh2", "Start with SDH2") && (!dryRun))
  {
    auto c = RoboSDHComponent::create(&entity, graph, "10.107.145.102", "_L");
    hwc.push_back(c);
  }

  if (argP.hasArgument("-fts1", "Start with FTS1") && (!dryRun))
  {
    auto c = RoboFTSComponent::create(&entity, graph, "WristLoadCellLBR_R");
    hwc.push_back(c);
  }

  if (argP.hasArgument("-fts2", "Start with FTS2") && (!dryRun))
  {
    auto c = RoboFTSComponent::create(&entity, graph, "WristLoadCellLBR_L");
    hwc.push_back(c);
  }

  if (argP.hasArgument("-ief", "Start with IEF Werner linear slide") && (!dryRun))
  {
    auto c = RoboIEFComponent::create(&entity, graph);
    hwc.push_back(c);
  }

  if (argP.hasArgument("-ime", "Start with IME omni platform") && (!dryRun))
  {
    RFATAL("Use -ime_odometry or -ime_vicon");
  }

  if (argP.hasArgument("-ime_odometry", "Start with IME omni platform") && (!dryRun))
  {
    // Third argument is false for odometry integration only
    hwc.push_back(RoboIMEComponent::create(&entity, graph, false));
  }

  if (argP.hasArgument("-ime_vicon", "Start with IME omni platform") && (!dryRun))
  {
    // Third argument is true for VICON feedback
    auto* c = RoboIMEComponent::create(&entity, graph, true);

    if (c)
    {
      double kalmanGain = c->getKalmanGain();
      argP.getArgument("-kalmanGain", &kalmanGain, "Kalman filter gain "
                       "(default is %f)", kalmanGain);
      c->setKalmanGain(kalmanGain);
      hwc.push_back(c);
    }
  }

  if (argP.hasArgument("-lbr1", "Start with LBR1") && (!dryRun))
  {
    hwc.push_back(RoboFriComponent::create(&entity, graph, "Right"));
  }

  if (argP.hasArgument("-lbr2", "Start with LBR2") && (!dryRun))
  {
    hwc.push_back(RoboFriComponent::create(&entity, graph, "Left"));
  }

  if (argP.hasArgument("-vicon", "Start with Vicon") && (!dryRun))
  {
    hwc.push_back(RoboViconComponent::create(&entity, 0.0));
  }

  if (argP.hasArgument("-jaco", "Start with Jaco") && (!dryRun))
  {
    hwc.push_back(RoboJacoComponent::create(&entity));
  }

  if (argP.hasArgument("-jaco7r", "Start with Jaco7 right") && (!dryRun))
  {
    hwc.push_back(RoboJacoComponent::create(&entity, Rcs::JacoComponent::Jaco7_right));
  }

  if (argP.hasArgument("-jaco7l", "Start with Jaco7 left") && (!dryRun))
  {
    hwc.push_back(RoboJacoComponent::create(&entity, Rcs::JacoComponent::Jaco7_left));
  }

  if (argP.hasArgument("-jacoShm7r", "Start with Jaco7 Shm right") && (!dryRun))
  {
    hwc.push_back(RoboJacoShmComponent::create(&entity, graph, Rcs::JacoShmComponent::Jaco7_right));
  }

  if (argP.hasArgument("-jacoShm7l", "Start with Jaco7 Shm left") && (!dryRun))
  {
    hwc.push_back(RoboJacoShmComponent::create(&entity, graph, Rcs::JacoShmComponent::Jaco7_left));
  }

  if (argP.hasArgument("-aruco", "Start with Aruco marker tracking") && (!dryRun))
  {
    hwc.push_back(RoboArucoComponent::create(&entity));
  }

  if (argP.hasArgument("-ptu", "Start with Scitos PTU") && (!dryRun))
  {
    hwc.push_back(PtuComponent::create(&entity));
  }

  for (size_t i=0; i<hwc.size(); ++i)
  {
    RCHECK_MSG(hwc[i], "Found NULL hardware component at index %zu", i);
  }

  return hwc;
}

}   // namespace

#endif //AFF_ECSHARDWARECOMPONENTS_H
