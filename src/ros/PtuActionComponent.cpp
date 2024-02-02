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

/*
PARAMETERS
 * /ptu/controller/pan_max_degrees: 30
 * /ptu/controller/pan_min_degrees: -30
 * /ptu/controller/tilt_max_degrees: 40
 * /ptu/controller/tilt_min_degrees: -10
 * /rosdistro: noetic
 * /rosversion: 1.15.9

  Pan_velocity[deg/s] 0.1186208426952362 not in range [1, 70].
  Tilt_velocity[deg/s] -30.0 not in range [1, 40].

 */

#include "PtuActionComponent.h"

#if defined (USE_ROS)

#include "Agent.h"

#include <TaskFactory.h>
#include <IkSolverRMR.h>
#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_basicMath.h>

#include <thread>

namespace aff
{

void PtuActionComponent::doneCallback(const actionlib::SimpleClientGoalState& state,
                                      const hri_scitos_schunk_ptu::PtuGotoResultConstPtr& result)
{
  RLOG(0, "Action server responded with state [%s]", state.toString().c_str());
  getEntity()->publish("EnableASR", true);
  getEntity()->publish("EnableSoundDirection", true);
}

void PtuActionComponent::activeCallback()
{
  RLOG(0, "Action is active");
  getEntity()->publish("EnableASR", false);
  getEntity()->publish("EnableSoundDirection", false);
}

PtuActionComponent::PtuActionComponent(EntityBase* parent) :
  ComponentBase(parent), actionClient("/ptu/goto", true),
  panJointName("ptu_pan_joint"), tiltJointName("ptu_tilt_joint"),
  panTiltUpdated(false), copyOfGraph(NULL),
  pan_min_degrees(0.0), pan_max_degrees(0.0), tilt_min_degrees(0.0), tilt_max_degrees(0.0)
{
  subscribe("Start", &PtuActionComponent::onStart);
  subscribe("Stop", &PtuActionComponent::onStop);
  subscribe("PtuPanTiltCommand", &PtuActionComponent::onPtuCommand);
  subscribe("PtuLookAtCommand", &PtuActionComponent::onLookAtCommand);
  subscribe("UpdateGraph", &PtuActionComponent::onUpdateGraph);
  subscribe("PostUpdateGraph", &PtuActionComponent::onPostUpdateGraph);
}

PtuActionComponent::~PtuActionComponent()
{
}

void PtuActionComponent::onStart()
{
  RLOG(0, "Waiting for server");
  ros::Duration waitForServerTimeout(30.0);
  bool foundServer = actionClient.waitForServer(waitForServerTimeout);

  if (!foundServer)
  {
    ROS_WARN("could not connect to server; halting");
  }

  if (!nh)
  {
    RLOG(0, "Creating node handle");
    nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  }

  nh->getParam("/ptu/controller/pan_min_degrees", pan_min_degrees);
  nh->getParam("/ptu/controller/pan_max_degrees", pan_max_degrees);
  nh->getParam("/ptu/controller/tilt_min_degrees", tilt_min_degrees);
  nh->getParam("/ptu/controller/tilt_max_degrees", tilt_max_degrees);

  RLOG(0, "Pan range: [%.2f..%.2f]  Tilt range: [%.2f..%.2f] (in degrees)",
       pan_min_degrees, pan_max_degrees, tilt_min_degrees, tilt_max_degrees);

  RLOG_CPP(5, "Subscribing to topic /ptu/state");
  this->ptuPanTiltSubscriber = nh->subscribe("/ptu/state", 1, &PtuActionComponent::rosCallback, this);
  RLOG(5, "Done subscribing");

  RLOG(0, "Connected to action server");
}

void PtuActionComponent::onStop()
{
  RLOG(0, "PtuActionComponent::onStop()");
  actionClient.cancelAllGoals();

  if (nh)
  {
    this->ptuPanTiltSubscriber.shutdown();
  }
}

void PtuActionComponent::onPtuCommand(double panInRad, double tiltInRad)
{
  if (!nh)
  {
    RLOG_CPP(1, "ROS not initialized - not sending pan/tilt command");
    return;
  }

  const double safetyAngInDeg = 1.0;
  hri_scitos_schunk_ptu::PtuGotoGoal goal;
  goal.pan_degrees = Math_clip(RCS_RAD2DEG(panInRad), pan_min_degrees+safetyAngInDeg, pan_max_degrees-safetyAngInDeg);
  goal.tilt_degrees = Math_clip(RCS_RAD2DEG(tiltInRad), tilt_min_degrees+safetyAngInDeg, tilt_max_degrees-safetyAngInDeg);

  // Determine velocity so that pan and tilt joints get to goal synchronously
  const double velMaxInDeg = 30.0;
  const double dPhiPan = goal.pan_degrees - RCS_RAD2DEG(panJointAngle);
  const double dPhiTilt = goal.tilt_degrees - RCS_RAD2DEG(tiltJointAngle);
  const double dPhiMax = std::max(fabs(dPhiPan), fabs(dPhiTilt));
  const double scalePan = fabs(dPhiPan/dPhiMax);
  const double scaleTilt = fabs(dPhiTilt/dPhiMax);

  goal.pan_velocity_degrees = velMaxInDeg;
  goal.tilt_velocity_degrees = velMaxInDeg;

  if (scalePan > 1.0 && scaleTilt>1.0)
  {
    goal.pan_velocity_degrees *= scalePan;
    goal.tilt_velocity_degrees *= scaleTilt;
  }

  RLOG(1, "Setting angles to %.1f and %.1f (in degrees)",
       goal.pan_degrees, goal.tilt_degrees);
  RLOG(1, "Setting velocities to %.1f and %.1f (in degrees)",
       goal.pan_velocity_degrees, goal.tilt_velocity_degrees);

  actionClient.sendGoal(goal,
                        boost::bind(&PtuActionComponent::doneCallback, this, _1, _2),
                        boost::bind(&PtuActionComponent::activeCallback, this));
}

void PtuActionComponent::onLookAtCommand(const std::string& bodyName)
{
  if (!copyOfGraph)
  {
    RLOG_CPP(1, "Graph currently unknown - can't look at body " << bodyName);
    return;
  }

  std::lock_guard<std::mutex> lock(panTiltMtx);
  this->lookAtBody = bodyName;
}

void PtuActionComponent::getPanTilt(std::string gazeTarget)
{
#if 1
  double panTilt[2], err[2];
  size_t maxIter = 100;
  double eps = 1.0e-8;

  int res = RobotAgent::getPanTilt(copyOfGraph, gazeTarget, panTilt, maxIter, eps, err);

  if ((err[0]<eps) && (err[1]<eps))
  {
    getEntity()->publish("PtuPanTiltCommand", panTilt[0], panTilt[1]);
  }

#else

  // Here we can safely assume that copyOfGraph exists.
  const RcsBody* gazeBdy = RcsGraph_getBodyByName(copyOfGraph, gazeTarget.c_str());
  if (!gazeBdy)
  {
    RLOG(1, "gaze body '%s' not found", gazeTarget.c_str());
    return;
  }

  const RcsJoint* panJoint = RcsGraph_getJointByName(copyOfGraph, panJointName.c_str());
  if (!panJoint)
  {
    RLOG(1, "Pan joint 'ptu_pan_joint' not found");
    return;
  }

  const RcsJoint* tiltJoint = RcsGraph_getJointByName(copyOfGraph, tiltJointName.c_str());
  if (!tiltJoint)
  {
    RLOG(1, "Tilt joint 'ptu_tilt_joint' not found");
    return;
  }

  Rcs::ControllerBase controller(copyOfGraph);   // Takes ownership of graph
  controller.setGraphOwnership(false);   // But it mustn't delete it

  std::string cameraFrame = "head_kinect_rgb_link";
  std::string xmlTask = "<Task name=\"Gaze\" controlVariable=\"XY\" effector=\"" +
                        gazeTarget + "\" " + "refBdy=\"" + cameraFrame + "\" />";

  Rcs::Task* task = Rcs::TaskFactory::createTask(xmlTask, controller.getGraph());
  if (!task)
  {
    RLOG(1, "Gazing task could not be created");
    return;
  }
  controller.add(task);

  Rcs::IkSolverRMR ikSolver(&controller);

  MatNd* dq_des = MatNd_createLike(copyOfGraph->q);
  MatNd* x_des = MatNd_create(2,1);
  MatNd* dx_des = MatNd_create(2,1);
  MatNd* dH = NULL;
  MatNd* a = NULL;
  double err[2];
  const double lambda = 1.0e-8;
  const double eps = 1.0e-6;
  const size_t maxIter = 200;
  double clipLimit = 0.01;

  for (int iter=0; iter<maxIter; ++iter)
  {
    controller.computeDX(dx_des, x_des);
    MatNd clipArr = MatNd_fromPtr(1, 1, &clipLimit);
    MatNd_saturateSelf(dx_des, &clipArr);

    if ((fabs(dx_des->ele[0])<eps) && (fabs(dx_des->ele[1])<eps))
    {
      err[0] = fabs(dx_des->ele[0]);
      err[1] = fabs(dx_des->ele[1]);
      break;
    }

    ikSolver.solveRightInverse(dq_des, dx_des, dH, a, lambda);
    MatNd_addSelf(copyOfGraph->q, dq_des);
    RcsGraph_setState(copyOfGraph, NULL, NULL);
  }

  // Clean up arrays
  MatNd_destroyN(3, dq_des, x_des, dx_des);

  // Only if the pan tilt has converged, we send the command.
  if ((err[0]<eps) && (err[1]<eps))
  {
    double panTilt[2];
    panTilt[0] = MatNd_get(copyOfGraph->q, panJoint->jointIndex, 0);
    panTilt[1] = MatNd_get(copyOfGraph->q, tiltJoint->jointIndex, 0);
    getEntity()->publish("PtuPanTiltCommand", panTilt[0], panTilt[1]);
  }

#endif
}

void PtuActionComponent::onPostUpdateGraph(RcsGraph* desired, RcsGraph* current)
{
  if (!this->copyOfGraph)
  {
    this->copyOfGraph = RcsGraph_clone(desired);
    RCHECK(this->copyOfGraph);
  }

  // Update the current pan-tilt angles in the desired graph.
  // \todo(MG): That should be consistently done in the current graph
  onUpdateGraph(desired);

  panTiltMtx.lock();
  std::string gazeTarget = this->lookAtBody;
  this->lookAtBody.clear();
  panTiltMtx.unlock();

  // Nothing to do if no gaze target has been sent
  if (gazeTarget.empty())
  {
    return;
  }

  // Update the graph
  RcsGraph_copy(this->copyOfGraph, desired);

  // From here on, we operate on the copied graph. This can go into a thread.
  std::thread t1(&PtuActionComponent::getPanTilt, this, gazeTarget);
  t1.detach();
}

void PtuActionComponent::onUpdateGraph(RcsGraph* graph)
{

  RCSGRAPH_TRAVERSE_JOINTS(graph)
  {
    if (STREQ(panJointName.c_str(), JNT->name))
    {
      NLOG(0, "q[%zu] = %f", JNT->jointIndex, panJointAngle);
      std::lock_guard<std::mutex> lock(panTiltMtx);
      MatNd_set(graph->q, JNT->jointIndex, 0, panJointAngle);
    }

    if (STREQ(tiltJointName.c_str(), JNT->name))
    {
      NLOG(0, "q[%zu] = %f", JNT->jointIndex, panJointAngle);
      std::lock_guard<std::mutex> lock(panTiltMtx);
      MatNd_set(graph->q, JNT->jointIndex, 0, tiltJointAngle);
    }

  }

}

void PtuActionComponent::rosCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

  for (size_t i=0; i<msg->name.size(); ++i)
  {

    if (msg->name[i]==panJointName)
    {
      NLOG(0, "Joint %zu is %s %f", i, msg->name[i].c_str(), msg->position[i]);
      std::lock_guard<std::mutex> lock(panTiltMtx);
      panJointAngle = msg->position[i];
      panTiltUpdated = true;
    }
    else if (msg->name[i]==tiltJointName)
    {
      NLOG(0, "Joint %zu is %s %f", i, msg->name[i].c_str(), msg->position[i]);
      std::lock_guard<std::mutex> lock(panTiltMtx);
      tiltJointAngle = msg->position[i];
      panTiltUpdated = true;
    }
    else
    {
      RLOG(0, "Joint %zu \"%s\" is not found", i, msg->name[i].c_str());
    }

  }
}

}   // namespace

#else  // namespace

namespace aff
{

PtuActionComponent::PtuActionComponent(EntityBase* parent) : ComponentBase(parent)
{
}

}

#endif   // defined USE_ROS
