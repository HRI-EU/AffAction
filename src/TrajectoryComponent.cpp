/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
  IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
  PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "TrajectoryComponent.h"

#include <TaskFactory.h>
#include <Rcs_typedef.h>
#include <Rcs_utils.h>
#include <Rcs_body.h>
#include <Rcs_shape.h>
#include <Rcs_joint.h>
#include <Rcs_timer.h>
#include <Rcs_macros.h>

#include <thread>
#include <mutex>

#define N_DOUBLES_IN_HTR   (sizeof(HTr)/sizeof(double))

using namespace tropic;

namespace aff
{

TrajectoryComponent::TrajectoryComponent(EntityBase* parent,
                                         Rcs::ControllerBase* controller,
                                         bool via,
                                         double horizon,
                                         bool checkTrajectory_) :
  ComponentBase(parent), tc(NULL), motionEndTime(0.0),
  lastMotionEndTime(0.0), motionDuration(0.0), a_des(NULL), x_des(NULL),
  tPred(NULL), animationGraph(NULL), animationTic(0),
  enableTrajectoryCheck(checkTrajectory_), enableDbgRendering(true),
  eStop(false)
{
  this->a_des = MatNd_create((int) controller->getNumberOfTasks(), 1);
  this->x_des = MatNd_create((int) controller->getTaskDim(), 1);
  controller->computeX(this->x_des);

  this->animationGraph = RcsGraph_clone(controller->getGraph());

  if (via)
  {
    tc = new TrajectoryController<ViaPointTrajectory1D>(controller, horizon);
  }
  else
  {
    tc = new TrajectoryController<ZigZagTrajectory1D>(controller, horizon);
  }

  // This is a sinister HACK to allow graph modifications to the controller
  // through ConstraintSets.
  tc->takeControllerOwnership(true);

  // Unfortunately we need this, otherwise there is a mismatch (small) between
  // prediction and execution. It seems that the task-level trajectory in some
  // cases is not matching perfectly. One reason might be due to the activation
  // of the tasks or so. This seems likely, since the error goes away if we do
  // not initialize the trajectory with the current velocity and acceleration
  // state.
  //tc->reactivateWithZeroVelAcc(true);
  //tc->readActivationsFromXML();
  controller->readActivationsFromXML(this->a_des);
  for (unsigned int i=0; i<a_des->m; ++i)
  {
    tc->setActivation(i, (a_des->ele[i]==0.0) ? false : true);
  }

  subscribe("Stop", &TrajectoryComponent::onStop);
  subscribe("ClearTrajectory", &TrajectoryComponent::onClearTrajectory);
  subscribe("EmergencyStop", &TrajectoryComponent::onEmergencyStop);
  subscribe("EmergencyRecover", &TrajectoryComponent::onEmergencyRecover);
  subscribe<const RcsGraph*>("InitFromState", &TrajectoryComponent::onInitFromState);
  subscribe<RcsGraph*>("ComputeTrajectory", &TrajectoryComponent::stepTrajectory);
  subscribe("CheckAndSetTrajectory", &TrajectoryComponent::onCheckAndSetTrajectory);
  subscribe("SetTrajectory", &TrajectoryComponent::onSetTrajectory);
  subscribe("SimulateTrajectory", &TrajectoryComponent::onSimulateTrajectory);
  subscribe("SetDebugRendering", &TrajectoryComponent::enableDebugRendering);
  subscribe("SetTrajectoryCheck", &TrajectoryComponent::onEnableTrajectoryCheck);
  subscribe("ChangeTaskVector", &TrajectoryComponent::onTaskVectorChangeSequential);
  subscribe("Print", &TrajectoryComponent::onPrint);
  subscribe("ToggleFastPrediction", &TrajectoryComponent::onResetAnimationTic);

  if (enableTrajectoryCheck)
  {
    subscribe("Render", &TrajectoryComponent::onRender);
  }
}

TrajectoryComponent::~TrajectoryComponent()
{
  // This is a sinister HACK to allow graph modifications to the controller
  // through ConstraintSets.
  tc->takeControllerOwnership(false);
  delete this->tc;
  MatNd_destroy(this->a_des);
  MatNd_destroy(this->x_des);
  MatNd_destroy(this->tPred);
  RcsGraph_destroy(this->animationGraph);
}

void TrajectoryComponent::stepTrajectory(RcsGraph* from)
{
  this->lastMotionEndTime = motionEndTime;
  this->motionEndTime = tc->step(getEntity()->getDt());

  const double phase = (motionDuration>0.0) ? 1.0 - (motionEndTime / motionDuration) : 0.0;
  const double phaseScale = sin(M_PI * phase);

  //RLOG(0, "Phase is %f", phaseScale);

  tc->getPosition(0.0, this->x_des);
  tc->getActivation(this->a_des);

  // RLOG(1, "lastMotionEndTime=%f   motionEndTime=%f", lastMotionEndTime, motionEndTime);

  if ((lastMotionEndTime > 0.0) && (motionEndTime == 0.0))
  {
    getEntity()->publish("TrajectoryMoving", false);
  }
  else if ((lastMotionEndTime == 0.0) && (motionEndTime > 0.0))
  {
    getEntity()->publish("TrajectoryMoving", true);
  }

  double blending = tc->computeBlending();

  // We disallow null space movements if there is no active task. This avoids null space
  // collisions during joint limit avoidance convergence.
  if (MatNd_maxAbsEle(this->a_des)<1.0e-8)
  {
    blending = 0.0;
  }
  getEntity()->publish("SetBlending", blending);
  getEntity()->publish("SetPhase", phaseScale);
}

void TrajectoryComponent::onClearTrajectory()
{
  RLOG(0, "TrajectoryComponent::clearTrajectory()");
  tc->clear(true);
}

const MatNd* TrajectoryComponent::getActivationPtr() const
{
  return this->a_des;
}

const MatNd* TrajectoryComponent::getTaskCommandPtr() const
{
  return this->x_des;
}

void TrajectoryComponent::onEmergencyStop()
{
  RLOG(0, "TrajectoryComponent::EmergencyStop");
  tc->clear();
  this->eStop = true;
  getEntity()->publish("ClearTrajectory");
  getEntity()->publish("ActionResult", false, 0.0, std::string("FATAL_ERROR REASON: Emergency stop triggered"));
}

void TrajectoryComponent::onEmergencyRecover()
{
  RLOG(0, "TrajectoryComponent::EmergencyRecover");
  this->eStop = false;
}

void TrajectoryComponent::onEnableTrajectoryCheck(bool enable)
{
  RLOG(0, "TrajectoryComponent::SetTrajectoryCheck");
  this->enableTrajectoryCheck = enable;
}

void TrajectoryComponent::onInitFromState(const RcsGraph* target)
{
  RLOG(1, "TrajectoryComponent::onInitFromState()");
  RcsGraph_copy(tc->getInternalController()->getGraph(), target);
  tc->getController()->computeX(this->x_des);
  tc->clear();
  tc->init();

  renderMtx.lock();
  enableDebugRendering(false);
  animationTic = 0;
  RcsGraph_copy(animationGraph, target);
  MatNd_destroy(tPred);
  tPred = NULL;
  renderMtx.unlock();
}

void TrajectoryComponent::onSimulateTrajectory(TCS_sptr tSet)
{
  // We always predict over the full trajectory, regardless if the check has
  // been disabled or not.
  std::shared_ptr<TrajectoryPredictor> pred = std::make_shared<TrajectoryPredictor>(tc);
  pred->setTrajectory(tSet);   // also clears it
  std::thread t1(&TrajectoryComponent::checkerThread, this, tSet, true, pred);
  t1.detach();
}

void TrajectoryComponent::onSetTrajectory(TCS_sptr tSet)
{
  REXEC(1)
  {
    tSet->toXML("onSetTrajectory.xml");
    tc->getController()->toXML("onSetController.xml");
  }

  RLOG(0, "Applying trajectory");
  tc->addAndApply(tSet);

  motionDuration = tSet->getDuration();
}

void TrajectoryComponent::onPrint()
{
  if (effectorTaskMap.empty())
  {
    RLOG_CPP(0, "effectorTaskMap is empty");
  }
  else
  {
    RLOG_CPP(0, "Printing effectorTaskMap:");
  }

  for (const auto& p : effectorTaskMap)
  {
    std::cout << "Effector '" << p.first << "' has these entries:" << std::endl;

    for (const auto& t : p.second)
    {
      std::cout << "  " << t << std::endl;
    }

  }

  RLOG_CPP(0, "TrajectoryController's internal controller:");
  size_t idx = 0;
  for (const auto& ti : tc->getInternalController()->taskVec())
  {
    RLOG_CPP(0, "Task " << idx++ << " : " << ti->getName());
    ti->print();
  }
  //tc->getInternalController()->print();

  RLOG_CPP(0, "TrajectoryController:");
  idx = 0;
  for (const auto& ti : tc->getTrajectories())
  {
    RLOG_CPP(0, "Trajectory " << idx++ << " : " << ti->getName());
  }
  //tc->print();
}

void TrajectoryComponent::onCheckAndSetTrajectory(TCS_sptr tSet)
{
  double t_calc = Timer_getSystemTime();

  // If an emergendy stop has been detected, or checking has been disabled,
  // we ignore incoming trajectories and return.
  if (this->eStop==true)
  {
    std::string resMsg = "FATAL_ERROR REASON: Emergency-Stop triggered";
    NLOG_CPP(0, resMsg);
    getEntity()->publish("ActionResult", false, 0.0, resMsg);
    return;
  }

  if (!enableTrajectoryCheck)
  {
    RLOG(1, "Skipping trajectory check - has been disabled");
    getEntity()->publish("SetTrajectory", tSet);
    // std::string resMsg = "Trajectory is valid but not checked";
    // getEntity()->publish("TrajectoryResult", true, 0.0, resMsg);
    return;
  }


  // We clone the TrajectoryController here, since this function is called from
  // within the ES event loop. The TrajectoryPredictor makes a deep copy of the
  // TrajectoryController. Doing the copying here ensures that the
  // TrajectoryController is not changing its state while being copied.
  // \todo: This can take quite some time, so that the process loop might
  //        violate the timing limits. The major time is used in cloning the
  //        graph. To circumvent this, we could implement a constructor that
  //        is provided with a separate graph (e.g. instantiated during
  //        construction) that will only be linked. However, this requires
  //        touching the ControllerBase and TrajectoryController classes.
  std::shared_ptr<TrajectoryPredictor> pred = std::make_shared<TrajectoryPredictor>(tc);

  // Clears the predictor's trajectories and copies tSet
  pred->setTrajectory(tSet);

  // Commenting out the below line runs the checking within the event loop.
  // This breaks the real-time constraints and should only be done for testing
  // in a simulator.
  // checkerThread(tSet, false, pred);

  // Here we launch a thread that performs the checking concurrently to the ES
  // event loop. It takes care about firing the events depending on the result
  // of the prediction. Therefore we can detach it and are done here. In the
  // thread function, we must take care of its deletion.
  std::thread t1(&TrajectoryComponent::checkerThread, this, tSet, false, pred);
  t1.detach();

  t_calc = Timer_getSystemTime() - t_calc;
  RLOG(1, "onCheckAndSetTrajectory took %.3f msec", t_calc*1.0e3);
}

/*******************************************************************************
 * The lock_guard protects the function from reentrant calls. We just allow it
 * to be called once the previous call has been completed.
 ******************************************************************************/
void TrajectoryComponent::checkerThread(TCS_sptr tSet, bool simulateOnly,
                                        std::shared_ptr<TrajectoryPredictor> predictor)
{
  // Reentrancy lock
  std::lock_guard<std::mutex> lock(checkerThreadMtx);

  // Perform the actual prediction
  auto result = predictor->predict(getEntity()->getDt());
  bool trajOk = result.success;
  REXEC(1)
  {
    result.print();
  }

  // Copy over predicted q-array and reset the animation counter. When rendering
  // the prediction model it should be in sync with the "real" motion. We need
  // the mutex since this function runs concurrently with the onRender event.
  // Why do we do such a weird pointer copy? Because the code inside the mutex
  // can slow down the event loop in case of large memory sizes.
  MatNd* newPredictions = MatNd_create(1, 1);
  predictor->getPredictionArray(newPredictions);
  renderMtx.lock();
  MatNd* buf = this->tPred;
  this->tPred = newPredictions;

  // This rewinds the animation counter of the predicted model so that it will
  // start over from the beginning for each new trajectory. \todo: Needs to
  // be thought through for cases where several trajectories are published
  // concurrently, for instance applied to different end effectors or so.
  this->animationTic = 0;
  renderMtx.unlock();
  MatNd_destroy(buf);

  // Early exit without applying the trajectory if the predictor reports issues
  if (!trajOk)
  {
    getEntity()->publish("ActionResult", false, 0.0, result.message);
    return;
  }
  else if (simulateOnly)
  {
    double quality = 1.0/(1.0+result.jlCost);   // \todo: Discuss and match expectations
    std::string resMsg = "SUCCESS DEVELOPER: Simulated trajectory is valid";
    getEntity()->publish("ActionResult", true, quality, resMsg);
  }

  // We can't directly set it here, since this runs concurrently with the event
  // loops's stepTrajectory call. We need to defer the trajectory to the event
  // loop and do this with the SetTrajectory event.
  if (!simulateOnly)
  {
    getEntity()->publish("SetTrajectory", tSet);
  }

  RLOG(0, "Simulated trajectory is valid");
}

void TrajectoryComponent::enableDebugRendering(bool enable)
{
  if ((enable==false) && (this->enableDbgRendering != enable))
  {
    getEntity()->publish<std::string,std::string>("RenderCommand", "Prediction",
                                                  "erase");
  }

  this->enableDbgRendering = enable;
}

void TrajectoryComponent::onStop()
{
  // Wait until the checkerThread has finished
  std::lock_guard<std::mutex> lock(checkerThreadMtx);
  enableDebugRendering(false);
}

void TrajectoryComponent::onRender()
{
  renderMtx.lock();

  if ((this->tPred==NULL) || (this->enableDbgRendering==false))
  {
    animationTic = 0;
    renderMtx.unlock();
    return;
  }

  double t_calc = Timer_getSystemTime();

  // Copy all transforms from the prediction to the graph
  double* src = MatNd_getRowPtr(tPred, animationTic);
  RCSGRAPH_FOREACH_BODY(animationGraph)
  {
    memcpy(&BODY->A_BI, src, sizeof(HTr));
    src += N_DOUBLES_IN_HTR;
  }


  animationTic += 10;

  if (animationTic >= tPred->m)
  {
    animationTic = 0;
  }
  renderMtx.unlock();

  getEntity()->publish<std::string,std::string>("RenderCommand", "Prediction",
                                                "setGhostMode");
  getEntity()->publish<std::string,const RcsGraph*>("RenderGraph", "Prediction",
                                                    animationGraph);

  t_calc = Timer_getSystemTime() - t_calc;
  if (t_calc>0.005)
  {
    RLOG(0, "*************************onRender in trajectory took %.3f msec", t_calc*1.0e3);
  }
}

double TrajectoryComponent::getMotionEndTime() const
{
  return this->motionEndTime;
}

const TrajectoryControllerBase* TrajectoryComponent::getTrajectoryController() const
{
  return this->tc;
}

/*******************************************************************************
 * The array channels contains the effectors that the tasks are utilizing. We
 * maintain a map (effectorTaskMap) with keys being the effctors, and values
 * being all tasks that are using the correpsonding key.
 *
 * On an incoming new task vector, all tasks that currently live in the
 * argument channels, are deleted.
 ******************************************************************************/
void TrajectoryComponent::onTaskVectorChangeSequential(std::vector<std::string> taskVec,
                                                       std::vector<std::string> channels)
{
  std::vector<Rcs::Task*> tasks = Rcs::TaskFactory::createTasks(taskVec, tc->getInternalController()->getGraph());

  if (tasks.empty())
  {
    return;
  }

  // From this point on, the vector tasks contains valid tasks for the given
  // taskVec.

  // Here we should delete only the tasks that are in out channels
  tc->getInternalController()->eraseTasks();

  for (auto t : tasks)
  {
    tc->getInternalController()->add(t);
  }


  // From here on, all tasks are valid.
  tc->eraseTrajectories();
  tc->populateTrajectories(1.0);// \todo: horizon?

  MatNd_realloc(a_des, tc->getInternalController()->getNumberOfTasks(), 1);
  MatNd_realloc(x_des, tc->getInternalController()->getTaskDim(), 1);

  RLOG(1, "Done task replacement");
}

void TrajectoryComponent::onResetAnimationTic()
{
  animationTic = 0;
}

}   // namespace aff
