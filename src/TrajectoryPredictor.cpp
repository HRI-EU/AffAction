/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:

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

#include "TrajectoryPredictor.h"

#include <IkSolverConstraintRMR.h>
#include <Rcs_typedef.h>
#include <Rcs_utils.h>
#include <Rcs_timer.h>
#include <Rcs_macros.h>
#include <Rcs_body.h>
#include <Rcs_joint.h>
#include <Rcs_shape.h>
#include <Rcs_kinematics.h>

#include <algorithm>
#include <cfloat>


// Permissible limit of each task component to lag behind desired reference.
// Does not make a difference between different units of tasks:
// 1m = 1rad for instance
#define MAX_TRACKING_ERROR (0.05)
#define N_DOUBLES_IN_HTR   (sizeof(HTr)/sizeof(double))

int aff::TrajectoryPredictor::animationMode = 0;

/*******************************************************************************
 * Clone graph without meshes and sensors.
 ******************************************************************************/
static RcsGraph* RcsGraph_cloneEssential(const RcsGraph* src)
{
  if (src==NULL)
  {
    return NULL;
  }

  RcsGraph* dst = RALLOC(RcsGraph);

  if (!dst)
  {
    RLOG(1, "Failed to allocate memory for cloning graph");
    return NULL;
  }

  // Copy the full memory block
  memcpy(dst, src, sizeof(RcsGraph));

  // Adjust pointers to local data
  dst->q = MatNd_clone(src->q);
  dst->q_dot = MatNd_clone(src->q_dot);
  dst->bodies = RNALLOC(src->nBodies, RcsBody);
  dst->joints = RNALLOC(src->dof, RcsJoint);

  // Skip copying sensors
  dst->nSensors = 0;
  dst->sensors = NULL;

  if ((!dst->q) || (!dst->q_dot) || (!dst->bodies) || (!dst->joints))
  {
    RLOG(1, "Failed to allocate memory for cloning graph");
    RcsGraph_destroy(dst);
    return NULL;
  }

  // Copy all joints, it's just a big memory block
  memcpy(dst->joints, src->joints, src->dof*sizeof(RcsJoint));

  // Same for bodies. We handle the copying of the shapes below.
  memcpy(dst->bodies, src->bodies, src->nBodies * sizeof(RcsBody));

  // Copy all shapes for each body
  for (unsigned int i=0; i<src->nBodies; ++i)
  {
    RcsBody* bdyDst = &dst->bodies[i];
    const RcsBody* bdySrc = &src->bodies[i];

    // Make a copy of all body shapes. We  do a block copy and reset the mesh
    // to NULL to avoid double deletion
    bdyDst->shapes = RNALLOC(bdySrc->nShapes, RcsShape);
    memcpy(bdyDst->shapes, bdySrc->shapes, bdySrc->nShapes*sizeof(RcsShape));
    for (unsigned int i = 0; i < bdyDst->nShapes; i++)
    {
      bdyDst->shapes[i].mesh = NULL;
    }

  }   // RCSGRAPH_TRAVERSE_BODIES(src)

  return dst;
}

using namespace tropic;

namespace aff
{

/*******************************************************************************
 *
 ******************************************************************************/
TrajectoryPredictor::PredictionResult::PredictionResult() :
  idx(-1), success(false), minDist(0.0), jlCost(0.0), collCost(0.0), actionCost(0.0),
  scaleJointSpeeds(1.0), elbowNS(0.0), wristNS(0.0), t_predict(0.0), graph(nullptr)
{
}

TrajectoryPredictor::PredictionResult::~PredictionResult()
{
}

double TrajectoryPredictor::PredictionResult::cost() const
{
  // Both terms are normalized between 0 and 1, therefore only hald the sum.
  return (jlCost + collCost + 8.0 * actionCost) / 10.0;
}

// The lesser function for sorting a vector of results. The failure -
// success comparisons ensure that the first-ranked solutions are valid.
bool TrajectoryPredictor::PredictionResult::lesser(const PredictionResult& a,
                                                   const PredictionResult& b)
{
  if (a.success && !b.success)
  {
    return true;
  }

  if (!a.success && b.success)
  {
    return false;
  }

  return a.cost() < b.cost();
}

void TrajectoryPredictor::PredictionResult::print(int verbosityLevel) const
{
  if (verbosityLevel < 0 || RcsLogLevel < 0)
  {
    return;
  }

  std::cout << "[" << __FILENAME__ << ": " << "(" << __LINE__ << ")]: ";
  std::cout << "PredictionResult " << std::to_string(idx) << ": ";

  if (success)
  {
    std::cout << "SUCCESS ";
  }
  else
  {
    std::cout << "FAILURE ";
  }

  if (verbosityLevel >= 1)
  {
    std::cout << "cost: " << cost();
  }

  if (verbosityLevel >= 2)
  {
    std::cout << std::endl;
    std::cout << "t_predict: " << t_predict << std::endl;
    std::cout << "minDist: " << minDist << std::endl;
    std::cout << "jlCost: " << jlCost << std::endl;
    std::cout << "collCost: " << collCost << std::endl;
    std::cout << "actionCost: " << actionCost << std::endl;
    std::cout << "scaleJointSpeeds: " << scaleJointSpeeds << std::endl;
    std::cout << "error message: " << feedbackMsg.error << std::endl;
    std::cout << "reason message: " << feedbackMsg.reason << std::endl;
    std::cout << "suggestion message: " << feedbackMsg.suggestion << std::endl;
    std::cout << "developer message: " << feedbackMsg.developer << std::endl;
    std::cout << "resolvedActionCommand: " << resolvedActionCommand << std::endl;
    std::cout << "minDistBdy1: " << minDistBdy1 << std::endl;
    std::cout << "minDistBdy2: " << minDistBdy2 << std::endl;
    std::cout << "animation frames: " << bodyTransforms.size() << std::endl;
    std::cout << "jMask: " << std::endl;
    for (size_t i = 0; i < jMask.size(); ++i)
    {
      std::cout << jMask[i];
    }
  }

  std::cout << std::endl;
}





/*******************************************************************************
 *
 ******************************************************************************/
TrajectoryPredictor::TrajectoryPredictor(const TrajectoryControllerBase* tc_) :
  tc(NULL), ikSolver(NULL), tStack(NULL)
{
  RCHECK(12 == N_DOUBLES_IN_HTR);   // Should be 12, just to be sure
  this->tc = new TrajectoryControllerBase(*tc_);
  this->ikSolver = new Rcs::IkSolverRMR(tc->getInternalController());
  this->tStack = MatNd_create(1, tc->getController()->getGraph()->nBodies*N_DOUBLES_IN_HTR);
}

TrajectoryPredictor::~TrajectoryPredictor()
{
  delete this->tc;
  delete this->ikSolver;
  MatNd_destroy(this->tStack);
}

// Predicts a trajectory constraint set until its end time. The function
// propagates the trajectory over time, computes the inverse kinematics,
// and performs various checks for each time step. The sampling time interval
// is passed through the value dt.
// The function returns a struct with various information.
TrajectoryPredictor::PredictionResult TrajectoryPredictor::predict(double dt, bool earlyExit)
{
  PredictionResult result;
  result.t_predict = Timer_getTime();

  const bool jointLimitCheck = true;
  const bool collisionCheck = true;
  const bool speedLimitCheck = true;
  const bool withSpeedAccLimit = true;

  Rcs::ControllerBase* controller = tc->getInternalController();
  RcsGraph* graph = controller->getGraph();
  double t = 0.0, qFilt = 0.0, err = 0.0;   // Tracking error
  double endTime = tc->getRootSet().getEndTime();
  double motionDuration = tc->getRootSet().getDuration();

  // Initialize the "used joint mask" with zero.
  result.jMask.resize(tc->getController()->getGraph()->nJ);
  MatNd jMaskArr = MatNd_fromPtr(result.jMask.size(), 1, result.jMask.data());
  MatNd_setZero(&jMaskArr);

  // We allocate it once before looping over the arrray, since reallocations
  // within the loop slow down the function quite a lot on some OS.
  const size_t nAfterSteps = 500;
  const size_t nSteps = lround(endTime / dt) + nAfterSteps;
  MatNd_realloc(this->tStack, nSteps+1, tStack->n);
  MatNd_reshape(this->tStack, 0, tStack->n);

  // We need to check the current state, otherwise we don't have any
  // information about it.

  // Copy all transforms of the time step 0. This is the first row of tStack.
  tStack->m++;
  double* dst = MatNd_getRowPtr(tStack, tStack->m-1);
  RCSGRAPH_FOREACH_BODY(graph)
  {
    Vec3d_copy(dst, BODY->A_BI.org);
    Mat3d_toArray(dst+3, BODY->A_BI.rot);
    dst += N_DOUBLES_IN_HTR;
  }

  // Update collision model and checking
  controller->computeCollisionModel();

  // We initialize the feedback message with the closest proximity at
  // intitialization time. It may be that there are no collisions, and
  // therefore minDist=DBL_MAX and the minDistBdy1 and 2 are "NULL"
  int minDistPair = -1;
  const RcsCollisionMdl* cmdl = controller->getCollisionMdl();
  result.minDist = RcsCollisionMdl_getMinDistPair(cmdl, &minDistPair);
  result.minDistBdy1 = (minDistPair==-1) ? "NULL" : RCSBODY_NAME_BY_ID(cmdl->graph, cmdl->pair[minDistPair].b1);
  result.minDistBdy2 = (minDistPair==-1) ? "NULL" : RCSBODY_NAME_BY_ID(cmdl->graph, cmdl->pair[minDistPair].b2);

  // Check of initial conditions of the prediction. Since we did not compute
  // the IK yet, there is no singularity check at this point

  //if (check(jointLimitCheck, collisionCheck, speedLimitCheck)==false)
  //{
  //  result.feedbackMsg.error = "FATAL_ERROR";
  //  result.feedbackMsg.reason = "Initial state is invalid";
  //  result.success = false;
  //  result.t_predict = Timer_getTime() - result.t_predict;
  //  return result;
  //}

  // Initialize the previous activation vector with the current activation
  // state to determine task switches.
  MatNd* a_des = MatNd_create(tc->getController()->getNumberOfTasks(), 1);
  tc->getActivation(a_des);
  MatNd* a_prev = MatNd_clone(a_des);
  MatNd* x_des = MatNd_create(tc->getController()->getTaskDim(), 1);

  const double alpha = 0.05;      // \todo (MG): Check if consistent with IK
  const double lambda = 1.0e-4;   // \todo (MG): Check if consistent with IK
  const double qFiltDecay = 0.1;  // \todo (MG): Check if consistent with IK
  const bool verbose = false;
  //double scaleJointSpeeds = 1.0;

  result.success = true;

  size_t count = 0;
  double jointSpeedScaler = 0.0;

  // Simulate the whole trajectory. We simulate a bit longer than the actual
  // trajectory duration, since there can be issues later with the null space
  // motion.
  //while (endTime > TRAJECTORY1D_ALMOST_ZERO)
  for (size_t iter=0; iter< nSteps; ++iter)
  {
    bool successPrev = result.success;

    bool taskSwitch = !MatNd_isEqual(a_prev, a_des, 1.0e-3);

    if (taskSwitch)
    {
      qFilt = 1.0;
    }

    if (qFiltDecay<=0.0)
    {
      qFilt = 0.0;
    }
    else
    {
      qFilt = Math_clip(qFilt-(1.0/qFiltDecay)*dt, 0, 1.0);
    }

    endTime = tc->step(dt);
    count++;

    tc->getPosition(x_des);
    MatNd_copy(a_prev, a_des);
    tc->getActivation(a_des);
    double blending = tc->computeBlending();

    // We disallow null space movements if there is no active task. This avoids null space
    // collisions during joint limit avoidance convergence.
    if (MatNd_maxAbsEle(a_des)<1.0e-8)
    {
      blending = 0.0;
    }

    // Updates graph with new state
    ActionResult resMsg;

    // The phase computation must match the one in the TrajectoryComponent so that predictor and
    // run-time lead to the same results. \todo(MG): This should not be duplicate code.
    const double phase = (motionDuration > 0.0) ? 1.0 - (endTime / motionDuration) : 0.0;
    const double phaseScale = sin(M_PI * phase);

    int ikRes = computeIK(ikSolver, a_des, x_des,
                          dt, blending*alpha, lambda, qFilt, phaseScale,
                          speedLimitCheck, jointLimitCheck,
                          collisionCheck, withSpeedAccLimit,
                          verbose, &jMaskArr, resMsg);

    //scaleJointSpeeds = std::min(RcsGraph_checkJointSpeeds(graph, graph->q_dot, 1.0, RcsStateFull), scaleJointSpeeds);

    // Determine joint speed scaling factor to exactly be at the speed limit
    double newJointScaling = 0.0;   // 0 for no speeds, 1 for max speeds
    RCSGRAPH_FOREACH_JOINT(graph)
    {
      const double q_dot = fabs(graph->q_dot->ele[JNT->jointIndex]);
      const double newScaling = q_dot / JNT->speedLimit;
      newJointScaling = std::max(newJointScaling, newScaling);

      if (newScaling > jointSpeedScaler)
      {
        REXEC(6)
        {
          double sLim = JNT->speedLimit;
          double toDeg = RcsJoint_isRotation(JNT) ? 180.0/M_PI : 1.0;
          RMSG("Scaling speeds with %5.5f (due to joint \"%s\": speed=%5.6f %s  "
               "limit=%5.6f)", newScaling, JNT->name, toDeg*fabs(q_dot),
               RcsJoint_isRotation(JNT) ? "deg" : "m", toDeg*sLim);
        }
        jointSpeedScaler = newScaling;
      }
    }



    // computeIK does already several checks:
    //   - Determinant of Jacobian being zero
    //   - Speed limit check
    //   - Joint limit check
    //   - Collision check
    if (ikRes!=0)
    {
      result.feedbackMsg = resMsg;
      result.success = false;
    }

    result.jlCost += controller->computeJointlimitCost();
    result.collCost += controller->getCollisionCost();

    // We compute a tracking error that models a task space error between
    // desired and commanded task space vectors. This error emergrs only
    // if the speed and / or acceleration limits are enforced. Otherwise,
    // it should never occur since the joint space displacements don't
    // undergo any clipping.
    if (result.success)
    {
#if 0
      // Difference between desired trajectory and IK state
      // New implementation considering different task metrics
      {
        unsigned int nRowsActive = 0, nRowsAll = 0;
        MatNd* dx_err = MatNd_createLike(x_des);   // It's a bit oo large, but this doesn't matter
        bool trackingErrorFound = false;

        for (size_t i = 0; i < controller->getNumberOfTasks(); ++i)
        {
          Rcs::Task* task = controller->getTask(i);
          unsigned int dimTask = task->getDim();

          if (MatNd_get2(a_des, i, 0) > 0.0)
          {
            const double* x_des_i = &x_des->ele[nRowsAll];
            double* dx_i = &dx_err->ele[nRowsActive];
            task->computeDX(dx_i, x_des_i);
            double errEps = MAX_TRACKING_ERROR;
            double err_i = VecNd_maxAbsEle(dx_i, dimTask);

            if (task->getClassName() == "Joint" ||
                task->getClassName() == "Joints" ||
                task->getClassName() == "POLAR" ||
                task->getClassName() == "ABC")
            {
              errEps = RCS_DEG2RAD(5.0);
            }

            if (err_i > errEps)
            {
              trackingErrorFound = true;
              RLOG(0, "Found tracking error in task %s: %f > %f", task->getName().c_str(), err_i, errEps);
            }

            err = std::max(err, err_i);

            nRowsActive += dimTask;
          }

          nRowsAll += dimTask;
        }


        if (trackingErrorFound)
        {
          result.feedbackMsg.reason = "Tracking error at t=" + std::to_string(t) + " is " +
                                      std::to_string(err);
          result.success = false;
        }

        MatNd_destroy(dx_err);
      }



#else
      // Old tracking error
      {
        // Difference between desired trajectory and IK state
        MatNd* dx_err = MatNd_createLike(x_des);
        controller->computeDX(dx_err, x_des, a_des);

        err = std::max(err, MatNd_maxAbsEle(dx_err));

        if (err > MAX_TRACKING_ERROR*(1.0+newJointScaling))
        {
          controller->decompressFromActiveSelf(dx_err, a_des);
          unsigned int errIdx = MatNd_maxAbsEleIndex(dx_err);
          RLOG(0, "errIdx = %d", errIdx);

          result.feedbackMsg.error = "Reachability problem";
          // result.feedbackMsg.suggestion = "Try another object that is closer, or try to get it closer with a tool";
          result.feedbackMsg.suggestion = "Try again, but move more slowly";
          result.feedbackMsg.developer = "Tracking error at t=" + std::to_string(t) + " is " + std::to_string(err);
          result.feedbackMsg.developer += " " + std::string(__FILENAME__) + " " + std::to_string(__LINE__);
          std::string unreachableObject = "object", reachingEffector = "hand";

          // Here we add details about the "guilty" task and its dimension.
          size_t count = 0;
          for (size_t i = 0; i < controller->getNumberOfTasks(); i++)
          {
            for (size_t j = 0; j < controller->getTaskDim(i); j++)
            {
              if (count==errIdx)
              {
                result.feedbackMsg.developer += ": Task " + controller->getTaskName(i) + "[dim " + std::to_string(j) + "] ";
                const RcsBody* effBdy = RCSBODY_BY_ID(controller->getGraph(), controller->getTask(i)->getEffectorId());
                const RcsBody* refBdy = RCSBODY_BY_ID(controller->getGraph(), controller->getTask(i)->getRefBodyId());

                if (effBdy && refBdy)
                {
                  if (!RcsBody_isArticulated(controller->getGraph(), effBdy))
                  {
                    unreachableObject = effBdy->name;
                  }
                  else if (!RcsBody_isArticulated(controller->getGraph(), refBdy))
                  {
                    unreachableObject = refBdy->name;
                  }

                  if (unreachableObject==effBdy->name)
                  {
                    reachingEffector = refBdy->name;
                  }
                  else if (unreachableObject==refBdy->name)
                  {
                    reachingEffector = effBdy->name;
                  }

                  result.feedbackMsg.developer += "effector: " + std::string(effBdy->name);
                  result.feedbackMsg.developer += " refBdy: " + std::string(refBdy->name);
                  result.feedbackMsg.developer += " reachingEffector: " + reachingEffector;
                  result.feedbackMsg.developer += " unreachable object: " + unreachableObject + "\n";
                }
              }
              count++;
            }
          }

          if (unreachableObject.empty())
          {
            result.feedbackMsg.reason = "Can't move the " + reachingEffector + " fast enough - the action duration is too short.";
          }
          else
          {
            result.feedbackMsg.reason = "Can't reach the " + unreachableObject + " with the " + reachingEffector + " - it might be too far away, or you need to move more slowly";
          }

          for (unsigned int i=0; i<controller->getGraph()->dof; ++i)
          {
            const RcsJoint* jnt = &controller->getGraph()->joints[i];
            if (jnt->constrained || jnt->speedLimit==DBL_MAX)
            {
              continue;
            }

            result.feedbackMsg.developer += "Joint " + std::string(jnt->name) + ": q_dot = " +
                                            std::to_string(controller->getGraph()->q_dot->ele[i]) +
                                            " q_dot_max = " + std::to_string(jnt->speedLimit) + "\n";
          }

          result.success = false;

          REXEC(4)
          {
            RLOG(0, "At index %d:", errIdx);
            controller->printX(dx_err, a_des);
          }
        }

        MatNd_destroy(dx_err);
        // Difference
      }
#endif
    }

    // Here we determine the closest pair of all proximities, and store its
    // value. The overal closest pair along with its distance is returned
    // with the result struct. Since the collision model may change between
    // different steps due to the broadphase updates, we need to keep track
    // of the closest distances in each iteration.
    double dist_i = RcsCollisionMdl_getMinDistPair(cmdl, &minDistPair);

    if (dist_i < result.minDist)
    {
      result.minDist = dist_i;
      result.minDistBdy1 = RCSBODY_NAME_BY_ID(cmdl->graph, cmdl->pair[minDistPair].b1);
      result.minDistBdy2 = RCSBODY_NAME_BY_ID(cmdl->graph, cmdl->pair[minDistPair].b2);
    }

    // Copy all transforms of the current time step
    tStack->m++;
    double* dst = MatNd_getRowPtr(tStack, tStack->m-1);
    RCSGRAPH_FOREACH_BODY(graph)
    {
      Vec3d_copy(dst, BODY->A_BI.org);
      Mat3d_toArray(dst+3, BODY->A_BI.rot);
      dst += N_DOUBLES_IN_HTR;
    }

    // Early exit in case of detected failure.
    // \todo: Re-think early exit with respect to only error cases. They might not be
    // comparable if the time horizon differs.
    if (!result.success && successPrev)
    {
      RLOG(4, "Predictor failed at t=%f (of %f, after-step period: %f) - quitting",
           t, nSteps*dt, nAfterSteps*dt);

      if (earlyExit)
      {
        break;
      }
    }

    if (Timer_getTime()-result.t_predict > 5.0)
    {
      RLOG(1, "Predictor takes pretty long: at t=%f", t);
    }

    t += dt;
  }   // while (endTime > TRAJECTORY1D_ALMOST_ZERO)

  result.jlCost /= tStack->m;   // Normalize by number of steps
  result.collCost /= tStack->m;   // Normalize by number of steps

  // Normalize the costs to a range between 0 and 1. The result will be
  // 0 for a cost=0, and 1 for a cost=inf
  result.jlCost = (result.jlCost) / (1.0+result.jlCost);
  result.collCost = (result.collCost) / (1.0+result.collCost);

  // Multiplay duration with this value to get optimal speed
  result.scaleJointSpeeds = jointSpeedScaler;

  MatNd_destroyN(3, a_des, x_des, a_prev);
  MatNd_binarizeSelf(&jMaskArr, 0.0);

  if (result.feedbackMsg.error.empty())
  {
    result.feedbackMsg.error = "SUCCESS";
  }

  // \todo: This is a lot of memory for large trees
  if ((animationMode==1 && result.success) || (animationMode>1))
  {
    result.bodyTransforms.resize(tStack->size);
    memcpy(result.bodyTransforms.data(), tStack->ele, tStack->size * sizeof(double));
  }


  // We add a clone so that we can call this classes methods several times.
  result.graph = RcsGraph_cloneEssential(graph);
  result.t_predict = Timer_getTime() - result.t_predict;

  RLOG(4, "Trajectory %s after %d steps, took %.3f sec",
       result.success ? "ok" : "failed", tStack->m, result.t_predict);
  RLOG(4, "   at t=%f (of %f, after-step period: %f)",
       tStack->m*dt, nSteps* dt, nAfterSteps* dt);
  RLOG_CPP(4, "Result: " << result.feedbackMsg.toString());

  return result;
}

void TrajectoryPredictor::clearTrajectory()
{
  tc->clear();
}

void TrajectoryPredictor::initFromState(const MatNd* q, const MatNd* q_dot)
{
  RcsGraph_setState(tc->getInternalController()->getGraph(), q, q_dot);
  tc->clear();
  tc->init();
}

void TrajectoryPredictor::setTrajectory(TCS_sptr tSet)
{
  tc->clear();
  tc->addAndApply(std::shared_ptr<ConstraintSet>(tSet->clone()));
}

bool TrajectoryPredictor::check(bool jointLimitCheck, bool collisionCheck,
                                bool speedLimitCheck) const
{
  bool success = tc->getInternalController()->checkLimits(jointLimitCheck,
                                                          collisionCheck,
                                                          speedLimitCheck,
                                                          0.0, 0.0, 0.01);

  if (success==false)
  {
    RLOG(1, "limit violation");
    return false;
  }

  return true;
}

void TrajectoryPredictor::addElbowNullspace(const RcsGraph* graph, MatNd* dH)
{
  double gain = 5.0;
  double boundary = 0.15;
  double penetrationClip = -0.5;
  double dist = 0.0;
  HTr A_SB;   // From base to shoulder
  HTr A_EB;   // From base to elbow
  MatNd* J = MatNd_create(1, graph->nJ);

  // These we need: s: shoulder, b: base (HTr_invTransform(A_sb, A_bI, A_sI))
  const RcsBody* base = RcsGraph_getBodyByName(graph, "base_footprint");
  const RcsBody* left_elbow = RcsGraph_getBodyByName(graph, "forearm_left");
  const RcsBody* right_elbow = RcsGraph_getBodyByName(graph, "forearm_right");
  const RcsBody* sh_left = RcsGraph_getBodyByName(graph, "upperarm_left");
  const RcsBody* sh_right = RcsGraph_getBodyByName(graph, "upperarm_right");

  // Left elbow
  if (base && left_elbow && sh_left)
  {
    HTr_invTransform(&A_SB, &base->A_BI, &sh_left->A_BI);
    HTr_invTransform(&A_EB, &base->A_BI, &left_elbow->A_BI);
    dist = Math_clip((A_EB.org[1] - A_SB.org[1]) - boundary, penetrationClip, 0.0);
    //RLOG(1, "Left elbow: dist=%f   el=%f   sh=%f", dist, A_EB.org[1], A_SB.org[1]);

    RcsGraph_1dPosJacobian(graph, left_elbow, base, NULL, 1, J);
    MatNd_constMulSelf(J, dist*gain);
    MatNd_addSelf(dH, J);
  }

  // Right elbow
  if (base && right_elbow && sh_right)
  {
    HTr_invTransform(&A_SB, &base->A_BI, &sh_right->A_BI);
    HTr_invTransform(&A_EB, &base->A_BI, &right_elbow->A_BI);
    dist = Math_clip((- A_EB.org[1] + A_SB.org[1]) - boundary, penetrationClip, 0.0);
    // RLOG(1, "Right elbow: dist=%f   el=%f   sh=%f", dist, A_EB.org[1], A_SB.org[1]);

    RcsGraph_1dPosJacobian(graph, right_elbow, base, NULL, 1, J);
    MatNd_constMulSelf(J,  -dist * gain);
    MatNd_addSelf(dH, J);
  }

#if 1
  left_elbow = RcsGraph_getBodyByName(graph, "hand_left");
  right_elbow = RcsGraph_getBodyByName(graph, "hand_right");
  boundary = 0.0;

  // Left wrist
  if (base && left_elbow && sh_left)
  {
    HTr_invTransform(&A_SB, &base->A_BI, &sh_left->A_BI);
    HTr_invTransform(&A_EB, &base->A_BI, &left_elbow->A_BI);
    dist = Math_clip((A_EB.org[1] - A_SB.org[1]) - boundary, penetrationClip, 0.0);
    //RLOG(1, "Left elbow: dist=%f   el=%f   sh=%f", dist, A_EB.org[1], A_SB.org[1]);

    RcsGraph_1dPosJacobian(graph, left_elbow, base, NULL, 1, J);
    MatNd_constMulSelf(J, dist*gain);
    MatNd_addSelf(dH, J);
  }

  // Right wrist
  if (base && right_elbow && sh_right)
  {
    HTr_invTransform(&A_SB, &base->A_BI, &sh_right->A_BI);
    HTr_invTransform(&A_EB, &base->A_BI, &right_elbow->A_BI);
    dist = Math_clip((- A_EB.org[1] + A_SB.org[1]) - boundary, penetrationClip, 0.0);
    // RLOG(1, "Right elbow: dist=%f   el=%f   sh=%f", dist, A_EB.org[1], A_SB.org[1]);

    RcsGraph_1dPosJacobian(graph, right_elbow, base, NULL, 1, J);
    MatNd_constMulSelf(J,  -dist * gain);
    MatNd_addSelf(dH, J);
  }
#endif

  // Clean up
  MatNd_destroy(J);
}

void TrajectoryPredictor::addWristNullspace(const RcsGraph* graph, MatNd* dH)
{
  // imagine a fronta plane in front of the robot with distance "boundary"
  // to the base_footprint frame. We push the wrist in front of this plane
  // if it is behind.
  const double boundary = 0.25;
  const double penetrationClip = -0.5;
  const double gain = 5.0;
  double dist = 0.0, dBound = 0.0;
  MatNd* J = MatNd_create(1, graph->nJ);
  HTr A_WB;   // From base to wrist

  // These we need
  const RcsBody* left_wrist = RcsGraph_getBodyByName(graph, "hand_left");
  const RcsBody* right_wrist = RcsGraph_getBodyByName(graph, "hand_right");
  const RcsBody* base = RcsGraph_getBodyByName(graph, "base_footprint");


  // Left arm, A_WB.org is the vector to the wrist, represented in the base frame.
  if (left_wrist && base)
  {
    HTr_invTransform(&A_WB, &base->A_BI, &left_wrist->A_BI);
    dBound = A_WB.org[2]>1.0 ? 0.4*(A_WB.org[2]-1.0) : 0.0;
    dist = Math_clip(A_WB.org[0] - (boundary+dBound), penetrationClip, 0.0);
    RcsGraph_1dPosJacobian(graph, left_wrist, base, NULL, 0, J);
    MatNd_constMulSelf(J, dist * gain);
    MatNd_addSelf(dH, J);
    //RLOG(1, "Left wrist:  dist=%f   dBound=%f   x_curr=%f", dist, dBound, A_WB.org[0]);
  }

  // Right arm
  if (right_wrist && base)
  {
    HTr_invTransform(&A_WB, &base->A_BI, &right_wrist->A_BI);
    dBound = A_WB.org[2]>1.0 ? 0.4*(A_WB.org[2]-1.0) : 0.0;
    dist = Math_clip(A_WB.org[0] - (boundary+dBound), penetrationClip, 0.0);
    RcsGraph_1dPosJacobian(graph, right_wrist, base, NULL, 0, J);
    MatNd_constMulSelf(J, dist * gain);
    MatNd_addSelf(dH, J);
    //RLOG(1, "Right wrist: dist=%f   dBound=%f   x_curr=%f", dist, dBound, A_WB.org[0]);
  }

  MatNd_destroy(J);
}


int TrajectoryPredictor::computeIK(Rcs::IkSolverRMR* solver, const MatNd* a, const MatNd* x,
                                   double dt, double alpha, double lambda, double qFilt, double phase,
                                   bool speedLimitCheck, bool jointLimitCheck,
                                   bool collisionCheck, bool withSpeedAccLimit,
                                   bool verbose, MatNd* jMask, ActionResult& resMsg)
{
  Rcs::ControllerBase* controller = solver->getController();
  RcsGraph* graph = controller->getGraph();

  MatNd* dq_des = MatNd_create(graph->dof, 1);
  MatNd* dx_des = MatNd_create(controller->getTaskDim(), 1);
  MatNd* dH = MatNd_create(1, graph->nJ);
  MatNd* qdot = MatNd_createLike(dq_des);

  // Inverse kinematics
  controller->computeDX(dx_des, x);
  controller->computeJointlimitGradient(dH);
  MatNd_constMulSelf(dH, alpha);

  // Extra nullspace to handle elbow and wrists getting too close to the body.
  // We do this after the joint masking since this should take effect also with
  // constraints on and objects in hand.
  // REXEC(0)
  {
    MatNd* dH_extra = MatNd_createLike(dH);
    MatNd* dH_tmp = MatNd_createLike(dH);
    addElbowNullspace(graph, dH_tmp);
    const double elbowNS = MatNd_getNorm(dH_tmp);
    MatNd_addSelf(dH_extra, dH_tmp);

    MatNd_setZero(dH_tmp);
    addWristNullspace(graph, dH_tmp);
    const double wristNS = MatNd_getNorm(dH_tmp);

    // RLOG(1, "elbowNS=%f   wristNS=%f", elbowNS, wristNS);

    MatNd_addSelf(dH_extra, dH_tmp);
    MatNd_constMulSelf(dH_extra, alpha);

    MatNd_transposeSelf(dH_extra);
    double scale = RcsGraph_checkJointSpeeds(graph, dH_extra, dt, RcsStateIK);
    MatNd_transposeSelf(dH_extra);

    if (scale < 1.0)
    {
      NLOG(0, "Scaling down joint speeds by factor %f", scale);
      MatNd_constMulSelf(dH_extra, 0.99999 * scale);
    }

    MatNd_constMulSelf(dH_extra, phase);
    MatNd_addSelf(dH, dH_extra);
    MatNd_destroyN(2, dH_extra, dH_tmp);
  }

  // Start Jacobian-based joint weighting: We apply null space gradients only to
  // joints that are contributing to the activated tasks or that correspond to a
  // joint recursion from a manipulator that is empty. This is important for
  // cases in which we need to keep a pose of an inactive task set (e.g. one arm
  // is holding an object), while we activate a task set for the other arm.
  // Otherwise, it is hard to compose complex motions out of independent simpler
  // ones when performing them in parallel. However, this introduces a kind of
  // hysteresis.
  //REXEC(0)
  {
    MatNd* aMask = MatNd_create(graph->nJ, 1);
    MatNd* eMask = MatNd_createLike(aMask);
    MatNd* tmpMask = MatNd_createLike(aMask);
    controller->computeActiveJointMask(aMask, a);

    if (jMask)
    {
      MatNd_addSelf(jMask, aMask);
    }
    MatNd_setElementsTo(eMask, 1.0);

    RCSGRAPH_FOREACH_BODY(graph)
    {
      if ((!BODY->rigid_body_joints) || (BODY->id == -1))
      {
        continue;
      }

      RcsGraph_computeJointRecursionMask(graph, BODY, tmpMask);

      for (unsigned int i = 0; i < tmpMask->size; ++i)
      {
        if (tmpMask->ele[i] > 0.0)
        {
          eMask->ele[i] = 0.0;
        }
      }
    }

    MatNd_addSelf(aMask, eMask);
    MatNd_binarizeSelf(aMask, 0.0);
    MatNd_transposeSelf(aMask);
    MatNd_eleMulSelf(dH, aMask);
    MatNd_destroyN(3, eMask, aMask, tmpMask);
  }

  // The right inverse is the method of choice here, since we have less task
  // dimensions than joint space dimensions. We also don't use conflicting
  // tasks (mainly). In some cases, we might add constraints to non-movable
  // dofs (e.g. when keeping an object upright that is parenting a static
  // object, to cover bi-manual actions). That's why we typically add a
  // small lambda regularization.
  RCHECK(lambda==1.0e-4);
  double det = solver->solveRightInverse(dq_des, dx_des, dH, a, lambda);

  // We treat a singular configuration as an error. Typically, lambda is
  // set to a value larger than zero, therefore this will probably never
  // be a source of error.
  if (det == 0.0)
  {
    resMsg.error = "ERROR";
    resMsg.reason = "Got into a singular posture";
    MatNd_destroyN(4, dq_des, dx_des, dH, qdot);
    return -1;
  }

  // Apply speed and acceleration limits only if speed limit check is set
  if (withSpeedAccLimit)
  {
    double scale = RcsGraph_checkJointSpeeds(graph, dq_des, dt, RcsStateFull);
    if (scale < 1.0)
    {
      NLOG(0, "Scaling down joint speeds by factor %f", scale);
      MatNd_constMulSelf(dq_des, 0.99999 * scale);
    }

    // Apply acceleration limits
    const MatNd* qdot_prev = graph->q_dot;
    MatNd_constMul(qdot, dq_des, 1.0 / dt);

    double dynLim = -0.99 * qFilt + 1.0;

    int nClippedAcc = RcsGraph_clipJointAccelerations(graph, qdot, qdot_prev,
                                                      dt, dynLim, RcsStateFull);

    // \todo: Currently deactivated acceleration clip, since we have a LPF
    // in the lower level drivers anyways.
    nClippedAcc = 0;
    if (nClippedAcc > 0)
    {
      MatNd_constMul(dq_des, qdot, dt);
    }

  }
  else
  {
    MatNd_constMul(qdot, dq_des, 1.0 / dt);
  }

  // Integration and FK  including velocities
  MatNd* q_old = MatNd_clone(graph->q);
  MatNd* qd_old = MatNd_clone(graph->q_dot);
  MatNd_addSelf(graph->q, dq_des);
  RcsGraph_setState(graph, NULL, qdot);

  // We perform the check after the forward kinematics to consider the
  // pose after the IK step.
  controller->computeCollisionModel();

  // This method is static and doesn't modify the TrajectoryPredictor instance
  int res = checkState(controller, speedLimitCheck, jointLimitCheck,
                       collisionCheck, verbose, resMsg);

  if (res != 0)
  {
    // \todo (MG): Currently commented out to see progress even in case of failure.
    // RcsGraph_setState(graph, q_old, qd_old);
  }

  // Clean up
  MatNd_destroyN(6, dq_des, dx_des, dH, qdot, q_old, qd_old);

  return res;
}


int TrajectoryPredictor::checkState(const Rcs::ControllerBase* controller,
                                    bool speedLimitCheck, bool jointLimitCheck,
                                    bool collisionCheck, bool verbose,
                                    ActionResult& resMsg)
{
  const RcsGraph* graph = controller->getGraph();
  resMsg.clear();

  // Speed limit check
  if (speedLimitCheck)
  {
    double scaling = RcsGraph_checkJointSpeeds(graph, graph->q_dot, 1.0, RcsStateFull);

    if (scaling < 1.0)
    {
      resMsg.error = "Speed limit violated, I can't move that fast";
      resMsg.reason = "The duration of the action might have been too short.";
      resMsg.suggestion = "Call the action with a longer duration.";

      if (verbose)
      {
        RLOG(1, "Joint speed limit violation");
        RCSGRAPH_TRAVERSE_JOINTS(controller->getGraph())
        {
          double qdi = graph->q_dot->ele[JNT->jointIndex];

          if ((!JNT->constrained) && (fabs(qdi) >= JNT->speedLimit))
          {
            double sf = RcsJoint_isRotation(JNT) ? 180.0 / M_PI : 1.0;
            char detailedMsg[256];
            snprintf(detailedMsg, 256, " DEVELOPER: %s: q_dot=%f   limit=%f [%s]", JNT->name,
                     sf * qdi, sf * JNT->speedLimit, sf == 1.0 ? "m/sec" : "deg/sec");
            RLOG(1, "%s", detailedMsg);
            resMsg.developer += detailedMsg;
          }
        }
      }

      return -2;
    }
  }

  // Joint limit check
  if (jointLimitCheck)
  {
    unsigned int aor = RcsGraph_numJointLimitsViolated(graph, 0.0, 0.0, false);
    if (aor > 0)
    {
      if (verbose)
      {
        REXEC(2)
        {
          RLOG(1, "%d joint limit violations", aor);
          RcsGraph_printState(graph, graph->q);
        }
      }

      resMsg.error = "ERROR";
      resMsg.reason = "Joint limit problem. The robot is not able to do move well enough. ";
      resMsg.developer = std::to_string(aor) + " joint limit violations";

      RCSGRAPH_TRAVERSE_JOINTS(graph)
      {
        const double qi = MatNd_get2(graph->q, JNT->jointIndex, 0);
        if (!JNT->constrained && (qi<JNT->q_min || qi>JNT->q_max))
        {
          resMsg.developer += " Joint " + std::string(JNT->name) + ": " + std::to_string(qi) + " outside [" +
                              std::to_string(JNT->q_min) + " ... " + std::to_string(JNT->q_max) + "]";
        }
      }

      return -3;
    }
  }

  // Collision check
  if (collisionCheck && controller->getCollisionMdl())
  {
    const double distLimit = 0.001;
    const RcsCollisionMdl* cmdl = controller->getCollisionMdl();
    int min_i = -1;
    double minDist = RcsCollisionMdl_getMinDistPair(cmdl, &min_i);

    if ((min_i != -1) && (minDist < distLimit))
    {
      const RcsBody* b1 = RCSBODY_BY_ID(graph, cmdl->pair[min_i].b1);
      const RcsBody* b2 = RCSBODY_BY_ID(graph, cmdl->pair[min_i].b2);
      RCHECK(b1 && b2);
      const bool articulatedB1 = RcsBody_isArticulated(graph, b1);
      const bool articulatedB2 = RcsBody_isArticulated(graph, b2);

      resMsg.error = "Collision problem";
      std::string collider;

      if (articulatedB1)
      {
        if (!articulatedB2)
        {
          collider = b2->name;
          resMsg.reason = "The " + std::string(b1->name) +
                          " collides with the " + collider;
        }
        else
        {
          resMsg.reason = "The " + std::string(b1->name) +
                          " and " + std::string(b2->name) + " collide";
        }
      }
      else // b1 is static object
      {
        collider = b1->name;
        resMsg.reason = "The " + std::string(b2->name) +
                        " collides with the " + collider;
      }

      if (!collider.empty())
      {
        resMsg.suggestion = "Try to get the " + collider + " out of the way";
      }
      else
      {
        resMsg.suggestion = "Try to get " + std::string(b1->name) + " and " +
                            std::string(b2->name) + " away from each other";
      }

      std::string devMsg =  "Collision detected at pair-idx " + std::to_string(min_i);
      devMsg += " between ";
      devMsg += RCSBODY_NAME_BY_ID(graph, cmdl->pair[min_i].b1);
      devMsg += " and ";
      devMsg += RCSBODY_NAME_BY_ID(graph, cmdl->pair[min_i].b2);
      resMsg.developer = devMsg;
      resMsg.developer += " " + std::string(__FILENAME__) + " line " + std::to_string(__LINE__);

      if (verbose)
      {
        RLOG(1, "Found collision distance of %f (must be >%f)", minDist, distLimit);
        REXEC(2)
        {
          RcsCollisionModel_fprintCollisions(stdout,
                                             controller->getCollisionMdl(),
                                             distLimit);
        }
      }

      return -4;
    }
  }


  return 0;
}

void TrajectoryPredictor::setAnimationMode(int mode)
{
  RLOG_CPP(0, "Setting animation mode to " << mode);
  animationMode = mode;
}

int TrajectoryPredictor::toggleAnimationMode()
{
  animationMode++;
  if (animationMode > 2)
  {
    animationMode = 0;
  }

  return animationMode;
}

}   // namespace aff
