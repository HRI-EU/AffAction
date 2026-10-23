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

#include "IKTeleOp.h"
#include "TrajectoryPredictor.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_utilsCPP.h>


namespace aff
{


template <typename T>
T get_required(const nlohmann::json& obj, const std::string& key)
{
  if (!obj.is_object())
  {
    throw std::runtime_error("Expected JSON object while reading key '" + key + "'");
  }

  if (!obj.contains(key))
  {
    throw std::runtime_error("Missing required key '" + key + "'");
  }

  const nlohmann::json& value = obj.at(key);

  try
  {
    return value.get<T>();
  }
  catch (const nlohmann::json::exception& e)
  {
    throw std::runtime_error(
      "Invalid type for key '" + key + "': " + std::string(e.what())
    );
  }
}

template <typename T>
bool get_optional(const nlohmann::json& obj, const std::string& key, T& out)
{
  if (!obj.is_object())
  {
    throw std::runtime_error("Expected JSON object while reading optional key '" + key + "'");
  }

  if (!obj.contains(key))
  {
    return false;
  }

  const nlohmann::json& value = obj.at(key);

  if (value.is_null())
  {
    return false;
  }

  try
  {
    out = value.get<T>();
    return true;
  }
  catch (const nlohmann::json::exception& e)
  {
    throw std::runtime_error(
      "Invalid type for optional key '" + key + "': " + std::string(e.what())
    );
  }
}

static std::vector<std::pair<int, double>> interpolateModelStates(const RcsGraph* graph,
                                                                  std::string name_ms1,
                                                                  std::string name_ms2,
                                                                  double s)
{
  // std::map<std::string, std::vector<std::pair<int, double>>>
  static auto mdlStateMap = Rcs::RcsGraph_getModelStates(graph);
  std::vector<std::pair<int, double>> res;

  auto it = mdlStateMap.find(name_ms1);

  if (it == mdlStateMap.end())
  {
    RLOG_CPP(1, "Couldn't find model state " << name_ms1);
    return res;
  }

  const auto& ms1 = it->second;

  it = mdlStateMap.find(name_ms2);

  if (it == mdlStateMap.end())
  {
    RLOG_CPP(1, "Couldn't find model state " << name_ms2);
    return res;
  }

  const auto& ms2 = it->second;

  if (ms1.size() != ms2.size())
  {
    RLOG_CPP(1, "Model states differ in size - not interpolating: " << name_ms2 << " " << name_ms2);
    return res;
  }

  if (s < 0.0)
  {
    s = 0.0;
  }
  else if (s > 1.0)
  {
    s = 1.0;
  }

  for (size_t i=0; i<ms1.size(); ++i)
  {
    // Enforce indices are matching
    if (ms1[i].first != ms2[i].first)
    {
      RLOG_CPP(1, "Model states indices differ - not interpolating: " << name_ms2 << " " << name_ms2);
      res.clear();
      return res;
    }

    const double blend = (1.0 - s) * ms1[i].second + s * ms2[i].second;
    res.push_back(std::make_pair(ms1[i].first, blend));
  }

  return res;
}



IKTeleOp::IKTeleOp(EntityBase* parent, Rcs::ControllerBase* controller) :
  ComponentBase(parent), ikSolver(controller),
  eStop(false), alpha(0.05), lambda(1.0e-4),
  speedLimitCheck(true), jointLimitCheck(true),
  collisionCheck(true), applySpeedAndAccLimits(true),
  activateTasks(true),
  jointSpeedScaling(0.0, 0.33, parent->getDt())   // Transition in 3 seconds
{
  subscribe("EmergencyStop", &IKTeleOp::onEmergencyStop);
  subscribe("EmergencyRecover", &IKTeleOp::onEmergencyRecover);
  subscribe("Render", &IKTeleOp::onRender);
  subscribe("InitFromState", &IKTeleOp::onInitFromState);
  subscribe("Print", &IKTeleOp::print);
  subscribe("EnableRetargetting", &IKTeleOp::onEnableRetargetting);
  subscribe("EnableTasks", &IKTeleOp::onEnableTasks);
}

const MatNd* IKTeleOp::getJointCommandPtr() const
{
  return ikSolver.getController()->getGraph()->q;
}

const RcsGraph* IKTeleOp::getGraph() const
{
  return ikSolver.getController()->getGraph();
}

RcsGraph* IKTeleOp::getGraph()
{
  return ikSolver.getController()->getGraph();
}

void IKTeleOp::onEnableRetargetting(bool enable)
{
  jointSpeedScaling.setTarget(enable ? 1.0 : 0.0);
}

void IKTeleOp::onEnableTasks(bool enable)
{
  RLOG(0, "onEnableTasks: %s", enable ? "TRUE" : "FALSE");
  this->activateTasks = enable;
}

void IKTeleOp::onEmergencyStop()
{
  RLOG(0, "EmergencyStop");
  this->eStop = true;
  this->alpha = 0.0;
}

void IKTeleOp::onEmergencyRecover()
{
  RLOG(0, "EmergencyRecover");
  this->eStop = false;
}

void IKTeleOp::onInitFromState(const RcsGraph* target)
{
  RLOG(1, "IKTeleOp::onInitFromState()");
  RcsGraph_copy(getGraph(), target);
  ikSolver.getController()->computeCollisionModel();
}

void IKTeleOp::onRender()
{
  getEntity()->publish<std::string,const RcsGraph*>("RenderGraph", "IK", getGraph());
}

void IKTeleOp::setEnableSpeedAccelerationLimit(bool enable)
{
  this->applySpeedAndAccLimits = enable;
  RLOG(0, "Setting applySpeedAndAccLimits to %s", this->applySpeedAndAccLimits ? "TRUE" : "FALSE");
}

void IKTeleOp::setSpeedLimitCheck(bool enable)
{
  this->speedLimitCheck = enable;
  RLOG(0, "Setting speedLimitCheck to %s", this->speedLimitCheck ? "TRUE" : "FALSE");
}

void IKTeleOp::setJointLimitCheck(bool enable)
{
  this->jointLimitCheck = enable;
  RLOG(0, "Setting jointLimitCheck to %s", this->jointLimitCheck ? "TRUE" : "FALSE");
}

void IKTeleOp::setCollisionCheck(bool enable)
{
  this->collisionCheck = enable;
  RLOG(0, "Setting collisionCheck to %s", this->collisionCheck ? "TRUE" : "FALSE");
}

void IKTeleOp::print() const
{
  RcsCollisionModel_fprintCollisions(stdout, ikSolver.getController()->getNarrowPhase(), 1000.0);
  RcsGraph_fprintModelState(stdout, getGraph(), getGraph()->q, NULL, 0);
}

void IKTeleOp::setAlpha(double value)
{
  this->alpha = value;
  RLOG_CPP(0, "Setting alpha to " << this->alpha);
}

double IKTeleOp::getAlpha() const
{
  return this->alpha;
}

void IKTeleOp::setLambda(double value)
{
  this->lambda = value;
  RLOG_CPP(0, "Setting lambda to " << this->lambda);
}

double IKTeleOp::getLambda() const
{
  return this->lambda;
}

void IKTeleOp::onWrenchCommand(std::array<double, 6> wrench, bool inWorldFrame)
{
  if (this->eStop)
  {
    return;
  }

  RCHECK_MSG(inWorldFrame, "Only world frame supported");
  RCHECK(ikSolver.getController()->getTaskDim()>=wrench.size());

  std::array<double, 6> twist;
  MatNd* x_des = MatNd_create(ikSolver.getController()->getTaskDim(), 1);
  MatNd* dx_des = MatNd_createLike(x_des);

  VecNd_copy(x_des->ele, wrench.data(), wrench.size());
  ikSolver.getController()->computeDX(dx_des, x_des);
  VecNd_copy(twist.data(), dx_des->ele, twist.size());
  MatNd_destroyN(2, x_des, dx_des);

  onTwistCommand(twist, inWorldFrame);
}

void IKTeleOp::onTwistCommand(std::array<double, 6> twist, bool inWorldFrame)
{
  if (this->eStop)
  {
    return;
  }

  Rcs::ControllerBase* controller = ikSolver.getController();

  MatNd* dx_des = MatNd_create(controller->getTaskDim(), 1);
  MatNd* lambdaArr = MatNd_createLike(dx_des);
  MatNd* a_des = MatNd_create(controller->getNumberOfTasks(), 1);
  MatNd_setElementsTo(a_des, 1.0);

  // Transform angular velocities into body frame
  if (inWorldFrame)
  {
    const RcsBody* ef = controller->getTask(1)->getEffector();
    RCHECK(ef);
    Vec3d_rotateSelf(twist.data()+3, (double (*)[3])ef->A_BI.rot);
  }

  // From here on, twist is in world coordinates. We can use the "normal" Jacobian.
  size_t dimTaskTwist = controller->getTaskDim(0) + controller->getTaskDim(1);
  VecNd_copy(dx_des->ele, twist.data(), dimTaskTwist);

  const double lambdaOri = 0.01;
  MatNd_setElementsTo(lambdaArr, lambdaOri);
  Vec3d_set(lambdaArr->ele, lambda, lambda, lambda);
  computeIK(a_des, dx_des, lambdaArr);

  MatNd_destroyN(3, a_des, dx_des, lambdaArr);
}


void IKTeleOp::computeIK(const MatNd* a_des_, const MatNd* dx_des_, const MatNd* lambdaArr)
{
  const double dt = getEntity()->getDt();
  ActionResult resMsg;
  RcsGraph* graph = getGraph();
  Rcs::ControllerBase* controller = ikSolver.getController();

  MatNd* dq_des = MatNd_create(graph->dof, 1);
  MatNd* dH = MatNd_create(1, graph->nJ);
  MatNd* qdot = MatNd_createLike(dq_des);
  MatNd* a_des = MatNd_clone(a_des_);

  MatNd_setElementsTo(a_des, this->activateTasks ? 1.0 : 0.0);

  const double globalScaling = jointSpeedScaling.iterate();
  MatNd* dx_des = MatNd_clone(dx_des_);
  MatNd_constMulSelf(dx_des, globalScaling);


  controller->computeJointlimitGradient(dH);
  MatNd_constMulSelf(dH, alpha);
  double det = ikSolver.solveRightInverse(dq_des, dx_des, dH, a_des, lambdaArr);

  MatNd_destroy(dx_des);
  MatNd_destroy(a_des);


  // We treat a singular configuration as an error. Typically, lambda is
  // set to a value larger than zero, therefore this will probably never
  // be a source of error.
  if (det == 0.0)
  {
    resMsg.error = "ERROR";
    resMsg.reason = "Got into a singular posture";
    MatNd_destroyN(3, dq_des, dH, qdot);
    return;
  }

  // Apply speed and acceleration limits only if speed limit check is set
  if (applySpeedAndAccLimits)
  {
    double scale = RcsGraph_getJointSpeedScaling(graph, dq_des, dt, RcsStateFull);
    if (scale < 1.0)
    {
      NLOG(0, "Scaling down joint speeds by factor %f", scale);
      MatNd_constMulSelf(dq_des, 0.99999 * scale);
    }

    // Apply acceleration limits
    const MatNd* qdot_prev = graph->q_dot;
    MatNd_constMul(qdot, dq_des, 1.0/dt);

    int nClippedAcc = RcsGraph_clipJointAccelerations(graph, qdot, qdot_prev,
                                                      dt, 1.0, RcsStateFull);
  }
  else
  {
    MatNd_constMul(qdot, dq_des, 1.0/dt);
  }

  // Update ramp for global joint speed scaling
  // const double globalScaling = jointSpeedScaling.iterate();
  // MatNd_constMulSelf(dq_des, globalScaling);
  // MatNd_constMulSelf(qdot, globalScaling);


  // Scaling down velocities
  MatNd* q_dot_max = MatNd_createLike(graph->q_dot);
  RcsGraph_getSpeedLimits(graph, q_dot_max, RcsStateFull);
  MatNd_destroy(q_dot_max);


  // Integration and FK  including velocities
  MatNd_addSelf(graph->q, dq_des);

  if (applySpeedAndAccLimits)
  {
    RcsGraph_limitJoints(graph, graph->q, RcsStateFull);
  }

  RcsGraph_setState(graph, NULL, qdot);

  // We perform the check after the forward kinematics to consider the
  // pose after the IK step.
  controller->computeCollisionModel();

  // This method is static and doesn't modify the TrajectoryPredictor instance
  bool verbose = true;
  int res = TrajectoryPredictor::checkState(controller, speedLimitCheck, jointLimitCheck,
                                            collisionCheck, verbose, resMsg);

  // We only print this once after the e-stop being triggered, therefore the second comparison
  if ((res<0) && (eStop==false) && applySpeedAndAccLimits)
  {
    RLOG_CPP(0, "res = " << res << " E-Stopping, error = " << resMsg.error
             << " reason = " << resMsg.reason << " developer = " << resMsg.developer);
    getEntity()->publish("EmergencyStop");
  }

  // Clean up
  MatNd_destroyN(3, dq_des, dH, qdot);
}

double IKTeleOp::getJointSpeedScaling() const
{
  return jointSpeedScaling.getPosition();
}

bool IKTeleOp::getTasksActive() const
{
  return activateTasks;
}




/*******************************************************************************
 * Action sending interface
 ******************************************************************************/
std::vector<std::pair<int, double>> IKTeleOp::fingerAnglesFromMetaQuest16(const RcsGraph* desired,
                                 const std::vector<double>& fingerAnglesRight,
                                 const std::vector<double>& fingerAnglesLeft) const
{
  std::vector<std::pair<int, double>> res;   // joint index - joint angle pairs

  if (fingerAnglesRight.empty() || fingerAnglesLeft.empty())
  {
    RLOG(4, "Received empty finger angles array");
    return res;
  }



  // Franka
  const RcsJoint* jnt = RcsGraph_getJointByName(desired, "joint_0_0_right");
  if (jnt)
  {
    RCHECK(fingerAnglesRight.size()>=16);
    for (size_t i=0; i<16; ++i)
    {
      res.push_back(std::make_pair(jnt->jointIndex+i, fingerAnglesRight[i]));
    }

  }

  if (!jnt)   // Kinova Gen3
  {
    jnt = RcsGraph_getJointByName(desired, "finger_joint_right");
    if (jnt)
    {
      // Index finger base joint.
      // input: 0: open, RCS_DEG2RAD(90): close
      // output: 0: open, RCS_DEG2RAD(40): close
      if (fingerAnglesRight.size()>1)
      {
        res.push_back(std::make_pair(jnt->jointIndex, 0.5*fingerAnglesRight[1]));
      }
    }
  }

  if (!jnt)   // Kinova Gen2
  {
    jnt = RcsGraph_getJointByName(desired, "j2s7s300_joint_finger_1_right");
    if (jnt && fingerAnglesRight.size()>1)
    {
      // Index finger base joint.
      // input: 0: open, RCS_DEG2RAD(90): close
      // output: 0: open, RCS_DEG2RAD(40): close
      const double gripperAngle = 0.75*fingerAnglesLeft[1];
      res.push_back(std::make_pair(jnt->jointIndex, gripperAngle));

      jnt = RcsGraph_getJointByName(desired, "j2s7s300_joint_finger_2_right");
      if (jnt)
      {
        res.push_back(std::make_pair(jnt->jointIndex, gripperAngle));
      }
      jnt = RcsGraph_getJointByName(desired, "j2s7s300_joint_finger_3_right");
      if (jnt)
      {
        res.push_back(std::make_pair(jnt->jointIndex, gripperAngle));
      }
    }
  }

  // Franka
  jnt = RcsGraph_getJointByName(desired, "joint_0_0_left");
  if (jnt)
  {
    RCHECK(fingerAnglesLeft.size()>=16);
    for (size_t i=0; i<16; ++i)
    {
      res.push_back(std::make_pair(jnt->jointIndex+i, fingerAnglesLeft[i]));
    }
  }

  if (!jnt)   // Kinova Gen3
  {
    jnt = RcsGraph_getJointByName(desired, "finger_joint_left");
    if (jnt && fingerAnglesLeft.size()>1)
    {
      res.push_back(std::make_pair(jnt->jointIndex, 0.5*fingerAnglesLeft[1]));
    }
  }

  if (!jnt)   // Kinova Gen2
  {
    jnt = RcsGraph_getJointByName(desired, "j2s7s300_joint_finger_1_left");
    if (jnt && fingerAnglesLeft.size()>1)
    {
      const double gripperAngle = 0.75*fingerAnglesLeft[1];
      res.push_back(std::make_pair(jnt->jointIndex, gripperAngle));
      jnt = RcsGraph_getJointByName(desired, "j2s7s300_joint_finger_2_left");
      if (jnt)
      {
        res.push_back(std::make_pair(jnt->jointIndex, gripperAngle));
      }
      jnt = RcsGraph_getJointByName(desired, "j2s7s300_joint_finger_3_left");
      if (jnt)
      {
        res.push_back(std::make_pair(jnt->jointIndex, gripperAngle));
      }
    }
  }

  return res;
}

std::vector<std::pair<int, double>> IKTeleOp::blendAnglesFromModelStates(const RcsGraph* desired,
                                 std::string modelState0,
                                 std::string modelState1,
                                 double s_01) const
{
  std::vector<std::pair<int, double>> res;   // joint index - joint angle pairs
  RcsGraph* graph = ikSolver.getController()->getGraph();

  std::vector<std::pair<int, double>> ms = interpolateModelStates(graph, modelState0, modelState1, s_01);

  for (const auto& msi : ms)
  {
    RCHECK((msi.first>=0) && (msi.first<(int)graph->dof));
    const RcsJoint* jnt_des = &graph->joints[msi.first];   // first is joint id, not index.
    res.push_back(std::make_pair(jnt_des->jointIndex, msi.second));
  }

  return res;
}


std::vector<std::pair<int, double>> IKTeleOp::getFingerAngles(const nlohmann::json& command,
                                                              const RcsGraph* desired) const
{
  std::vector<std::pair<int, double>> res;   // joint index - joint angle pairs


  // Left finger angles from scalar value
  std::string fingersPose0, fingersPose1;
  double s_01 = 0.0;
  bool hasLeftGripperEntry = get_optional<std::string>(command, "left_fingers_pose_0", fingersPose0);
  hasLeftGripperEntry = hasLeftGripperEntry && get_optional<std::string>(command, "left_fingers_pose_1", fingersPose1);
  hasLeftGripperEntry = hasLeftGripperEntry && get_optional<double>(command, "s_left_01", s_01);

  if (hasLeftGripperEntry)
  {
    auto desired_joints = blendAnglesFromModelStates(desired, fingersPose0, fingersPose1, s_01);
    res.insert(res.end(), desired_joints.begin(), desired_joints.end());

    const RcsJoint* jnt = RcsGraph_getJointByName(desired, "joint_gripper_left");
    if (jnt)
    {
      res.push_back(std::make_pair(jnt->jointIndex, s_01));
    }
  }

  // Right finger angles from scalar value
  bool hasRightGripperEntry = get_optional<std::string>(command, "right_fingers_pose_0", fingersPose0);
  hasRightGripperEntry = hasRightGripperEntry && get_optional<std::string>(command, "right_fingers_pose_1", fingersPose1);
  hasRightGripperEntry = hasRightGripperEntry && get_optional<double>(command, "s_right_01", s_01);

  if (hasRightGripperEntry)
  {
    auto desired_joints = blendAnglesFromModelStates(desired, fingersPose0, fingersPose1, s_01);
    res.insert(res.end(), desired_joints.begin(), desired_joints.end());

    const RcsJoint* jnt = RcsGraph_getJointByName(desired, "joint_gripper_right");
    if (jnt)
    {
      res.push_back(std::make_pair(jnt->jointIndex, s_01));
    }
  }



  // Overwrite finger joint angles
  std::vector<double> fingerAnglesRight, fingerAnglesLeft;
  bool hasRight = get_optional<std::vector<double>>(command, "finger_angles_right", fingerAnglesRight);
  bool hasLeft = get_optional<std::vector<double>>(command, "finger_angles_left", fingerAnglesLeft);

  if (hasRight && hasLeft && (!hasRightGripperEntry) && (!hasLeftGripperEntry))
  {
    auto desired_joints = fingerAnglesFromMetaQuest16(desired, fingerAnglesRight, fingerAnglesLeft);
    res.insert(res.end(), desired_joints.begin(), desired_joints.end());
  }


  return res;
}







// std::vector<std::pair<int, double>> IKTeleOp::getFingerAngles(const nlohmann::json& command,
//                                                               const RcsGraph* desired) const
// {
//   std::vector<std::pair<int, double>> res;   // joint index - joint angle pairs

//   // Overwrite finger joint angles
//   std::vector<double> fingerAnglesRight, fingerAnglesLeft;
//   bool hasRight = get_optional<std::vector<double>>(command, "finger_angles_right", fingerAnglesRight);
//   bool hasLeft = get_optional<std::vector<double>>(command, "finger_angles_left", fingerAnglesLeft);

//   if (hasRight && hasLeft)
//   {
//     res = fingerAnglesFromMetaQuest16(desired, fingerAnglesRight, fingerAnglesLeft);

//   }
//   else
//   {
//     std::string fingersPose0, fingersPose1;
//     double s_01;
//     bool hasEntry = get_optional<std::string>(command, "left_fingers_pose_0", fingersPose0);
//     hasEntry = hasEntry && get_optional<std::string>(command, "left_fingers_pose_1", fingersPose1);
//     hasEntry = hasEntry && get_optional<double>(command, "s_left_01", s_01);

//     if (hasEntry)
//     {
//       res = blendAnglesFromModelStates(desired, fingersPose0, fingersPose1, s_01);
//     }

//     hasEntry = get_optional<std::string>(command, "right_fingers_pose_0", fingersPose0);
//     hasEntry = hasEntry && get_optional<std::string>(command, "right_fingers_pose_1", fingersPose1);
//     hasEntry = hasEntry && get_optional<double>(command, "s_right_01", s_01);

//     if (hasEntry)
//     {
//       auto desired_joints = blendAnglesFromModelStates(desired, fingersPose0, fingersPose1, s_01);
//       res.insert(res.end(), desired_joints.begin(), desired_joints.end());
//     }
//   }


//   double gripperAngleRight=-1.0;
//   bool hasRightGripper = get_optional<double>(command, "gripper_angle_right", gripperAngleRight);
//   if (hasRightGripper)
//   {
//     const RcsJoint* jnt = RcsGraph_getJointByName(desired, "joint_gripper_right");
//     if (jnt)
//     {
//       res.push_back(std::make_pair(jnt->jointIndex, gripperAngleRight));
//     }
//   }

//   double gripperAngleLeft=-1.0;
//   bool hasLeftGripper = get_optional<double>(command, "gripper_angle_left", gripperAngleLeft);
//   if (hasLeftGripper)
//   {
//     const RcsJoint* jnt = RcsGraph_getJointByName(desired, "joint_gripper_left");
//     if (jnt)
//     {
//       res.push_back(std::make_pair(jnt->jointIndex, gripperAngleLeft));
//     }
//   }

//   return res;
// }

void IKTeleOp::sendActionBiManualPose(std::vector<double> poseRight,
                                      std::vector<double> poseLeft)
{
  if (this->eStop)
  {
    return;
  }

  if (poseLeft.size()==9)
  {
    double rm[3][3];
    Vec3d_copy(rm[0], poseLeft.data()+3);
    Vec3d_copy(rm[1], poseLeft.data()+6);
    Vec3d_crossProduct(rm[2], rm[0], rm[1]);

    std::vector<double> pose6(6);
    Vec3d_copy(pose6.data(), poseLeft.data());
    Mat3d_toEulerAngles(pose6.data()+3, rm);

    poseLeft = pose6;
  }

  if (poseRight.size()==9)
  {
    double rm[3][3];
    Vec3d_copy(rm[0], poseRight.data()+3);
    Vec3d_copy(rm[1], poseRight.data()+6);
    Vec3d_crossProduct(rm[2], rm[0], rm[1]);

    std::vector<double> pose6(6);
    Vec3d_copy(pose6.data(), poseRight.data());
    Mat3d_toEulerAngles(pose6.data()+3, rm);

    poseRight = pose6;
  }

  if ((poseLeft.size()!=6) || (poseRight.size()!=6))
  {
    RLOG(1, "Desired pose is invalid.");
    return;
  }

  MatNd* a_des = MatNd_create(ikSolver.getController()->getNumberOfTasks(), 1);
  MatNd* x_des = MatNd_create(ikSolver.getController()->getTaskDim(), 1);
  MatNd* dx_des = MatNd_createLike(x_des);
  MatNd* lambdaArr = MatNd_createLike(dx_des);
  MatNd_setElementsTo(lambdaArr, this->lambda);

  const double lambdaOri = 0.01;
  int task_idx = -1;
  int array_idx = -1;
  std::string taskName;

  // Left arm position
  taskName = "left_hand_pos";
  task_idx = ikSolver.getController()->getTaskIndex(taskName.c_str());
  array_idx = ikSolver.getController()->getTaskArrayIndex(taskName.c_str());
  if ((array_idx != -1) && (task_idx!=-1))
  {
    const double* target = poseLeft.data();
    Vec3d_copy(x_des->ele + array_idx, target);
    a_des->ele[task_idx] = 1.0;
  }

  // Left arm orientation
  taskName = "left_hand_ori";
  task_idx = ikSolver.getController()->getTaskIndex(taskName.c_str());
  array_idx = ikSolver.getController()->getTaskArrayIndex(taskName.c_str());
  if (array_idx != -1 && (task_idx!=-1))
  {
    const double* target = poseLeft.data() + 3;
    Vec3d_copy(x_des->ele + array_idx, target);
    a_des->ele[task_idx] = 1.0;
    Vec3d_setElementsTo(lambdaArr->ele + array_idx, lambdaOri);
  }

  // Right arm position
  taskName = "right_hand_pos";
  task_idx = ikSolver.getController()->getTaskIndex(taskName.c_str());
  array_idx = ikSolver.getController()->getTaskArrayIndex(taskName.c_str());
  if (array_idx != -1 && (task_idx!=-1))
  {
    const double* target = poseRight.data();
    Vec3d_copy(x_des->ele + array_idx, target);
    a_des->ele[task_idx] = 1.0;
  }

  // Right arm orientation
  taskName = "right_hand_ori";
  task_idx = ikSolver.getController()->getTaskIndex(taskName.c_str());
  array_idx = ikSolver.getController()->getTaskArrayIndex(taskName.c_str());
  if (array_idx != -1 && (task_idx!=-1))
  {

    const Rcs::Task* task = ikSolver.getController()->getTask(task_idx);
    const double* target = poseRight.data() + 3;

    if (task->getClassName()=="POLAR")
    {
      double rm[3][3];
      Mat3d_fromEulerAngles(rm, target);
      Vec3d_getPolarAngles(x_des->ele + array_idx, rm[1]);
    }
    else if (task->getClassName()=="Inclination")
    {
      x_des->ele[array_idx] = M_PI_2;
    }
    else
    {
      Vec3d_copy(x_des->ele + array_idx, target);
      Vec3d_setElementsTo(lambdaArr->ele + array_idx, lambdaOri);
    }

    a_des->ele[task_idx] = 1.0;
  }

  ikSolver.getController()->computeDX(dx_des, x_des);

  computeIK(a_des, dx_des, lambdaArr);

  MatNd_destroyN(4, a_des, x_des, dx_des, lambdaArr);
}

bool IKTeleOp::sendActionHumanPose(const nlohmann::json& command,
                                   std::vector<std::string> taskNames)
{
  if (this->eStop || command.empty())
  {
    return false;
  }

  Rcs::ControllerBase* controller = ikSolver.getController();
  MatNd* x_des = MatNd_create(controller->getTaskDim(), 1);
  MatNd* dx_des = MatNd_createLike(x_des);
  MatNd* a_des = MatNd_create(controller->getNumberOfTasks(), 1);

  controller->computeX(x_des);

  const double lambdaOri = 0.01;
  MatNd* lambdaArr = MatNd_createLike(x_des);
  MatNd_setElementsTo(lambdaArr, this->lambda);

  for (const auto& taskName : taskNames)
  {
    int array_idx = controller->getTaskArrayIndex(taskName.c_str());
    int task_idx = controller->getTaskIndex(taskName.c_str());

    if ((array_idx != -1) && (task_idx!=-1))
    {
      auto vec = command[taskName].get<std::vector<double>>();

      // Polar angles calculation for arms: the z-axis is pointing from the
      // link's origin outwards (to distal direction).
      if (vec.size()==3)
      {
        auto vec = command[taskName].get<std::vector<double>>();
        Vec3d_getPolarAngles(x_des->ele + array_idx, vec.data());
        a_des->ele[task_idx] = 1.0;
        RCHECK_MSG(controller->getTask(taskName)->getClassName()=="POLAR", "%s: %s",
                   controller->getTask(taskName)->getName().c_str(),
                   controller->getTask(taskName)->getClassName().c_str());
      }
      else if (vec.size()==6)
      {
        double rm[3][3];
        Vec3d_copy(rm[0], vec.data());
        Vec3d_copy(rm[1], vec.data()+3);
        Vec3d_crossProduct(rm[2], rm[0], rm[1]);
        Mat3d_toEulerAngles(x_des->ele + array_idx, rm);
        Vec3d_setElementsTo(lambdaArr->ele + array_idx, lambdaOri);   // Regularize 3d orientations
        a_des->ele[task_idx] = 1.0;
        RCHECK_MSG(controller->getTask(taskName)->getClassName()=="ABC", "%s: %s",
                   controller->getTask(taskName)->getName().c_str(),
                   controller->getTask(taskName)->getClassName().c_str());
      }
      else
      {
        RLOG_CPP(0, "Unsupported task dimension (should be 3 or 6): " << vec.size());
      }
    }
  }

  controller->computeDX(dx_des, x_des);

  computeIK(a_des, dx_des, lambdaArr);

  MatNd_destroyN(4, a_des, x_des, dx_des, lambdaArr);

  return true;
}

bool IKTeleOp::sendAction(const nlohmann::json& command,
                          RcsGraph* desired,
                          ActionScene* scene)
{
  if (!command.is_object())
  {
    RLOG_CPP(1, "Expected top-level nlohmann::json object");
    RcsGraph_setState(desired, nullptr, nullptr);
    return false;
  }

  ActionType input_type = ActionType::None;

  bool success = get_optional<ActionType>(command, "input_type", input_type);
  if (!success)
  {
    RcsGraph_setState(desired, nullptr, nullptr);
    return false;
  }

  switch (input_type)
  {
    case ActionType::PolarRetarget:
    {
      // IK step
      std::vector<std::string> taskNames = { "right_upperarm_ori",
                                             "right_forearm_ori",
                                             "right_hand_ori",
                                             "left_upperarm_ori",
                                             "left_forearm_ori",
                                             "left_hand_ori",
                                             "head_ori"
                                           };
      sendActionHumanPose(command, taskNames);
      break;
    }

    case ActionType::BiManualPose:
    {
      RLOG_CPP(3, "command: " << command.dump(2));
      auto leftHandPose = get_required<std::vector<double>>(command, "left_hand_pose");
      auto rightHandPose = get_required<std::vector<double>>(command, "right_hand_pose");
      sendActionBiManualPose(rightHandPose, leftHandPose);
      break;
    }

    case ActionType::JointPosition:
    {
      RLOG_CPP(3, "command: " << command.dump(2));
      const Rcs::ControllerBase* c = ikSolver.getController();
      MatNd* x_des = MatNd_create(c->getTaskDim(), 1);
      MatNd* dx_des = MatNd_create(c->getTaskDim(), 1);
      MatNd* a_des = MatNd_create(c->getNumberOfTasks(), 1);

      /*
      {
        "joints": {
        "joint1": 1.1,
        "joint2": 2.2,
        "joint3": 3.3
        },
      ...
      }
      */
      if (command.contains("joints") && command["joints"].is_object())
      {
        const nlohmann::json& joints = command["joints"];

        for (nlohmann::json::const_iterator it = joints.begin(); it != joints.end(); ++it)
        {
          const std::string jointName = it.key();
          const nlohmann::json& jointValue = it.value();

          if (!jointValue.is_number())
          {
            throw std::runtime_error("Joint '" + jointName + "' must be a number");
          }

          const double jointPos = jointValue.get<double>();

          const Rcs::Task* ti = c->getTask(jointName);
          if (ti)
          {
            const int array_idx = c->getTaskArrayIndex(ti);
            const int task_idx = c->getTaskIndex(ti);
            RCHECK((array_idx != -1) && (task_idx != -1));
            x_des->ele[array_idx] = jointPos;
            a_des->ele[task_idx] = 1.0;
          }
        }
      }

      c->computeDX(dx_des, x_des, a_des);

      computeIK(a_des, dx_des, nullptr);

      MatNd_destroyN(3, x_des, dx_des, a_des);
      break;
    }

    default:
    {
      RLOG_CPP(1, "Unsupported input type: " << static_cast<int>(input_type));
      RcsGraph_setState(desired, nullptr, nullptr);
      return false;
    }

  }   // switch


  // Overwrite finger joint angles
  auto finger_angles = getFingerAngles(command, desired);
  for (const auto& jpair : finger_angles)
  {
    MatNd_set(desired->q, jpair.first, 0, jpair.second);
  }




  return true;
}


}   // namespace aff
