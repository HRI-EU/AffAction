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

#include "RetargetComponent.h"

#include <Rcs_typedef.h>
#include <Rcs_macros.h>
#include <Rcs_resourcePath.h>
#include <Rcs_graphParser.h>
#include <Rcs_Mat3d.h>
#include <Rcs_Vec3d.h>
#include <Rcs_timer.h>

#include <COSNode.h>
#include <SphereNode.h>
#include <TextNode3D.h>

#include <sstream>
#include <iomanip>
#include <thread>


#define NUM_MARKERS              (32)
#define INIT_COUNT_DOWN_TIME     (2.0)
#define TIMEOUT_TIME             (3.0)
#define MARKER_SPHERES_POINTERS
#define WITH_NEWTON_EULER

#if defined (WITH_NEWTON_EULER)
#include <PhysicsFactory.h>
#include <KineticSimulation.h>
#endif

/*
Index   Joint name  Parent joint
0   PELVIS  -
1   SPINE_NAVAL     PELVIS
2   SPINE_CHEST     SPINE_NAVAL
3   NECK            SPINE_CHEST
4   CLAVICLE_LEFT   SPINE_CHEST
5   SHOULDER_LEFT   CLAVICLE_LEFT
6   ELBOW_LEFT      SHOULDER_LEFT
7   WRIST_LEFT      ELBOW_LEFT
8   HAND_LEFT       WRIST_LEFT
9   HANDTIP_LEFT    HAND_LEFT
10  THUMB_LEFT      WRIST_LEFT
11  CLAVICLE_RIGHT  SPINE_CHEST
12  SHOULDER_RIGHT  CLAVICLE_RIGHT
13  ELBOW_RIGHT     SHOULDER_RIGHT
14  WRIST_RIGHT     ELBOW_RIGHT
15  HAND_RIGHT      WRIST_RIGHT
16  HANDTIP_RIGHT   HAND_RIGHT
17  THUMB_RIGHT     WRIST_RIGHT
18  HIP_LEFT        PELVIS
19  KNEE_LEFT       HIP_LEFT
20  ANKLE_LEFT      KNEE_LEFT
21  FOOT_LEFT       ANKLE_LEFT
22  HIP_RIGHT       PELVIS
23  KNEE_RIGHT      HIP_RIGHT
24  ANKLE_RIGHT     KNEE_RIGHT
25  FOOT_RIGHT      ANKLE_RIGHT
26  HEAD            NECK
27  NOSE            HEAD
28  EYE_LEFT        HEAD
29  EAR_LEFT        HEAD
30  EYE_RIGHT       HEAD
31  EAR_RIGHT       HEAD
*/
namespace aff
{

typedef enum
{
  PELVIS         = 0,
  SPINE_NAVAL,
  SPINE_CHEST,
  NECK,
  CLAVICLE_LEFT,
  SHOULDER_LEFT,
  ELBOW_LEFT,
  WRIST_LEFT,
  HAND_LEFT,
  HANDTIP_LEFT,
  THUMB_LEFT,
  CLAVICLE_RIGHT,
  SHOULDER_RIGHT,
  ELBOW_RIGHT,
  WRIST_RIGHT,
  HAND_RIGHT,
  HANDTIP_RIGHT,
  THUMB_RIGHT,
  HIP_LEFT,
  KNEE_LEFT,
  ANKLE_LEFT,
  FOOT_LEFT,
  HIP_RIGHT,
  KNEE_RIGHT,
  ANKLE_RIGHT,
  FOOT_RIGHT,
  HEAD,
  NOSE,
  EYE_LEFT,
  EAR_LEFT,
  EYE_RIGHT,
  EAR_RIGHT

} BodyName;


RetargetComponent::Pose::Pose(EntityBase* parent, BodyType bType_, int id) :
  ComponentBase(parent), poseId(id), controller(getConfigFileName(bType_)),
  ikSolver(&controller), visGraph(RcsGraph_clone(controller.getGraph())),
  a_des(NULL), x_des(NULL), countDown(INIT_COUNT_DOWN_TIME), lastUpdate(-1000.0),
  graphicsInitialized(false), kineticsEnabled(false), poseState(PoseState::Hidden),
  bType(bType_), numIkIterationsPerStep(3), bodyHeight(0.0)
{
  this->a_des = MatNd_create(controller.getNumberOfTasks(), 1);
  controller.readActivationsFromXML(this->a_des);

  this->x_des = MatNd_create(controller.getTaskDim(), 1);
  controller.computeX(this->x_des);

  nameIdMap = getNameIdMap(bType);

  HTr_setIdentity(&A_CI);

  subscribe("Render", &RetargetComponent::Pose::onRender);
  subscribe("GraphicsWindowFeedback", &RetargetComponent::Pose::onGraphicsWindowFeedback);
  subscribe("RetargetInitialize", &RetargetComponent::Pose::initialize);
  subscribe("RetargetToggleKinetics", &RetargetComponent::Pose::onToggleKinetics);

  frames.resize(NUM_MARKERS);

  for (size_t i=0; i<NUM_MARKERS; ++i)
  {
    HTr_setIdentity(&frames[i]);
    frames[i].org[2] = -100.0;
  }

  rawFrames = frames;
}

RetargetComponent::Pose::~Pose()
{
  MatNd_destroy(a_des);
  MatNd_destroy(x_des);
  RcsGraph_destroy(visGraph);

  // That's the parent node of all markers and texts
  std::string graphId = getGraphIdStr();
  getEntity()->publish("RenderCommand", std::string("erase"), graphId);
}

void RetargetComponent::Pose::onToggleKinetics()
{
  kineticsEnabled = !kineticsEnabled;

  if (!kineticsEnabled)
  {
    RLOG(0, "Resetting colors");
    const RcsGraph* src = controller.getGraph();
    RcsGraph* dst = this->visGraph;
    for (unsigned int i = 0; i < dst->nBodies; ++i)
    {
      const RcsBody* bSrc = &src->bodies[i];
      RcsBody* bDst = &dst->bodies[i];
      for (unsigned int i = 0; i < bSrc->nShapes; i++)
      {
        strcpy(bDst->shapes[i].color, bSrc->shapes[i].color);
        RLOG(0, "Setting color of body %s shape %d to %s",
             bDst->name, i, bDst->shapes[i].color);
      }
    }
  }
}

std::string RetargetComponent::Pose::getGraphIdStr() const
{
  std::string graphId = "Osim_" + std::to_string(poseId);
  return graphId;
}

RetargetComponent::PoseState RetargetComponent::Pose::getPoseState() const
{
  return poseState;
}

void RetargetComponent::Pose::setPoseState(PoseState newState)
{
  poseState = newState;
}

std::string RetargetComponent::Pose::getConfigFileName(BodyType bType)
{
  std::string cfgName;

  switch (bType)
  {
    case BodyType::OpenSim:
      cfgName = "c_retarget.xml";
      break;

    case BodyType::DexBot:
      cfgName = "c_dexbot.xml";
      break;
    default:
      RFATAL("Unknown body type: %d", (int)bType);
  }

  return cfgName;
}

// This map matches the Azure's landmark ids with the bodies in the graph
// that are to be retargetted:
// 0  PELVIS
// 12 SHOULDER_RIGHT
// 5  SHOULDER_LEFT
// 13 ELBOW_RIGHT
// 6  ELBOW_LEFT
// 14 WRIST_RIGHT
// 7  WRIST_LEFT
// 15 HAND_RIGHT
// 8  HAND_LEFT
std::map<std::string, int> RetargetComponent::Pose::getNameIdMap(BodyType bType)
{
  std::map<std::string, int> res;

  switch (bType)
  {
    // The below correspondences relate to a model from OpenSim.
    case BodyType::OpenSim:
      res["pelvis"] = 0;

      res["femur_r_f1"] = 22;
      res["tibia_r"] = 23;
      res["talus_r"] = 24;
      res["toes_r"] = 25;

      res["femur_l_f1"] = 18;
      res["tibia_l"] = 19;
      res["talus_l"] = 20;
      res["toes_l"] = 21;

      res["humerus_r_f2"] = 12;
      res["ulna_r"] = 13;
      res["hand_r"] = 15;

      res["humerus_l_f2"] = 5;
      res["ulna_l"] = 6;
      res["hand_l"] = 8;

      res["head_and_neck_f2"] = 3;
      res["nose"] = 27;
      break;

    // The below correspondences relate to a model from OpenSim.
    case BodyType::DexBot:
      res["lbr_link_2_R"] = 12;
      res["lbr_link_4_R"] = 13;
      res["lbr_link_6_R"] = 14;

      res["lbr_link_2_L"] = 5;
      res["lbr_link_4_L"] = 6;
      res["lbr_link_6_L"] = 7;
      break;
    default:
      RFATAL("Unknown body type: %d", (int)bType);
  }

  return res;
}

void RetargetComponent::Pose::initialize(double seconds)
{
  // RLOG(0, "Setting countdown of pose %d to %f", poseId, seconds);
  countDown = seconds;
}

bool RetargetComponent::Pose::isValid() const
{
  const double timeout = TIMEOUT_TIME;
  double age = getEntity()->getTime() - lastUpdate;
  RLOG(1, "[%d]: age is %.1f sec, poseState is %d", poseId, age, (int) poseState);
  return (age < timeout) ? true : false;
}

void RetargetComponent::Pose::makeValid()
{
  lastUpdate = getEntity()->getTime();
  countDown = INIT_COUNT_DOWN_TIME;
}

void RetargetComponent::Pose::onRender()
{
  const bool poseValid = isValid();
  const PoseState ps = getPoseState();

  switch (ps)
  {
    case PoseState::Visible:
      if (!poseValid)
      {
        std::string graphId = getGraphIdStr();
        getEntity()->publish("RenderCommand", graphId, std::string("hide"));
        setPoseState(PoseState::Hidden);
        for (size_t i=0; i<NUM_MARKERS; ++i)
        {
          HTr_setIdentity(&frames[i]);
          frames[i].org[2] = -100.0;
        }
        countDown = INIT_COUNT_DOWN_TIME;
      }
      else
      {
        // double bHeight = computeHeight();
        // if (fabs(bHeight-bodyHeight)>0.1)   // 10cm mismatch
        // {
        //   setPoseState(PoseState::Visible);
        //   countDown = INIT_COUNT_DOWN_TIME;
        //   lastUpdate = -10.0;
        //   double scale = bHeight/bodyHeight;
        //   RcsGraph_scaleSubTree(controller.getGraph(), 0, scale);
        //   RcsGraph_scaleSubTree(visGraph, 0, scale);
        //   getEntity()->publish("RenderCommand", getGraphIdStr(), std::string("erase"));
        //   RLOG(0, "[%d]: Body height: %f   new height: %f - Resizing with factor %f",
        //        poseId, bodyHeight, bHeight, scale);
        //   bodyHeight = bHeight;
        // }
        // else
        {
          getEntity()->publish<std::string, const RcsGraph*>("RenderGraph", getGraphIdStr(), visGraph);
        }

      }
      break;

    case PoseState::Hidden:
      if (poseValid)
      {
        initialize(INIT_COUNT_DOWN_TIME);
        setPoseState(PoseState::Visible);

        // Adjust body height. We assume that the body model comes first, and
        // only scale the subtree. This avoids that the camera transform is also scaled.
        // \todo: We should fix this properly.
        if (true==false)
        {
          const double defaultHeight = 1.82;
          double scale = bodyHeight/defaultHeight;
          RcsGraph_scaleSubTree(controller.getGraph(), 0, scale);
          RcsGraph_scaleSubTree(visGraph, 0, scale);
          getEntity()->publish("RenderCommand", getGraphIdStr(), std::string("erase"));
          RLOG(0, "[%d]: Body height: %f   default height: %f - Resizing with factor %f",
               poseId, bodyHeight, defaultHeight, scale);
        }

        // Show it again
        std::string graphId = getGraphIdStr();
        getEntity()->publish("RenderCommand", graphId, std::string("show"));
      }
      break;

    default:
      RFATAL("Unknown PoseState: %d", (int) ps);
  }






  const double tmc = 0.5;
  filterVisualGraph(tmc);

#if defined (WITH_NEWTON_EULER)
  if (kineticsEnabled)
  {
    Rcs::PhysicsBase* sim = Rcs::PhysicsFactory::create("NewtonEuler", visGraph, (const char*)NULL);
    RCHECK(sim);
    sim->setParameter(Rcs::PhysicsBase::Simulation, "Euler", "Integrator", 0.0);

    Rcs::KineticSimulation* ksim = dynamic_cast<Rcs::KineticSimulation*>(sim);
    RCHECK(ksim);
    MatNd* q = MatNd_clone(visGraph->q);
    MatNd* q_dot = MatNd_clone(visGraph->q_dot);
    MatNd* q_ddot = MatNd_createLike(visGraph->q_dot);
    MatNd* T_curr = MatNd_createLike(visGraph->q_dot);
    ksim->simulate(0.001, q, q_dot, q_ddot, NULL, false);
    ksim->getJointTorque(T_curr, RcsStateFull);

    REXEC(1)
    {
      if (getGraphIdStr() == "Osim_1")
      {
        MatNd_printCommentDigits("jointTorque", T_curr, 6);
      }
    }

    // Copy colors
    const RcsGraph* src = ksim->getGraph();
    RcsGraph* dst = this->visGraph;
    for (unsigned int i = 0; i < dst->nBodies; ++i)
    {
      const RcsBody* bSrc = &src->bodies[i];
      RcsBody* bDst = &dst->bodies[i];
      for (unsigned int i = 0; i < bSrc->nShapes; i++)
      {
        strcpy(bDst->shapes[i].color, bSrc->shapes[i].color);
        //RLOG(0, "Setting color of %s to %s", bDst->name, bDst->shapes[i].color);
      }
    }


    MatNd_destroyN(4, q, q_dot, q_ddot, T_curr);

    delete sim;
  }
#endif
}

// Callback once a GraphNode has been created completely, and can be adressed in the
// GraphicsWindow class
void RetargetComponent::Pose::onGraphicsWindowFeedback(std::string feedbackType,
                                                       std::string graphId)
{
  RLOG_CPP(5, "fbType: " << feedbackType << " graphId: " << graphId);

  if ((feedbackType=="GraphNodeCreated") && (graphId==getGraphIdStr()))
  {
    std::vector<std::string> gCol;
    gCol.push_back("RED");
    gCol.push_back("GREEN");
    gCol.push_back("BLUE");
    gCol.push_back("PEWTER");
    gCol.push_back("BRONZE");
    gCol.push_back("BRASS");
    gCol.push_back("EMERALD");
    gCol.push_back("JADE");
    gCol.push_back("RUBY");
    gCol.push_back("TURQUOISE");
    //getEntity()->publish("SetObjectColor", graphId, std::string(), gCol[poseId%gCol.size()]);


    if (!graphicsInitialized)
    {
      graphicsInitialized = true;
      osg::ref_ptr<osg::Group> ndGrp = new osg::Group;
      ndGrp->setName("BodyLM_" + std::to_string(poseId));

      for (size_t i = 0; i < NUM_MARKERS; ++i)
      {
        osg::ref_ptr<Rcs::SphereNode> nd = new Rcs::SphereNode(Vec3d_zeroVec(), 0.015);
#if defined (MARKER_SPHERES_POINTERS)
        nd->makeDynamic(frames[i].org);
#endif
        std::string nodeName = "BodyLM_" + std::to_string(poseId) +
                               std::string("_") + std::to_string(i);
        nd->setName(nodeName);
        nd->setMaterial("WHITE");

        osg::ref_ptr<Rcs::TextNode3D> text = new Rcs::TextNode3D(std::to_string(i));
        text->setPosition(0.0, 0.0, 0.05);
        nd->addChild(text.get());

        ndGrp->addChild(nd.get());
      }

      //getEntity()->publish<osg::ref_ptr<osg::Node>, std::string, std::string>("AddChildNode", ndGrp, graphId, std::string());
      getEntity()->publish<osg::ref_ptr<osg::Node>>("AddNode", ndGrp);
    }

  }

}

// The OpenSim model is 1.69m high
double RetargetComponent::Pose::computeHeight() const
{
  double height = 0.0;

  height += Vec3d_distance(frames[ANKLE_LEFT].org, frames[KNEE_LEFT].org);
  height += Vec3d_distance(frames[KNEE_LEFT].org, frames[HIP_LEFT].org);
  height += Vec3d_distance(frames[PELVIS].org, frames[SPINE_NAVAL].org);
  height += Vec3d_distance(frames[SPINE_NAVAL].org, frames[SPINE_CHEST].org);
  height += Vec3d_distance(frames[SPINE_CHEST].org, frames[NECK].org);
  height += Vec3d_distance(frames[NECK].org, frames[HEAD].org);
  height += Vec3d_distance(frames[HEAD].org, frames[NOSE].org);

  height *= 1.8 / 1.62;   // heuristic correction

  return height;
}

void RetargetComponent::Pose::retarget(const std::vector<HTr>& frames_)
{
  RCHECK_MSG(frames_.size() == NUM_MARKERS, "%zu != %d", frames_.size(), NUM_MARKERS);
  lastUpdate = getEntity()->getTime();
  rawFrames = frames_;
  frames = frames_;

  // Estimate the body height of the pose
  double bHeight = computeHeight();

  if (bodyHeight==0.0)
  {
    bodyHeight = bHeight;
  }
  else
  {
    bodyHeight = 0.95*bodyHeight + 0.05*bHeight;
  }

  for (size_t i = 0; i < frames.size(); ++i)
  {
    Vec3d_transRotateSelf(frames[i].org, A_CI.rot);   // I_r_CB
    Vec3d_addSelf(frames[i].org, A_CI.org);           // I_r_IB
    Mat3d_postMulSelf(frames[i].rot, A_CI.rot);       // A_BI = A_BC A_CI
  }

  const double dt_cntdown = 0.2;

  if (countDown > 0.0 && bType== BodyType::OpenSim)
  {
    retarget_init(frames);
  }
  else
  {
    retarget_IK(frames);
  }

  this->countDown -= dt_cntdown;

#if !defined (MARKER_SPHERES_POINTERS)
  if (graphicsInitialized && poseState==PoseState::Visible)
  {
    for (size_t i = 0; i < frames.size(); ++i)
    {
      std::string nodeName = "BodyLM_" + std::to_string(poseId) + std::string("_") + std::to_string(i);
      getEntity()->publish("SetNodeTransform", nodeName, frames[i]);
    }
  }
#endif
}

void RetargetComponent::Pose::retarget_init(const std::vector<HTr>& frames)
{
  // Pelvis: z-up, y-forward, x-right
  {
    HTr pelvis = frames[0];
    HTr spine = frames[1];
    HTr hip_r = frames[22];
    HTr hip_l = frames[18];

    HTr A_bdy;
    HTr_copy(&A_bdy, &pelvis);
    Vec3d_sub(A_bdy.rot[2], spine.org, pelvis.org);   // z
    Vec3d_normalizeSelf(A_bdy.rot[2]);
    Vec3d_sub(A_bdy.rot[0], hip_l.org, hip_r.org);  // helper axis
    Vec3d_normalizeSelf(A_bdy.rot[0]);
    Vec3d_crossProduct(A_bdy.rot[1], A_bdy.rot[0], A_bdy.rot[2]);   // y
    Vec3d_normalizeSelf(A_bdy.rot[1]);
    Vec3d_crossProduct(A_bdy.rot[0], A_bdy.rot[1], A_bdy.rot[2]);

    RcsBody* bdy = RcsGraph_getBodyByName(controller.getGraph(), "pelvis");
    RCHECK(bdy);
    double rbj[6];
    RcsGraph_relativeRigidBodyDoFs(controller.getGraph(), bdy, &A_bdy, NULL, rbj);
    bool ok = RcsGraph_setRigidBodyDoFs(controller.getGraph(), bdy, rbj);
    RCHECK(ok);
  }

  // Elbow right
  {
    HTr sh = frames[12];
    HTr el = frames[13];
    HTr wr = frames[14];

    double ua[3], la[3];
    Vec3d_sub(ua, sh.org, el.org);
    Vec3d_sub(la, wr.org, el.org);
    double ang = M_PI - Vec3d_diffAngle(ua, la);
    bool ok = RcsGraph_setJoint(controller.getGraph(), "elbow_flex_r", ang);
    RCHECK(ok);
  }

  // Elbow left
  {
    HTr sh = frames[5];
    HTr el = frames[6];
    HTr wr = frames[7];

    double ua[3], la[3];
    Vec3d_sub(ua, sh.org, el.org);
    Vec3d_sub(la, wr.org, el.org);
    double ang = M_PI - Vec3d_diffAngle(ua, la);
    bool ok = RcsGraph_setJoint(controller.getGraph(), "elbow_flex_l", ang);
    RCHECK(ok);
  }

  // Chest frame: x-forward, z-up, y-left
  HTr A_chest = frames[2];
  {
    double* up = A_chest.rot[2];
    double* left = A_chest.rot[1];
    double* fwd = A_chest.rot[0];

    Vec3d_sub(up, frames[3].org, frames[2].org);
    Vec3d_normalizeSelf(up);

    // That's only a helper axis - pointing right

    //Vec3d_sub(left, frames[11].org, frames[4].org);
    Vec3d_sub(left, frames[12].org, frames[5].org);
    Vec3d_normalizeSelf(left);

    Vec3d_crossProduct(fwd, up, left);
    Vec3d_normalizeSelf(fwd);

    Vec3d_crossProduct(left, up, fwd);
    RCHECK(Mat3d_isValid(A_chest.rot));
  }

  // Left shoulder abduction
  {
    double tmp[3];
    Vec3d_sub(tmp, frames[5].org, frames[6].org);   // el - sh
    Vec3d_rotateSelf(tmp, A_chest.rot);             // in chest frame
    tmp[0] = 0.0;                                   // in chest xy plane
    double ang = -Vec3d_diffAngle(Vec3d_ez(), tmp);
    bool ok = RcsGraph_setJoint(controller.getGraph(), "arm_add_l", ang);
    RCHECK(ok);
  }

  // Left shoulder flexion
  {
    double tmp[3];
    Vec3d_sub(tmp, frames[5].org, frames[6].org);   // el - sh
    Vec3d_rotateSelf(tmp, A_chest.rot);             // in chest frame
    tmp[1] = 0.0;                                   // in chest xz plane
    double ang = Vec3d_diffAngle(Vec3d_ex(), tmp) - M_PI_2;
    bool ok = RcsGraph_setJoint(controller.getGraph(), "arm_flex_l", ang);
    RCHECK(ok);
  }

  // Right shoulder abduction
  {
    double tmp[3];
    Vec3d_sub(tmp, frames[12].org, frames[13].org); // el - sh
    Vec3d_rotateSelf(tmp, A_chest.rot);             // in chest frame
    tmp[0] = 0.0;                                   // in chest xy plane
    double ang = -Vec3d_diffAngle(Vec3d_ez(), tmp);
    bool ok = RcsGraph_setJoint(controller.getGraph(), "arm_add_r", ang);
    RCHECK(ok);
  }

  // Right shoulder flexion
  {
    double tmp[3];
    Vec3d_sub(tmp, frames[12].org, frames[13].org); // el - sh
    Vec3d_rotateSelf(tmp, A_chest.rot);             // in chest frame
    tmp[1] = 0.0;                                   // in chest xz plane
    double ang = Vec3d_diffAngle(Vec3d_ex(), tmp) - M_PI_2;
    bool ok = RcsGraph_setJoint(controller.getGraph(), "arm_flex_r", ang);
    RCHECK(ok);
  }

  RcsGraph_setState(controller.getGraph(), NULL, NULL);
  RcsGraph_setState(visGraph, controller.getGraph()->q, NULL);
}

void RetargetComponent::Pose::retarget_IK(const std::vector<HTr>& frames)
{
  for (size_t i = 0; i < controller.getNumberOfTasks(); ++i)
  {
    if (a_des->ele[i] == 0.0)
    {
      continue;
    }

    auto it = nameIdMap.find(controller.getTaskName(i));
    if (it != nameIdMap.end())
    {
      int frameId = it->second;
      Vec3d_copy(&x_des->ele[controller.getTaskArrayIndex(i)], frames[frameId].org);
    }
  }

  const double lambda = 1.0, alpha = 0.05;
  MatNd* dx_des = MatNd_create(controller.getTaskDim(), 1);
  MatNd* dH = MatNd_create(1, controller.getGraph()->nJ);
  MatNd* dq_des = MatNd_create(controller.getGraph()->dof, 1);

  for (size_t i = 0; i < numIkIterationsPerStep; ++i)
  {
    controller.computeDX(dx_des, x_des, a_des);
    controller.computeJointlimitGradient(dH);
    MatNd_constMulSelf(dH, alpha);
    ikSolver.solveLeftInverse(dq_des, dx_des, dH, a_des, lambda);
    MatNd_addSelf(controller.getGraph()->q, dq_des);
    RcsGraph_setState(controller.getGraph(), NULL, NULL);
  }

  MatNd_destroy(dx_des);
  MatNd_destroy(dH);
  MatNd_destroy(dq_des);
}

void RetargetComponent::Pose::setCameraTransform(const HTr* A_CI_)
{
  HTr_copy(&A_CI, A_CI_);

  std::string bdyName = "Camera";
  RcsBody* cam = RcsGraph_getBodyByName(controller.getGraph(), bdyName.c_str());

  if (!cam)
  {
    RLOG_CPP(1, "Could not find camera body '" << bdyName << "'");
  }
  else
  {
    HTr_copy(&cam->A_BP, A_CI_);
  }

  cam = RcsGraph_getBodyByName(visGraph, bdyName.c_str());

  if (!cam)
  {
    RLOG_CPP(1, "Could not find camera body '" << bdyName << "'");
  }
  else
  {
    HTr_copy(&cam->A_BP, A_CI_);
  }
}

bool RetargetComponent::Pose::setCameraTransform(const std::string& bdyName)
{
  const RcsBody* cam = RcsGraph_getBodyByName(controller.getGraph(), bdyName.c_str());

  if (!cam)
  {
    RLOG_CPP(1, "Could not find camera body '" << bdyName << "'");
    return false;
  }

  setCameraTransform(&cam->A_BI);

  return true;
}

void RetargetComponent::Pose::filterVisualGraph(double tmc)
{
  for (unsigned int i = 0; i < visGraph->dof; ++i)
  {
    visGraph->q->ele[i] = tmc * controller.getGraph()->q->ele[i] +
                          (1.0 - tmc) * visGraph->q->ele[i];
  }

  RcsGraph_setState(visGraph, NULL, NULL);
}

































RetargetComponent::RetargetComponent(EntityBase* parent, BodyType bType, size_t maxPeople) :
  ComponentBase(parent), threadedRetarget(false), dt_retarget(0.0)
{
  subscribe("RetargetPose", &RetargetComponent::onRetarget);
  subscribe("SetAzureKinectTransform", &RetargetComponent::onSetCameraTransform);
  HTr_setIdentity(&A_CI);
  for (int i=0; i<maxPeople; ++i)
  {
    poses.push_back(new Pose(getEntity(), bType, i));
  }
}

RetargetComponent::~RetargetComponent()
{
}

void RetargetComponent::onSetCameraTransform(HTr A_CI_)
{
  HTr_copy(&A_CI, &A_CI_);
  for (auto& pose : poses)
  {
    pose->setCameraTransform(&A_CI_);
  }
}

bool RetargetComponent::setCameraTransform(const std::string& bdyName)
{
  bool success = true;

  for (auto& pose : poses)
  {
    success &= pose->setCameraTransform(bdyName);
    // \todo: What happens if no poses are in the map?
    HTr_copy(&A_CI, &pose->A_CI);
  }

  return success;
}

/*
    This function creates a correspondence map in the form:

    poseIndex   ->   bodyId
        0               5
        1               1
        2               3
        3              -1
        4              -1

 */
std::vector<int> RetargetComponent::findCorrespondences(std::map<int, std::vector<HTr>> markerMap) const
{
  std::vector<int> res(poses.size(), -1);   // Vector with number of poses entries, all being -1

  // No pose yet assigned - we just go through the incoming markerMap in their
  // order and assign their body id. For poses without any matching update, we
  // set the index to invalid (-1).
  if (poses[0]->lastUpdate == 0.0)
  {
    auto it = markerMap.begin();

    for (size_t i=0; i<poses.size(); ++i)
    {
      if (it == markerMap.end())
      {
        break;
      }
      res[i] = it->first;
      it++;
    }
  }
  /*
      Find correspondences based on closest distance to pelvis

                Pose      0     1     2     3
      bdy-id  frameIdx
        5         0      d00   d01   d02   d02

        1         1      d10   d11   d12   d12

        7         2      d20   d21   d22   d22
   */
  else
  {
    // Create pair-wise distance matrix
    MatNd* dMat = MatNd_create(markerMap.size(), poses.size());
    std::vector<int> frameIdVec(markerMap.size());

    unsigned int frameIdx = 0;
    for (auto it = markerMap.begin(); it != markerMap.end(); it++)
    {
      frameIdVec[frameIdx] = it->first;
      HTr pelv = it->second[0];

      for (size_t poseIdx = 0; poseIdx < poses.size(); ++poseIdx)
      {
        double dPelv = Vec3d_distance(pelv.org, poses[poseIdx]->rawFrames[0].org);
        MatNd_set(dMat, frameIdx, poseIdx, dPelv);
      }

      frameIdx++;
    }

    // Now we go through each frame and determine the closest pose. We then
    // create the match by calling retarget, and then delete the corresponding
    // row and column. It's a bit heuristic. There might be more accurate (global)
    // algorithms out there, maybe the "Closest Pair of Points" is a good starting
    // point.
    // MatNd_printCommentDigits("dMat", dMat, 6);

    // Go row by row
    for (frameIdx = 0; frameIdx < dMat->m; ++frameIdx)
    {
      double dMin = MatNd_get(dMat, frameIdx, 0);
      int minPose = 0;
      for (unsigned int poseIdx = 1; poseIdx < dMat->n; ++poseIdx)
      {
        double dMin_i = MatNd_get(dMat, frameIdx, poseIdx);
        if (dMin_i < dMin)
        {
          dMin = dMin_i;
          minPose = poseIdx;
        }
      }

      // RLOG(0, "Frame %d is closest to Pose %d with distance %f", frameIdx, minPose, dMin);
      res[minPose] = frameIdVec[frameIdx];

      // Ensure that same pose is not considered again
      MatNd_setColumnToValue(dMat, minPose, DBL_MAX);
    }

    MatNd_destroy(dMat);
  }



  return res;
}



static void ikFunc(RetargetComponent::Pose* ret, std::vector<HTr> frames)
{
  ret->retarget(frames);
}

void RetargetComponent::onRetarget(std::map<int, std::vector<HTr>> markerMap)
{
  double dt_calc = Timer_getSystemTime();


  std::vector<int> corrMap = RetargetComponent::findCorrespondences(markerMap);

  // The threadMtx makes the thread launch and join atomic against changing the
  // threadedRetarget flag. If we don't do this, we might join threads that have
  // not been launched which results in crashes.
  threadMtx.lock();
  std::vector<std::thread> ikThread(corrMap.size());

  for (size_t i=0; i<corrMap.size(); ++i)
  {
    if (corrMap[i] != -1)
    {
      auto it = markerMap.find(corrMap[i]);
      RCHECK(it!=markerMap.end());

      if (!threadedRetarget)
      {
        ikFunc(poses[i], it->second);
      }
      else
      {
        ikThread[i] = std::move(std::thread(ikFunc, poses[i], it->second));
      }

    }

  }

  if (threadedRetarget)
  {
    for (size_t i=0; i<corrMap.size(); ++i)
    {
      if (corrMap[i] != -1)
      {
        ikThread[i].join();
      }
    }
  }
  threadMtx.unlock();

  dt_calc = Timer_getSystemTime() - dt_calc;

  if (dt_retarget == 0.0)
  {
    dt_retarget = dt_calc;
  }
  else
  {
    dt_retarget = 0.05 * dt_calc + 0.95 * dt_retarget;
  }

  std::stringstream hudStr;
  hudStr << (threadedRetarget ? "Threaded" : "Non-threaded") << " retarget took "
         << std::setprecision(3) << 1.0e3*dt_retarget <<  " msec" << std::endl;
  for (size_t i=0; i<corrMap.size(); ++i)
  {
    hudStr << "pose[" << i << "] = frame " << std::setw(2) << corrMap[i];
    hudStr << " valid: " << (poses[i]->isValid() ? "yes" : "no ");
    hudStr << " initializing: " << (poses[i]->countDown>0.0 ? "yes" : "no ");
    hudStr << " height: " << poses[i]->bodyHeight;
    hudStr << std::endl;
  }
  getEntity()->publish("SetTextLine", hudStr.str(), 2);
}

void RetargetComponent::toggleThreading()
{
  threadMtx.lock();
  threadedRetarget = !threadedRetarget;
  threadMtx.unlock();
}


















RetargetLogger::RetargetLogger(EntityBase* parent, const std::string& fileName_) :
  ComponentBase(parent), fd(nullptr), lastSampleTime(-1), fileName(fileName_)
{
  subscribe("RetargetPose", &RetargetLogger::onRetarget);
}

RetargetLogger::~RetargetLogger()
{
  stopRecording();
}

bool RetargetLogger::isRecording() const
{
  return fd ? true : false;
}

void RetargetLogger::onRetarget(std::map<int, std::vector<HTr>> poseMap)
{
  if (!isRecording())
  {
    return;
  }

  if (lastSampleTime == getEntity()->getTime())
  {
    RLOG(0, "Azure thread faster than main thread - skipping sample at t=%f", lastSampleTime);
    return;
  }

  for (auto it = poseMap.begin(); it != poseMap.end(); it++)
  {
    int poseId = it->first;
    const std::vector<HTr>& frames = it->second;

    for (size_t i=0; i<frames.size(); ++i)
    {
      const double* ptr = (const double*)frames[i].org;

      fprintf(fd, "%f %d %f %f %f %f %f %f %f %f %f %f %f %f\n",
              getEntity()->getTime(), poseId,
              ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5],
              ptr[6], ptr[7], ptr[8], ptr[9], ptr[10], ptr[11]);
    }
  }

  lastSampleTime = getEntity()->getTime();
}

void RetargetLogger::startRecording()
{
  if (!fd)
  {
    fd = fopen(fileName.c_str(), "w+");
    if (!fd)
    {
      RLOG_CPP(1, "Failed to open " << fileName << " for logging");
    }
  }
}

void RetargetLogger::stopRecording()
{
  if (fd)
  {
    fclose(fd);
    fd = NULL;
  }
  else
  {
    RLOG_CPP(1, "Recording already stopped");
  }
}










RetargetPlayer::RetargetPlayer(EntityBase* parent, const std::string& fileName) :
  ComponentBase(parent), data(NULL), play(false), rowIdx(0), loopCount(0), everyNth(1)
{
  RLOG(0, "Read data from %s", fileName.c_str());
  data = MatNd_createFromFile(fileName.c_str());
  RCHECK_MSG(data, "Couldn't read data from file %s", fileName.c_str());
  RCHECK_MSG(data->n == 14, "Data ill-formed: %d columns, but must be 13", data->n);
  RCHECK_MSG(data->m%NUM_MARKERS==0, "%d", data->m % NUM_MARKERS);
  RLOG(0, "Read data with %d transforms", data->m / NUM_MARKERS);
  subscribe("Render", &RetargetPlayer::onRender);
  everyNth = lround(getDt()/getEntity()->getDt());
}

RetargetPlayer::~RetargetPlayer()
{
  MatNd_destroy(data);
}

void RetargetPlayer::startPlaying()
{

}

void RetargetPlayer::stopPlaying()
{

}

void RetargetPlayer::onRender()
{
  if ((loopCount++) % everyNth != 0)
  {
    return;
  }
  std::map<int, std::vector<HTr>> poseMap;
  const size_t rowIdx0 = rowIdx;
  const double t_data0 = MatNd_get(data, rowIdx, 0);

  do
  {
    const double* row = MatNd_getRowPtr(data, rowIdx);
    const double t_data = row[0];

    if (t_data != t_data0)
    {
      break;
    }

    int bodyId = lround(row[1]);
    HTr frm;
    Vec3d_copy(frm.org, &row[2]);
    Mat3d_fromArray(frm.rot, &row[5]);
    poseMap[bodyId].push_back(frm);
    rowIdx++;
  }
  while (rowIdx < data->m);

  RCHECK_MSG((rowIdx-rowIdx0)% NUM_MARKERS==0, "%zu %zu %zu",
             rowIdx, rowIdx0, rowIdx-rowIdx0);

  for (auto it = poseMap.begin(); it != poseMap.end(); it++)
  {
    int poseId = it->first;
    const std::vector<HTr>& frames = it->second;
    // RLOG_CPP(0, "Time " << t_data0 << ": Frame " << poseId <<"  with "
    //          << frames.size() << " frames");

    if (frames.size() != NUM_MARKERS)
    {
      RLOG_CPP(0, "Mismatch in number of markers: found " << frames.size()
               << ", expected " << NUM_MARKERS << " - rewinding");
      rowIdx = 0;
      return;
    }
  }

  getEntity()->publish("RetargetPose", poseMap);

  char txt[256];
  snprintf(txt, 256, "Playing time %.1f from %.1f", t_data0, MatNd_get(data, data->m-1, 0));
  getEntity()->publish("SetTextLine", std::string(txt), 3);

  // Loop over the data array. If we reach the end, we jump to the beginning. This
  // leads to large jumps in the frames, therefore we re-initialize the retarget
  // calculation.
  if (rowIdx > data->m-1)
  {
    rowIdx = 0;
    getEntity()->publish("RetargetInitialize", 1.0);
  }
}

double RetargetPlayer::getDt() const
{
  RCHECK(data && data->m > 0);
  double dt = 0.0;

  const double t0 = MatNd_get(data, 0, 0);
  for (unsigned int i = 0; i < data->m; ++i)
  {
    double t1 = MatNd_get(data, i, 0);

    if (t1 != t0)
    {
      dt = t1 - t0;
      break;
    }
  }

  RCHECK_MSG(dt > 0.0, "%f", dt);

  return dt;
}

}  // namespace aff
