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
#if 0
#include "ActionPush.h"
#include "ActionFactory.h"
#include "CollisionModelConstraint.h"

#include <ActivationSet.h>
#include <PositionConstraint.h>
#include <PolarConstraint.h>
#include <EulerConstraint.h>
#include <ConnectBodyConstraint.h>
#include <VectorConstraint.h>

#include <Rcs_typedef.h>
#include <Rcs_body.h>
#include <Rcs_macros.h>
#include <Rcs_utilsCPP.h>

#include <limits>


#define fingersOpen   (0.01)
#define fingersClosed (0.7)
#define t_fingerMove  (1.5)

// used for the selection of the feasible affordance
#define maxAngleOfCone (45)



namespace aff
{
REGISTER_ACTION(ActionPush, "push");

ActionPush::ActionPush(const ActionScene& domain,
                     const RcsGraph* graph,
                     std::vector<std::string> params)
{

  for (const auto& wordpair : params)
  {
    std::vector<std::string> keyValue = Rcs::String_split(wordpair, "=");
    RCHECK(keyValue.size()==2);
    if (keyValue[0]=="objectAffordance")
    {
      objectAffordance = keyValue[1].c_str();
    }
    if (keyValue[0]=="manipulator")
    {
      manipulator = keyValue[1].c_str();
    }
    else if (keyValue[0]=="pushPose")
    {
      std::vector<std::string> surfCoords = Rcs::String_split(keyValue[1].c_str(), ",");
      RCHECK(surfCoords.size()==3);
      push2x = atof(surfCoords[0].c_str());
      push2y = atof(surfCoords[1].c_str());
      push2theta = atof(surfCoords[2].c_str());
    }
    else if (keyValue[0]=="prePushDist")
    {
      prePushDist = atof(keyValue[1].c_str());
    }
  }

  RLOG_CPP(0, "Calling push with manipulator='" << manipulator
           << "' object='" << objectAffordance << "' surface='"
           << surfaceAffordance << "'") ;

  std::string errorMsg;

  Vec3d_setZero(obj_euler_cur);


  // Initialize object to get. It must have a grasp feature
  const AffordanceEntity* object = domain.getAffordanceEntity(objectAffordance);

  if (!object)
  {
    throw std::invalid_argument("ActionPush: object '" + objectAffordance + "' not found");
  }

  auto graspables = getAffordances<Graspable>(object);

  if (graspables.empty())
  {
    throw std::invalid_argument("ActionPush: object '" + objectAffordance +
                                "' not found or has no grasp features");
  }

  objectName = object->bdyName;

  const RcsBody* objBdy = resolveBodyName(graph, objectName);

  if (!objBdy)
  {
    errorMsg = "Couldn't resolve object body '" + objectName +
               "'. Is it part of the graph?";
    throw std::invalid_argument(errorMsg);
  }


  auto bottoms = getAffordances<Stackable>(object);

  if (bottoms.empty())
  {
    throw ActionException("ActionPut: object " + objectAffordance + " has no bottom (Stackable)",
                          ActionException::ParamNotFound);
  }

  RCHECK_MSG(bottoms.size()==1, "Not yet handled");
  objectBottomName = bottoms[0]->frame;
  const RcsBody* objBottomBdy = resolveBodyName(graph, objectBottomName);

  if (!objBottomBdy)
  {
    throw std::invalid_argument("Could not resolve object bottom " + objectBottomName);
  }

  // Initialize the manipulator to get the object with. We don't enforce one.
  // If it is not passed as an argument, the closest one to the object will be
  // selected.
  const Manipulator* hand = NULL;
  if (!manipulator.empty())
  {
    hand = domain.getManipulator(manipulator);
  }

  // If a manipulator has explicitly been passed but cannot be found, we
  // consider this as an error.
  if ((!manipulator.empty()) && (!hand))
  {
    throw std::invalid_argument("ActionPush: Passed manipulator '" +
                                manipulator + "' not found in graph");
  }

  // If no hand is specified, we search for the closest one to the object. We
  // already have the objBody as a valid pointer.
  if (!hand)
  {
    double dMin = std::numeric_limits<double>::max();
    for (auto& m : domain.getFreeManipulators(graph))   // Disallow using occupied manipulator.
    {
      // We could loop over all frames here. Since they are usually positioned
      // rather close to each other, we only consider the first one.
      auto m_cap = getCapabilities<GraspCapability>(m);
      if (m_cap.empty())
        //if (m->grasps.empty())
      {
        continue;
      }

      const RcsBody* bi = RcsGraph_getBodyByNameNoCase(graph, m_cap[0]->frame.c_str());

      // There are grasps without a frame, so we only check for these
      if (!bi)
      {
        continue;
      }

      // Compute the distance between manipulator and object and keep track of the closest manipulator.
      double d = Vec3d_distance(bi->A_BI.org, objBdy->A_BI.org);
      if (d < dMin)
      {
        dMin = d;
        hand = m;
      }
    }
  }

  if (!hand)
  {
    throw ActionException("ActionPush: Could not find manipulator", ActionException::ParamNotFound);
  }

  RLOG_CPP(1, "Closest hand: " << hand->name);

  usedManipulators.push_back(hand->name);


  // Initialize the surface to put the object on. We don't enforce to have one.
  // If not, it will be later searched by ray casting.
  const AffordanceEntity* surface = domain.getAffordanceEntity(surfaceAffordance);

  // If a surface has explicitely been passed but cannot be found, we
  // consider this as an error.
  if ((!surfaceAffordance.empty()) && (!surface))
  {
    throw std::invalid_argument("ActionPush: Passed surface '" +
                                surfaceAffordance + "' not found in graph");
  }

  if (surface)// && !surface->supportSurfaces.empty())
  {
    auto supportSurfaces = getAffordances<Supportable>(surface);
    RCHECK(supportSurfaces.size()==1, "More than 1 support surface to be implemented");

    surfaceName = supportSurfaces[0]->frame;
  }

  // Determine the surface the object is to be slided on
  const RcsBody* surfaceBdy = resolveBodyName(graph, surfaceName);

  // First we check if the object has any parent. If yes - this is what we
  // consider as the surface body. Comment: We don't do this any more since
  // for marker tracking, our parent might be the camera.
  if (objBdy->parentId != -1)
  {
    surfaceBdy = &graph->bodies[objBdy->parentId];
    surfaceName = std::string(surfaceBdy->name);
  }

  // In case there is no parent - we do a geometric raycast downwards and
  // see what's below.
  if (!surfaceBdy)
  {
    double dir[3], dMin = 0.0;
    Vec3d_set(dir, 0.0, 0.0, -1.0);
    // We raycast down from a tiny bit below the bottom so that we don't
    // intersect with the object itself.
    double bottomPt[3];
    Vec3d_copy(bottomPt, objBottomBdy->A_BI.org);
    bottomPt[2] -= 1.0e-8;
    surfaceBdy = RcsBody_closestInDirection(graph, bottomPt, dir,
                                            NULL, &dMin);
    // \todo: Check for self intersections.
    if (surfaceBdy)
    {
      if (dMin <  1.0e-2 )  // Otherwise we push it into the air
      {
        surfaceBdy = NULL;
        errorMsg = "Distance to surface too large: " + std::to_string(dMin);
        throw std::invalid_argument(errorMsg);
      }
      else
      {
        this->surfaceName = std::string(surfaceBdy->name);
        RLOG(0, "Closest surface of %s is %s with d=%f",
             objBdy->name, surfaceBdy->name, dMin);
      }
    }
    else
    {
      errorMsg = "Surface detection below object failed - no surface found";
      throw std::invalid_argument(errorMsg);
    }
  }

  // get the current orientation of the object in relative to the parent (surface) coordinates
  HTr_invTransform(&Obj2Sur, &surfaceBdy->A_BI, &objBdy->A_BI);
  Mat3d_toEulerAngles(obj_euler_cur, Obj2Sur.rot);

  // From here on, we have a valid manipulator and a valid object model
  // We can now determine the most feasible grasp feature.
  // We need to determine these variables:
  // handBdy, handName, graspOrientation, objGraspFrmName
  // We go through all graspFrame - grasp feature combinations and select the
  // combination that has the least angular (SLERP) difference.
  const RcsBody* handBdy = NULL;
  //auto gm = hand->getFeasibleGraspFeatures(object);
  auto gm = match<Graspable>(object, hand);
  RLOG_CPP(0, "Found " << gm.size() << " grasp combinations");
  double angMin = std::numeric_limits<double>::max();

  for (auto& gg : gm)
  {
    // consider only push affordances
    if (gg.second.type=="Push")
    {
      const char* graspFrmName = gg.first.frame.c_str();
      const char* objFrmName = gg.second.bdyName.c_str();
      RcsBody* graspFrm = RcsGraph_getBodyByName(graph, graspFrmName);
      RcsBody* objFrm = RcsGraph_getBodyByName(graph, objFrmName);

      // check if the relative displacement is within the cone of the GraspFeature
      // this is defined w.r.t the x axis of the GraspFeature frame
      double ggXaxis[3];
      Vec3d_set(ggXaxis, objFrm->A_BI.rot[0][0], objFrm->A_BI.rot[0][1], objFrm->A_BI.rot[0][2]);

      // create the goal displacement vector
      double delta2DPush[3];
      Vec3d_set(delta2DPush, push2x, push2y, 0.0);

      // angle is always between 0 and 90 degrees
      double angle2Xaxis = Vec3d_diffAngle(ggXaxis, delta2DPush);
      RMSG("Angle %f ", RCS_RAD2DEG(angle2Xaxis));
      if (RCS_RAD2DEG(angle2Xaxis) < maxAngleOfCone)
      {

        // RCHECK_MSG(graspFrm, "Not found in graph: %s", graspFrmName);
        // RCHECK_MSG(objFrm, "Not found in graph: %s", objFrmName);
        const double ang = Mat3d_diffAngle(graspFrm->A_BI.rot, objFrm->A_BI.rot);
        RLOG(0, "Checking %s - %s: %f", graspFrmName, objFrmName, RCS_RAD2DEG(ang));
        if (ang <= angMin)
        {
          angMin = ang;
          handName = gg.first.frame;
          handBdy = graspFrm;
          objGraspFrmName = gg.second.bdyName;
          graspOrientation = gg.second.orientation;
          RLOG(0, "Grasp frame is \"%s\"", objGraspFrmName.c_str());
          RLOG(0, "Grasp orientation is \"%s\"", graspOrientation.c_str());
        }
      }
    }
  }
  RLOG(0, "Selected %s - %s", handName.c_str(), objGraspFrmName.c_str());

  RCHECK(handBdy);


  // Assemble finger task name
  for (auto& f : hand->fingerJoints)
  {
    fingerJoints += f;
    fingerJoints += " ";
  }

  // Task naming
  std::string utterance = object->name;
  this->taskObjHandPos = utterance + "-" + handName + "-XYZ";
  this->taskObjSurfacePosX = utterance + "-" + surfaceName + "-X";
  this->taskObjSurfacePosY = utterance + "-" + surfaceName + "-Y";
  this->taskObjSurfacePosZ = utterance + "-" + surfaceName + "-Z";
  this->taskObjOri = utterance + "-EULER";
  this->taskHandObjOri = handName + "-" + utterance + "-EULER";
  this->taskFingers = handName + "_fingers";
}

ActionPush::~ActionPush()
{
}

tropic::TCS_sptr ActionPush::createTrajectory(double t_start, double t_end) const
{
  const double t_cnt = t_start + 0.5*(t_end - t_start);
  auto a1 = createTrajectory(t_start, t_cnt, t_end);

  return a1;
}

std::shared_ptr<tropic::ConstraintSet>
ActionPush::createTrajectory(double t_start,
                            double t_cnt,
                            double t_end) const
{
  const double afterTime = 0.5;//2.0;

  // Push the object
  auto a1 = std::make_shared<tropic::ActivationSet>();

  // Hand position with respect to object
  const double t_precnt = t_start + 0.75*(t_cnt-t_start);
  const double t_postcnt = t_cnt + 0.5*(t_end-t_cnt);
  a1->addActivation(t_start, true, 0.5, taskObjHandPos);

  // RMSG(" times %f,  %f,  %f,  %f,  %f ", t_start, t_precnt, t_cnt, t_postcnt, t_end);

  // Deactivate object collisions when in pregrasp
  a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_start, objectName, false));

  a1->add(t_precnt, prePushDist, 0.0, 0.0, 1, taskObjHandPos + " 0");// some distance in front
  a1->add(t_precnt, 0.0, 0.0, 0.0, 1, taskObjHandPos + " 1");// and centered w.r.t y
  // a1->add(t_precnt, 0.0, 0.0, 0.0, 5, taskObjHandPos + " 2");// and centered w.r.t z

  // Hand orientation with respect to object
  a1->addActivation(t_start, true, 0.5, taskHandObjOri);
  if (graspOrientation=="Euler")
  {
    a1->add(std::make_shared<tropic::EulerConstraint>(t_precnt, 0.0, 0.0, 0.0, taskHandObjOri));
  }
  else
  {
    RFATAL("Unknown grasp orientation: \"%s\"", graspOrientation.c_str());
  }

  // a1->add(t_cnt, 0.0, 0.0, 0.0, 7, taskObjHandPos + " 0");// some distance in front
  // a1->add(t_cnt, 0.0, 0.0, 0.0, 7, taskObjHandPos + " 1");// and centered w.r.t y
  // a1->add(t_cnt, 0.0, 0.0, 0.0, 7, taskObjHandPos + " 2");// and centered w.r.t z
  // add contact constraint
  a1->add(std::make_shared<tropic::PositionConstraint>(t_cnt, 0.0, 0.0, 0.0, taskObjHandPos));


  // attach object to a free body (meaning detach it from the table)
  a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_cnt, objectName, handName));

  // Object orientation with respect to world frame.
  a1->addActivation(t_cnt, true, 0.5, taskObjOri);
  a1->addActivation(t_postcnt+ afterTime, false, 0.5, taskObjOri);
  a1->add(std::make_shared<tropic::EulerConstraint>(t_postcnt, obj_euler_cur[0], obj_euler_cur[1], obj_euler_cur[2] + RCS_DEG2RAD(push2theta), taskObjOri));

  // Push object
  a1->addActivation(t_cnt, true, 0.5, taskObjSurfacePosX);
  a1->addActivation(t_postcnt+ afterTime, false, 0.5, taskObjSurfacePosX);
  a1->add(t_postcnt, Obj2Sur.org[0] + push2x, 0.0, 0.0, 7, taskObjSurfacePosX + " 0");

 // Reduce snap to null space y-position by pushing forward straight
  a1->addActivation(t_cnt, true, 0.5, taskObjSurfacePosY);
  a1->addActivation(t_postcnt+ afterTime, false, 0.5, taskObjSurfacePosY);
  a1->add(t_postcnt, Obj2Sur.org[1] + push2y, 0.0, 0.0, 7, taskObjSurfacePosY + " 0");

  // Push it in the x-y plane (keep height constant)
  a1->addActivation(t_cnt, true, 0.5, taskObjSurfacePosZ);
  a1->addActivation(t_postcnt+afterTime, false, 0.5, taskObjSurfacePosZ);

  // attach object to the table(surface) (meaning detach it from the hand --> free the hand)
  a1->add(std::make_shared<tropic::ConnectBodyConstraint>(t_postcnt, objectName, surfaceName));

  // add again the contact constraint cause the graph has been modified just before
  a1->add(std::make_shared<tropic::PositionConstraint>(t_postcnt, 0.0, 0.0, 0.0, taskObjHandPos));

  // Retract hand from object
  a1->add(t_end, prePushDist, 0.0, 0.0, 7, taskObjHandPos + " 0");// some distance in front
  a1->add(t_end, 0.0, 0.0, 0.0, 7, taskObjHandPos + " 1");// and centered w.r.t y
  a1->add(t_end, -prePushDist, 0.0, 0.0, 7, taskObjHandPos + " 2");// and centered w.r.t z

  // Detach hand from object
  a1->addActivation(t_end+afterTime, false, 0.5, taskObjHandPos);
  a1->addActivation(t_end+afterTime, false, 0.5, taskHandObjOri);


  // Activate object collisions after release
  a1->add(std::make_shared<tropic::CollisionModelConstraint>(t_end + 4*afterTime, objectName, true));

  return a1;
}

void ActionPush::print() const
{
  std::cout << "handName: " << handName << std::endl;
  std::cout << "objectName: " << objectName << std::endl;
  std::cout << "objectGraspName: " << objGraspFrmName << std::endl;
  std::cout << "surfaceName: " << surfaceName << std::endl;
  std::cout << "fingerJoints: " << fingerJoints << std::endl;
  std::cout << "graspOrientation: " << graspOrientation << std::endl;
}

std::vector<std::string> ActionPush::createTasksXML() const
{
  std::vector<std::string> tasks;
  std::string xmlTask;

  // taskObjHandPos: XYZ-task with effector=object and refBdy=hand
  xmlTask = "<Task name=\"" + taskObjHandPos + "\" " +
            "controlVariable=\"XYZ\" " + "effector=\"" + objGraspFrmName +
            "\" " + "refBdy=\"" + handName + "\" />";
  tasks.push_back(xmlTask);

  // taskObjSurfacePosition z and sideways velocities
  xmlTask = "<Task name=\"" + taskObjSurfacePosX + "\" " +
            "controlVariable=\"X\" " + "effector=\"" + objectBottomName + "\" " +
            "refBdy=\"" + surfaceName + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskObjSurfacePosY + "\" " +
            "controlVariable=\"Y\" " + "effector=\"" + objectBottomName + "\" " +
            "refBdy=\"" + surfaceName + "\" />";
  tasks.push_back(xmlTask);

  xmlTask = "<Task name=\"" + taskObjSurfacePosZ + "\" " +
            "controlVariable=\"Z\" " + "effector=\"" + objectBottomName + "\" " +
            "refBdy=\"" + surfaceName + "\" />";
  tasks.push_back(xmlTask);

  // taskObjPolar
  if (graspOrientation == "Euler")
  {
    // xmlTask = "<Task name=\"" + taskObjOri + "\" " +
    //           "controlVariable=\"ABC\" " + "effector=\"" +
    //           objGraspFrmName + "\" />";
    xmlTask = "<Task name=\"" + taskObjOri + "\" " +
          "controlVariable=\"ABC\" " + "effector=\"" +
          objectName + "\" " +
          "refBdy=\"" + surfaceName + "\" />";
  }
  else
  {
    RLOG(1, "Unknown grasp orientation: %s", graspOrientation.c_str());
    return std::vector<std::string>();
  }

  tasks.push_back(xmlTask);

  // taskHandObjOri
  if (graspOrientation == "Euler")
  {
    xmlTask = "<Task name=\"" + taskHandObjOri + "\" " +
              "controlVariable=\"ABC\" " + "effector=\"" + handName + "\" " +
              "refBdy=\"" + objGraspFrmName + "\" />";
  }
  tasks.push_back(xmlTask);

  // Fingers
  xmlTask = "<Task name=\"" + taskFingers + "\" controlVariable=\"Joints\" " +
            "jnts=\"" + fingerJoints + "\" />";
  tasks.push_back(xmlTask);

  return tasks;
}

double ActionPush::getDurationHint() const
{
  return 13.5;
}


std::string ActionPush::explain() const
{
  return std::string("I'm pushing the " + objectName);
}

std::vector<std::string> ActionPush::getManipulators() const
{
  return usedManipulators;
}

std::unique_ptr<ActionBase> ActionPush::clone() const {
  return std::make_unique<ActionPush>(*this);
}

}   // namespace aff
#endif//0
