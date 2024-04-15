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

#ifndef AFF_ACTIONPUT_H
#define AFF_ACTIONPUT_H


#include "ActionBase.h"



namespace aff
{

class ActionPut : public ActionBase
{
public:

  ActionPut();
  ActionPut(const ActionScene& domain,
            const RcsGraph* graph,
            std::vector<std::string> params);

  virtual ~ActionPut();
  std::unique_ptr<ActionBase> clone() const;

  tropic::TCS_sptr createTrajectory(double t_start, double t_end) const;
  std::vector<std::string> getManipulators() const;
  std::vector<double> getInitOptimState(tropic::TrajectoryControllerBase* tc,
                                        double duration) const;

  bool initialize(const ActionScene& domain, const RcsGraph* graph, size_t solutionRank);
  void print() const;
  size_t getNumSolutions() const;
  double getDurationHint() const;
  double actionCost(const ActionScene& domain,
                    const RcsGraph* graph) const;
  std::string getActionCommand() const;

protected:

  double actionCost(const ActionScene& domain,
                    const RcsGraph* graph,
                    const std::string& place) const;

  void parseArgs(const ActionScene& domain,
                 const RcsGraph* graph,
                 std::vector<std::string>& params);

  const AffordanceEntity* initHands(const ActionScene& domain,
                                    const RcsGraph* graph,
                                    const std::string& objAffordance);

  void initOptions(const ActionScene& domain,
                   const RcsGraph* graph,
                   const AffordanceEntity* object,
                   const std::string& surface);

  std::vector<std::string> createTasksXML() const;

  std::shared_ptr<tropic::ConstraintSet>
  createTrajectory(double t_start,
                   double t_grasp,
                   double t_end) const;

  std::string graspFrame;    // Name of grasp frame, needed for retract motion
  std::string objName;   // For ConnectBodyConstraint only
  std::string objBottomName; // For CollisionModelConstraint
  std::string objGraspFrame; // For retracting only
  std::string surfaceFrameName;   // Rcs body name of surface
  std::string fingerJoints;

  std::string taskObjHandPos;       // XYZ-task with effector=object and refBdy=hand
  std::string taskHandSurfacePos;   // XYZ-task with effector=hand, refBdy=object and refFrame=surface
  std::string taskObjSurfacePosX;
  std::string taskObjSurfacePosY;
  std::string taskObjSurfacePosZ;
  std::string taskObjSurfacePolar;
  std::string taskHandObjPolar;
  std::string taskHandPolar;    // Hand Polar angles in world coordinates (for freezing orienttion when retracting upwards)
  std::string taskSurfaceOri;   // In case the object to put on is in the robot's hand
  std::string taskFingers;

  std::string whereOn;   // Optional keyword specifying the name of the Supportable to put the object on.
  std::string nearTo;    // Keyword specifying the name of an entity or agent near to which the object is to be put.
  std::string farFrom;   // Keyword specifying the name of an entity or agent far of which the object is to be put.
  std::string detailedActionCommand;

  std::vector<std::string> usedManipulators;
  std::vector<double> handOpen;

  bool putDown;
  bool isObjCollidable;
  bool isPincerGrasped;
  double supportRegionX, supportRegionY;
  unsigned int polarAxisIdx;
  double distance;

  // Point in the frame of the surface object on which the object will be put.
  double downProjection[3];

  std::vector<std::tuple<Affordance*, Affordance*>> affordanceMap;
};

}   // namespace aff



#endif // AFF_ACTIONPUT_H
