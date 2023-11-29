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

#ifndef AFF_JACOSHMCOMPONENT_H
#define AFF_JACOSHMCOMPONENT_H

#include "ComponentBase.h"

#include <Rcs_graph.h>



namespace aff
{

class JacoShm;

class JacoShmComponent
{
public:

  enum JacoType
  {
    Jaco6 = 0,
    Jaco7_left,
    Jaco7_right
  };

  JacoShmComponent(const RcsGraph* graph, JacoType roboType);
  virtual ~JacoShmComponent();
  void updateSensors(RcsGraph* graph);
  void setCommand(const MatNd* q);

protected:

  void enableCommands();

private:

  struct JointData
  {
    JointData() : q_curr(0.0), q0(0.0), qd_curr(0.0), jointIdx(-1), arrayIdx(-1)
    {
    }

    JointData(int arrayIdx_) : q_curr(0.0), qd_curr(0.0),
      jointIdx(-1), arrayIdx(arrayIdx_), limitlessJoint(false)
    {
    }

    void setCurrent(double qNew, double qdNew)
    {
      q_curr = qNew;
      qd_curr = qdNew;
    }

    double getCurrentJointAngle() const
    {
      return q_curr;
    }

    double getCurrentJointVelocity() const
    {
      return qd_curr;
    }

    double q_curr;
    double q0;
    double qd_curr;
    int jointIdx;
    int arrayIdx;
    bool limitlessJoint;
  };

  JacoShmComponent& operator = (const JacoShmComponent&);
  JacoShmComponent(const JacoShmComponent& other);

  std::map<std::string,JointData> jntMap;
  std::vector<std::string> jntNames;
  std::vector<double> jntOffset;
  JacoType jType;
  JacoShm* shm;
};



/*******************************************************************************
 *
 ******************************************************************************/
class RoboJacoShmComponent : public ComponentBase, public JacoShmComponent
{
public:

  static RoboJacoShmComponent* create(EntityBase* parent,
                                      const RcsGraph* graph,
                                      JacoType jt);
  RoboJacoShmComponent(EntityBase* parent,
                       const RcsGraph* graph,
                       JacoType jt);

private:

  void onInitFromState(const RcsGraph* target);
  void onEmergencyStop();
  void onEmergencyRecover();
  void onSetJointPosition(const MatNd* q_des);
  void onUpdateGraph(RcsGraph* graph);
  void onEnableCommands();

  bool eStop;
  bool enableCommands;
};






}

#endif   // AFF_JACOSHMCOMPONENT_H
