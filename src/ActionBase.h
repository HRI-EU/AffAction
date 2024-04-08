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

#ifndef AFF_ACTIONBASE_H
#define AFF_ACTIONBASE_H

#include "ActionScene.h"
#include "TrajectoryPredictor.h"

#include <ConstraintSet.h>
#include <ControllerBase.h>

#include <exception>



/*! \brief Base class for affordance-based actions.
 *
 */

namespace aff
{

class ActionException: public std::exception
{
public:

  enum ActionError
  {
    ParamNotFound,            //!< Parameter name could not be retrieved
    ParamInvalid,             //!< Parameter is invalid
    KinematicallyImpossible,  //!< Out of reached, hand occupied ...
    NoError,
    UnknownError
  };

  ActionException(ActionError e,
                  std::string reasonMsg_,
                  std::string suggestionMsg_=std::string(),
                  std::string developerMsg_=std::string()) :
    error(e), errorMsg(err2str(e)), reasonMsg(reasonMsg_), suggestionMsg(suggestionMsg_), developerMsg(developerMsg_)
  {

    // Create legacy message
    msg = (e==NoError) ? "SUCCESS" : "ERROR";
    msg += " REASON: " + reasonMsg + " SUGGESTION: " + suggestionMsg;
    if (!developerMsg.empty())
    {
      msg += " DEVELEOPER: " + developerMsg;
    }
  }

  ActionException(const std::string& message, ActionError e) : msg(message), error(e)
  {
    msg = err2str(e) + message;
  }

  virtual ~ActionException() noexcept
  {
  }

  virtual const char* what() const noexcept
  {
    return msg.c_str();
  }

  std::string err2str(ActionError e) const
  {
    // \todo(MG): somehow improve developer feeback msg.
    return std::string();
    std::string str;

    switch (e)
    {
      case ParamNotFound:
        str = "ParamNotFound: ";
        break;

      case ParamInvalid:
        str = "ParamInvalid: ";
        break;

      case KinematicallyImpossible:
        str = "KinematicallyImpossible: ";
        break;

      case NoError:
        str = "NoError: ";
        break;

      default:
        str = "UnknownError: ";
        break;
    }

    return str;
  }

protected:

  std::string msg;
  std::string errorMsg;
  std::string reasonMsg;
  std::string suggestionMsg;
  std::string developerMsg;

  ActionError error;
};



class ActionBase
{
public:

  ActionBase();
  virtual ~ActionBase();
  virtual std::unique_ptr<ActionBase> clone() const = 0;
  virtual tropic::TCS_sptr createTrajectory(double t_start, double t_end) const = 0;
  virtual std::vector<std::string> getManipulators() const = 0;
  virtual std::vector<std::string> createTasksXML() const = 0;

  virtual tropic::TCS_sptr createTrajectory() const;
  virtual double getDurationHint() const;
  virtual std::string explain() const;
  void setName(const std::string& name);
  std::string getName() const;
  void setActionParams(const std::vector<std::string>& params);
  virtual std::string getActionCommand() const;
  virtual void print() const;
  virtual double actionCost(const ActionScene& domain,
                            const RcsGraph* graph) const;

  static std::vector<std::string> planActionSequence(ActionScene& domain,
                                                     RcsGraph* graph,
                                                     const RcsBroadPhase* bp,
                                                     std::vector<std::string> actions,
                                                     size_t numStepsToPlan,
                                                     size_t maxNumThreads,
                                                     double dt,
                                                     std::string& errMsg);

  // Interface for prediction
  virtual bool initialize(const ActionScene& domain, const RcsGraph* graph, size_t solutionRank);
  virtual size_t getNumSolutions() const;
  virtual TrajectoryPredictor::PredictionResult predict(ActionScene& scene,
                                                        const RcsGraph* graph,
                                                        const RcsBroadPhase* broadphase,
                                                        double duration,
                                                        double dt,
                                                        bool earlyExit) const;

protected:

  virtual const AffordanceEntity* raycastSurface(const ActionScene& domain,
                                                 const AffordanceEntity* dropEntity,
                                                 const RcsGraph* graph,
                                                 HTr* surfTransform,
                                                 std::string& errMsg) const;


  // Interface for optimization.
  virtual size_t getOptimDim() const;
  virtual std::vector<double> getOptimState() const;
  virtual void setOptimState(std::vector<double> state);
  virtual std::vector<double> getInitOptimState(tropic::TrajectoryControllerBase* tc, double duration) const;

  const RcsBody* resolveBodyName(const RcsGraph* graph, std::string& bdyName);
  double defaultDuration;


private:

  virtual size_t addTasks(Rcs::ControllerBase* controller) const;

  std::string actionName;
  std::vector<std::string> actionParams;
  std::vector<double> optimState;
};

}   // namespace aff


#endif // AFF_ACTIONBASE_H
