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

//////////////////////////////////////////////////////////////////////////////
// Simple helper class that blocks the execution in the wait() function
// until the ActionResult event has been received.
//////////////////////////////////////////////////////////////////////////////
class PollBlockerComponent
{
public:

  PollBlockerComponent(aff::ExampleActionsECS* sim_) : sim(sim_)
  {
    sim->setProcessingAction(true);
  }

  void wait()
  {
    while (sim->isProcessingAction())
    {
      Timer_waitDT(0.1);
    }
    RLOG(0, "Done wait");
  }

  aff::ExampleActionsECS* sim;
};


void bind_planning(py::class_<aff::ExampleActionsECS>& cls)
{

  //////////////////////////////////////////////////////////////////////////////
  // Execute the action command, and return immediately.
  //////////////////////////////////////////////////////////////////////////////
  cls.def("execute", [](aff::ExampleActionsECS& ex, std::string actionCommand)
  {
    ex.getEntity().publish("ActionSequence", actionCommand);
  })

  //////////////////////////////////////////////////////////////////////////////
  // Execute the action command, and return only after finished.
  //////////////////////////////////////////////////////////////////////////////
  .def("executeBlocking", [](aff::ExampleActionsECS& ex, std::string actionCommand) -> bool
  {
    PollBlockerComponent blocker(&ex);
    ex.getEntity().publish("ActionSequence", actionCommand);
    blocker.wait();
    RLOG_CPP(0, "Finished: " << actionCommand);
    bool success = ex.lastActionResult[0].success();

    RLOG(0, "   success=%s   result=%s", success ? "true" : "false", ex.lastActionResult[0].error.c_str());
    return success;
  })
  .def("getRobotCapabilities", [](aff::ExampleActionsECS& ex)
  {
    return aff::ActionFactory::printToString();
  })

  //////////////////////////////////////////////////////////////////////////////
  // Predict action sequence as tree
  // Call it like: agent.sim.predictActionSequence("get fanta_bottle;put fanta_bottle lego_box;")
  //////////////////////////////////////////////////////////////////////////////
  .def("predictActionSequence", [](aff::ExampleActionsECS& ex, std::string sequenceCommand) -> std::vector<std::string>
  {
    std::string errMsg;
    std::vector<std::string> seq = Rcs::String_split(sequenceCommand, ";");
    auto tree = ex.getQuery()->planActionTree(aff::PredictionTree::SearchType::DFSMT,
                                              seq, ex.getEntity().getDt());
    return tree ? tree->findSolutionPathAsStrings() : std::vector<std::string>();
  })

  //////////////////////////////////////////////////////////////////////////////
  // Predict action sequence as tree
  //////////////////////////////////////////////////////////////////////////////
  .def("plan_fb", [](aff::ExampleActionsECS& ex, std::string sequenceCommand) -> std::string
  {
    ex.getEntity().publish("FreezePerception", true);
    PollBlockerComponent blocker(&ex);
    ex.getEntity().publish("PlanDFSEE", sequenceCommand);
    blocker.wait();
    ex.getEntity().publish("FreezePerception", false);

    if (ex.lastActionResult[0].success())
    {
      RLOG_CPP(0, "SUCCESS");
      return "SUCCESS";
    }

    std::string fbmsgAsString = "No solution found:\n";
    std::string fbLine, fbLinePrev;
    for (size_t i = 0; i < ex.lastActionResult.size(); ++i)
    {
      const aff::ActionResult& fb = ex.lastActionResult[i];
      fbLine = fb.reason + " Suggestion: " + fb.suggestion + "\n";

      if (fbLine != fbLinePrev)
      {
        fbmsgAsString += "  Issue " + std::to_string(i) + ": " + fbLine;
      }
      fbLinePrev = fbLine;
    }
    // size_t i = 0;
    // for (const auto& fb : ex.lastActionResult)
    // {
    //   fbmsgAsString += "  Issue " + std::to_string(i) + ": " + fb.reason + " Suggestion: " + fb.suggestion + "\n";
    //   ++i;
    // }

    RLOG_CPP(0, fbmsgAsString);

    return fbmsgAsString;
  })

  //////////////////////////////////////////////////////////////////////////////
  // Predict and execute action sequence as tree, non-blocking version
  //////////////////////////////////////////////////////////////////////////////
  .def("plan_fb_nonblock", [](aff::ExampleActionsECS& ex, std::string sequenceCommand) -> std::string
  {
    static std::mutex mtx;
    static std::lock_guard<std::mutex> lock(mtx);
    Timer_waitDT(0.1);

    static std::string prev_seq = sequenceCommand;

    if (ex.isProcessingAction())
    {
      RLOG_CPP(0, "Skipped " << sequenceCommand << ": I am already doing something else: " << prev_seq);
      return prev_seq;
    }

    ex.setProcessingAction(true);
    ex.getEntity().publish("FreezePerception", true);
    ex.getEntity().publish("PlanDFSEE", sequenceCommand);

    prev_seq = sequenceCommand;

    return std::string();
  })

  //////////////////////////////////////////////////////////////////////////////
  // Query non-blocking planner
  //////////////////////////////////////////////////////////////////////////////
  .def("query_fb_nonblock", [](aff::ExampleActionsECS& ex) -> std::string
  {
    if (ex.isProcessingAction())
    {
      return std::string();
    }

    // We unfreeze the perception the first time we see that processing
    // has finished
    ex.getEntity().publish("FreezePerception", false);

    if (ex.lastActionResult[0].success())
    {
      RLOG_CPP(0, "SUCCESS");
      return "SUCCESS";
    }

    std::string fbmsgAsString = "No solution found:\n";
    std::string fbLine, fbLinePrev;
    for (size_t i = 0; i < ex.lastActionResult.size(); ++i)
    {
      const aff::ActionResult& fb = ex.lastActionResult[i];

      if (fb.error == "Actions interrupted")
      {
        return "INTERRUPT";
      }

      fbLine = fb.reason + " Suggestion: " + fb.suggestion + "\n";

      if (fbLine != fbLinePrev)
      {
        fbmsgAsString += "  Issue " + std::to_string(i) + ": " + fbLine;
      }
      fbLinePrev = fbLine;
    }

    RLOG_CPP(0, fbmsgAsString);

    return fbmsgAsString;
  })

  .def("plan", [](aff::ExampleActionsECS& ex, std::string sequenceCommand) -> bool
  {
    PollBlockerComponent blocker(&ex);
    ex.getEntity().publish("PlanDFSEE", sequenceCommand);
    blocker.wait();
    bool success = ex.lastActionResult[0].success();
    RLOG(0, "   success=%s   result=%s", success ? "true" : "false",
         ex.lastActionResult[0].error.c_str());

    return success;
  })

  //////////////////////////////////////////////////////////////////////////////
  //
  //////////////////////////////////////////////////////////////////////////////
  .def("plan_fb_rich", [](aff::ExampleActionsECS& ex,
                          std::string sequenceCommand,
                          bool successes_only,
                          size_t max_threads) -> nlohmann::json
  {
    const std::string actionSequence = aff::ActionSequence::resolve(ex.getGraph()->cfgFile, sequenceCommand);
    RLOG_CPP(0, "Processing sequence: '" << actionSequence << "'");
    std::vector<std::string> seq = Rcs::String_split(actionSequence, ";");

    auto tree = ex.getQuery()->planActionTree(aff::PredictionTree::SearchType::DFSMT, seq, ex.getEntity().getDt(),
                                              0, true, ex.earlyExitAction);

    nlohmann::json j_inner = {
      {"lifted_actions",    nlohmann::json::array()},
      {"actions",    nlohmann::json::array()},
      {"success",    false},
      {"error",      ""},
      {"reason",     ""},
      {"suggestion", ""},
      {"developer",  ""},
      {"cost",       0.0},
      {"duration",   0.0}
    };

    if (!tree)
    {
      j_inner["error"] = "Failed to compute prediction tree";
      return nlohmann::json::array({ j_inner });
    }

    if (!tree->root->feedbackMsg.error.empty())
    {
      j_inner["error"] = "Error in Solution 0";
      return nlohmann::json::array({ j_inner });
    }

    // Handling a fatal error in the syntax for the first action
    std::vector<aff::PredictionTreeNode*> slnPath = tree->findSolutionPath(0, false);

    if (slnPath.empty())
    {
      j_inner["error"] = "Prediction tree contains no solutions";
      return nlohmann::json::array({ j_inner });
    }

    std::vector<aff::PredictionTreeNode*> leafs = tree->getLeafNodes(successes_only);

    // Find the deepest level of the leaf nodes
    int deepest_level = 0;
    for (auto leaf : leafs)
    {
      deepest_level = std::max(deepest_level, leaf->level);
    }

    // Remove all nodes not at deepest level
    leafs.erase(
      std::remove_if(leafs.begin(), leafs.end(),
                     [deepest_level](aff::PredictionTreeNode* node)
    {
      return node->level < deepest_level;
    }),
    leafs.end());


    // Reporting only the nodes with the longest failure
    nlohmann::json j_result = nlohmann::json::array();

    for (auto leaf : leafs)
    {
      // Accumulate the grounded sequence command
      auto slnPath = tree->getPathToNode(leaf);
      std::vector<std::string> predictedSeq;
      predictedSeq.reserve(slnPath.size());

      double sln_duration = 0.0;
      for (auto node : slnPath)
      {
        predictedSeq.push_back(node->actionCommand());
        sln_duration += node->duration;
      }

      const aff::ActionResult& errMsg = slnPath.back()->feedbackMsg;
      j_result.push_back(
      {
        {"lifted_actions", actionSequence},
        {"actions", predictedSeq},
        {"success", slnPath.back()->success},
        {"error", errMsg.error},
        {"reason", errMsg.reason},
        {"suggestion", errMsg.suggestion},
        {"developer", errMsg.developer},
        {"cost", slnPath.back()->cost},
        {"duration", sln_duration}
      });

    }

    return j_result;
  },
  py::arg("sequenceCommand"),
  py::arg("successes_only") = true,
  py::arg("max_threads") = 0,
  R"pbdoc(
Plans an action sequence and returns detailed feedback for each attempted
solution.

This function resolves a semicolon-separated string of action commands into a
sequence of robot actions. It attempts to plan and evaluate possible execution
paths using a depth-first search strategy. The result includes detailed
feedback for the deepest (most complete) failed solution paths, or the
successful path(s) if available.

Parameters
----------
sequence_command : str
    A semicolon-separated sequence of high-level action commands to execute.
    Example: "get bottle_of_tomato_sauce; put bottle_of_tomato_sauce tray frame tray_position_5"

successes_only: bool
    If true, only successful paths will be returned. Otherwise, all paths that
    reach the overall deepest level will be returned.

max_threads: int
    The maximum number of threads used in the depth-first search. If
    max_threads is 0 (default), the number of threads will be determined by
    the computer's thread affinity (as many as possible)

Returns
-------
List[dict]
    A list of result dictionaries, each containing:
      - actions (List[str]): The list of action strings that were executed or
        planned.
      - success (bool): Whether the plan was successful.
      - error (str): Description of the failure or error ("SUCCESS" if
        successful).
      - reason (str): More specific explanation of the failure, if available.
      - suggestion (str): Suggested corrective action.
      - developer (str): Developer-oriented debug message, if applicable.
      - cost (float): Planning cost of the solution (lower is better).

    The list is sorted according to the quality of the solution. The first
    entries are the successful solutions, sorted by their accumulated cost (the
    first entry is the overall best solution path). This is followed by
    solutions that contain the most number of steps. If planning fails
    completely, a single-element list is returned with an error summary. If
    multiple deepest failure paths exist, each found one is reported.

Example
-------
>>> results = sim.plan_fb_rich("get bottle_of_tomato_sauce; put bottle_of_tomato_sauce tray")
>>> for r in results:
...     print("Actions:", r["actions"])
...     print("Success:", r["success"])
...     print("Error:", r["error"])
...     print("Suggestion:", r["suggestion"])

)pbdoc")



;

}
