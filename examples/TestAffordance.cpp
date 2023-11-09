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

// This file is included from Rcs/examples (build tree) or
// Rcs/1.0/include/RcsExamples (install tree)

// #include <ExampleRunner.hpp>

#include <AffordanceEntity.h>
#include <ActionGet.h>

#include <Rcs_resourcePath.h>
#include <Rcs_cmdLine.h>
#include <Rcs_macros.h>



using namespace aff;

int main(int argc, char** argv)
{
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Rcs log level");
  // Rcs_addResourcePath("config/xml/SmileActions");
  // auto scene = ActionScene::parse("config/xml/SmileActions/g_affordance_test.xml");
  Rcs_addResourcePath("config/xml/SmileSimulator");
  auto scene = ActionScene::parse("config/xml/SmileSimulator/g_scenario_pizza.xml");
  scene.print();

  // Test assignment operator
  {
    RLOG(0, "Assigning starts");
    ActionScene newScene;
    newScene = scene;
    RLOG(0, "Assigning ends");
    RPAUSE();
  }

  {
    auto res = match<Containable, Pourable>(&scene.entities[0], &scene.entities[1]);
    std::cout << std::endl << std::endl;
    std::cout << "Result match<Containable, Pourable> has " << res.size()
              << " elements:" << std::endl;
    for (const auto& r : res)
    {
      std::cout << "Affordance 0: ";
      std::get<0>(r)->print();
      std::cout << " Affordance 1: ";
      std::get<1>(r)->print();
      //std::cout << " Quality :" << std::get<2>(r) << std::endl;
    }
    std::cout << std::endl << std::endl;
  }

  {
    auto res = match<Supportable, Stackable>(&scene.entities[0], &scene.entities[1]);
    std::cout << std::endl << std::endl;
    std::cout << "Result match<Supportable, Stackable> has " << res.size()
              << " elements:" << std::endl;
    for (const auto& r : res)
    {
      std::cout << "Affordance 0: ";
      std::get<0>(r)->print();
      std::cout << " Affordance 1: ";
      std::get<1>(r)->print();
      //std::cout << " Quality :" << std::get<2>(r) << std::endl;
    }
    std::cout << std::endl << std::endl;
  }

  {
    auto res = match<PowerGraspable>(&scene.entities[1], &scene.manipulators[0]);
    std::cout << "Result match<PowerGraspable> has " << res.size() << " elements:"
              << std::endl;
    for (const auto& r : res)
    {
      std::cout << "Affordance: ";
      std::get<0>(r)->print();
      std::cout << " Capability: ";
      std::get<1>(r)->print();
      //std::cout << " Quality :" << std::get<2>(r) << std::endl;
    }
    std::cout << std::endl << std::endl;
  }

  {
    auto res = match<Graspable>(&scene.entities[1], &scene.manipulators[0]);
    std::cout << "Result match<Graspable> has " << res.size() << " elements:"
              << std::endl;
    for (const auto& r : res)
    {
      std::cout << "Affordance: ";
      std::get<0>(r)->print();
      std::cout << " Capability: ";
      std::get<1>(r)->print();
      //std::cout << " Quality :" << std::get<2>(r) << std::endl;
    }
    std::cout << std::endl << std::endl;
  }

  {
    auto res = getAffordances<Graspable>(&scene.entities[1]);
    std::cout << "Result getAffordances<Graspable> has " << res.size() << " elements:"
              << std::endl;
    for (const auto& r : res)
    {
      std::cout << "  Affordance: ";
      r->print();
      std::cout << std::endl;
    }
    std::cout << std::endl << std::endl;
  }

  {
    RcsGraph* graph = RcsGraph_create("config/xml/SmileActions/g_affordance_test.xml");
    RCHECK(graph);

    try
    {
      std::vector<std::string> mParams = {"cola_bottle_1", "hand_left"};
      ActionGet a(scene, graph, mParams);

      for (size_t i = 0; i < a.getNumSolutions(); ++i)
      {
        a.initialize(scene, graph, i);
        a.print();
      }
      //auto traj = a.createTrajectory(0.0, 10.0);
      //traj->print();
      //traj->toXML("traj.xml");
    }
    catch (const std::exception& ex)
    {
      RLOG_CPP(0, "Failed to create action: " << ex.what());
    }
    catch (...)
    {
      RLOG_CPP(0, "Failed to create action");
    }



    RcsGraph_destroy(graph);
  }


  xmlCleanupParser();

  return 0;
}
