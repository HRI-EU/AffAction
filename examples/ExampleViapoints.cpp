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

#include "ExampleActionsECS.h"
#include "PickAndPlaceConstraint.cpp"

#include <ExampleFactory.h>

#include <ActivationSet.h>
#include <PositionConstraint.h>
#include <PolarConstraint.h>
#include <VectorConstraint.h>
#include <ConnectBodyConstraint.h>



namespace aff
{

class ExamplePlayBackViapoints : public ExampleActionsECS
{
public:

  ExamplePlayBackViapoints(int argc, char** argv) : ExampleActionsECS(argc, argv)
  {
  }

  virtual ~ExamplePlayBackViapoints()
  {
  }

  bool parseArgs(Rcs::CmdLineParser* parser)
  {
    bool res = ExampleActionsECS::parseArgs(parser);
    parser->getArgument("-debug", &debug, "Debug mode: no limits and checks");
    parser->getArgument("-inputFile", &inputFile, "Input file name (default: %s)", inputFile.c_str());
    parser->getArgument("-outputFile", &outputFile, "Output file name (default: %s)", outputFile.c_str());

    if (debug)
    {
      noTrajCheck = true;
      noLimits = true;
    }

    return res;
  }

  bool initParameters()
  {
    ExampleActionsECS::initParameters();
    xmlFileName = "g_iros25.xml";
    inputFile = "test_robot_traj.txt";
    outputFile = "action_iros.xml";
    debug = false;
    zigzag = true;
    return true;
  }

  virtual bool initGraphics()
  {
    bool success = ExampleActionsECS::initGraphics();
    if (!success)
    {
      return false;
    }

    viewer->setKeyCallback('F', [this](char k)
    {
      RLOG(0, "Creating action file");
      bool success = createActionFile(inputFile, outputFile);

      if (!success)
      {
        RLOG(0, "Failed to create action file");
      }
      else
      {
        if (debug)
        {
          entity.publish("ActionSequence", std::string("load action_iros.xml; pose default_top"));
        }
        else
        {
          entity.publish("PlanDFSEE", std::string("load action_iros.xml; pose default_top"));
        }
      }

    }, "apply via point policy");

    return success;
  }

  std::string help()
  {
    std::string str = "Push 'F' key to generate action from file 'test_robot_traj.txt' in the build folder\n\n";
    str += ExampleActionsECS::help();
    return str;
  }

  static bool createActionFile(std::string inFile, std::string outFile)
  {
    // Read data file
    MatNd* trj = MatNd_createFromFile(inFile.c_str());

    if (!trj)
    {
      RLOG_CPP(1, "Failed to read input file: '" << inFile << "'");
      return false;
    }

    REXEC(1)
    {
      MatNd_printCommentDigits(inFile.c_str(), trj, 5);
    }

    std::ofstream fd;
    fd.open(outFile.c_str());

    if (!fd.good())
    {
      RLOG_CPP(1, "Failed to open file " << outFile);
      return false;
    }

    // Open set's xml description. The class name is polymorphic
    fd << "<Action name='iros25' >" << std::endl << std::endl;

    // Here come the tasks
    std::string tasks;
    tasks += "  <Task name='hand_left'  controlVariable='XYZ' effector='hand_left_pincergrasp'  refBdy='table' active='true' />\n";
    tasks += "  <Task name='hand_left_ori'  controlVariable='POLAR' effector='hand_left_pincergrasp' refBdy='table' axisDirection='X' active='true' />\n";
    tasks += "  <Task name='fingers_left'  controlVariable='Joints' jnts='j2s7s300_joint_finger_1_left j2s7s300_joint_finger_2_left j2s7s300_joint_finger_3_left' />\n";
    fd << tasks << std::endl;

    const double initialTimeoffset = 0.0;
    const double finalTimeOffset = 0.0;

    // Open fingers: 0.01, close fingers: 0.6
    std::vector<double> fingersClosed = std::vector<double>(3, 0.6);
    std::vector<double> fingersOpen = std::vector<double>(3, 0.01);
    double t_final = MatNd_get(trj, trj->m - 1, 0) + initialTimeoffset + finalTimeOffset;

    std::unique_ptr<tropic::ActivationSet> a = std::make_unique<tropic::ActivationSet>();
    a->addActivation(0.05, true, 0.5, "hand_left");
    a->addActivation(t_final, false, 0.5, "hand_left");
    a->addActivation(0.05, true, 0.5, "fingers_left");
    a->addActivation(t_final, false, 0.5, "fingers_left");
    a->addActivation(0.05, true, 0.5, "hand_left_ori");
    a->addActivation(t_final, false, 0.5, "hand_left_ori");

    a->add(std::make_shared<tropic::PolarConstraint>(5.0, 0.9 * M_PI, 0.0, "hand_left_ori"));

    // Go through all rows of the input file
    for (size_t i = 0; i < trj->m; ++i)
    {
      const double* row = MatNd_getRowPtr(trj, i);
      const std::vector<double>& fingerAngles = (row[4] < 0.5) ? fingersOpen : fingersClosed;
      a->add(std::make_shared<tropic::PositionConstraint>(row[0] + initialTimeoffset, row[1], row[2], row[3], "hand_left"));
      a->add(std::make_shared<tropic::VectorConstraint>(row[0] + initialTimeoffset, fingerAngles, "fingers_left"));
    }

    for (size_t i = 1; i < trj->m; ++i)
    {
      double t = MatNd_get(trj, i, 0);;
      double prevGrip = MatNd_get(trj, i-1, 4);
      double grip = MatNd_get(trj, i, 4);

      if (grip <= 0.5 && prevGrip>0.5)   // release
      {
        a->add(std::make_shared<tropic::PickAndPlaceConstraint>(t + initialTimeoffset, "hand_left_pincergrasp"));
      }
      else if (grip >=0.5 && prevGrip < 0.5)   // get
      {
        a->add(std::make_shared<tropic::PickAndPlaceConstraint>(t + initialTimeoffset, "hand_left_pincergrasp"));
      }

    }


    a->toXML(fd, 2);
    fd << std::endl;
    fd << "</Action>" << std::endl;

    fd.close();

    MatNd_destroy(trj);

    return true;
  }

private:

  bool debug;
  std::string inputFile, outputFile;
};

RCS_REGISTER_EXAMPLE(ExamplePlayBackViapoints, "Actions", "Via point action");

}   // namespace aff
