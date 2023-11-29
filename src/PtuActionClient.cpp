/*******************************************************************************

  Copyright (c) Honda Research Institute Europe GmbH

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are
  met:

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

#include "EntityBase.h"

#include <Rcs_resourcePath.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_parser.h>
#include <Rcs_cmdLine.h>
#include <SegFaultHandler.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <hri_scitos_schunk_ptu/PtuGotoAction.h>

#include <fstream>
#include <string>
#include <iostream>
#include <csignal>
#include <sstream>

RCS_INSTALL_ERRORHANDLERS


/*******************************************************************************
 *
 ******************************************************************************/
void sigQuit(int /*sig*/)
{
  static int kHit = 0;
  ros::shutdown();
  fprintf(stderr, "Trying to exit gracefully - %dst attempt\n", kHit + 1);
  kHit++;

  if (kHit == 2)
  {
    fprintf(stderr, "Exiting without cleanup\n");
    exit(0);
  }
}

void doneCb(const actionlib::SimpleClientGoalState& state,
            const hri_scitos_schunk_ptu::PtuGotoResultConstPtr& result)
{
  RLOG(0, "Action server responded with state [%s]", state.toString().c_str());
}

/*******************************************************************************
 *
 ******************************************************************************/
int main(int argc, char** argv)
{
  RMSG("Starting PtuActionClient");

  // Ctrl-C callback handler
  signal(SIGINT, sigQuit);

  // Parse command line for calling with rosrun
  Rcs::CmdLineParser argP(argc, argv);
  argP.getArgument("-dl", &RcsLogLevel, "Debug level (default is 0)");

  // We take care about shutting down ourselves.
  RLOG(0, "Initializing ROS");
  ros::init(argc, argv, "ptu", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  // Construct the client and connect to the server:
  RLOG(0, "Starting action client");
  hri_scitos_schunk_ptu::PtuGotoGoal goal;
  goal.pan_degrees = 30.0;
  goal.pan_velocity_degrees = 10.0;
  goal.tilt_degrees = 10.0;
  goal.tilt_velocity_degrees = 10.0;
  actionlib::SimpleActionClient<hri_scitos_schunk_ptu::PtuGotoAction> actionClient("/ptu/goto", true);

  RLOG(0, "Waiting for server");
  bool foundServer = actionClient.waitForServer();

  if (!foundServer)
  {
    ROS_WARN("could not connect to server; halting");
    return 0;
  }

  RLOG(0, "Connected to action server");


  // Start run-loop until node is shut down
  size_t loopCount = 0;
  while (ros::ok())
  {
    if (loopCount==10)
    {
      RLOG(0, "Sending goal");
      actionClient.sendGoal(goal, &doneCb);
    }

    Timer_waitDT(0.1);
    ros::spinOnce();
    loopCount++;
  }

  actionClient.cancelAllGoals();

  fprintf(stderr, "Thank you for using the gui_action_client_node\n");

  return 0;
}
