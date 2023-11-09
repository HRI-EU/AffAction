#!/usr/bin/python
# -*- coding: utf-8 -*-
#
#  Copyright (c) Honda Research Institute Europe GmbH.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice,
#     this list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice,
#     this list of conditions and the following disclaimer in the documentation
#     and/or other materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its
#     contributors may be used to endorse or promote products derived from
#     this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
#  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
#  IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
#  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
#  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
#  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
#  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
import sys
sys.path.append('lib')
sys.path.append('lib/Release')

from pyAffaction import *
b = LlmSim()
b.noTextGui = True
b.unittest = True
b.useWebsocket = False
b.speedUp = 1
b.noLimits = True
b.init(True)
b.run()
b.stop()
b.run("get salt_bottle")
b.run("put salt_bottle shelf")
b.run("get salt_bottle;put salt_bottle shelf")

setLogLevel(-1)
b.showGraphicsWindow()
b.hideGraphicsWindow()
b.render()





import json
import sys
sys.path.append('lib')
sys.path.append('lib/Release')

from pyAffaction import *
#addResourcePath("config/xml/Affaction/pizza")
addResourcePath("config/xml/Affaction/examples")
#l = LandmarkBase("g_scenario_pizza.xml")
l = LandmarkBase("g_aruco.xml")

l.addSkeletonTracker(3)

jsonFile = open("config/xml/Affaction/50_landmarks.json")
#jsonFile = open("config/xml/Affaction/from_python.txt")
json_string = jsonFile.read()
json_object = json.loads(json_string)

l.startCalibration("camera", 20)

# This should run in a loop

# Missing: Acquire json_object from camera (ROS or other)
l.setJsonInput(json_object)       # Update all tracker's landmarks
l.updateScene(getWallclockTime()) # Update RcsGraph and ActionScene
l.getState()                      # This returns the json of the scene
l.updateSim(b)                    # Copy graph to LlmSim (visualization)




################################################################################
#
# Simple test running a LandmarkBase perception class that regularly updates 
# the graph of a LlmSim class for visualization
#
################################################################################
import sys
import json
import time

sys.path.append('lib')
sys.path.append('lib/Release')

from pyAffaction import *

addResourcePath("config/xml/Affaction/examples")

def graphicsTest():
    b = LlmSim()
    b.noTextGui = True
    b.useWebsocket = False
    b.speedUp = 1
    b.noLimits = True
    b.xmlFileName = "g_dialogue.xml"
    b.init(True)
    b.run()
    # Mimic perception: Load landmark json from file
    jsonFile = open("config/xml/Affaction/data/50_landmarks.json")
    json_string = jsonFile.read()
    json_object = json.loads(json_string)
    # Create LandmarkBase component with link to LlmSim (including graphics)
    l = LandmarkBase(b)
    
    st = l.addSkeletonTracker(3)

    # use this:
    # st.addAgent("Bob")
    # st.addAgent("Dave")
    # st.addAgent("Peter") # not defined in config, supposed to fail.

    # OR use this:
    st.addAgents();

    l.setSim(b)


    l.setCameraTransform("camera")
    #
    while b.isRunning():
        l.setJsonInput(json_object)       # Update all tracker's landmarks
        t = getWallclockTime()
        t = 1696502752.905868             # Set time to json file to show skeleton
        l.updateScene(t)                  # Update RcsGraph and ActionScene
        l.getState()                      # This returns the json of the scene
        time.sleep(1)

graphicsTest()

################################################################################
#
# Simple LandmarkBase without any trackers, for function testing
#
################################################################################

import json
import sys
sys.path.append('lib')
sys.path.append('lib/Release')

from pyAffaction import *

addResourcePath("config/xml/Affaction/examples")

b = LlmSim()
b.noTextGui = True
b.useWebsocket = False
b.speedUp = 1
b.noLimits = True
b.xmlFileName = "g_scenario_pizza.xml"
b.xmlFileName = "g_dialogue.xml"
b.init(True)
b.run()

l = LandmarkBase(b)
l.setSim(b)

# Tests:
l.getAffordanceFrame("oven", AffordanceType.Hingeable);
l.getAffordanceFrame("pizza_dough", AffordanceType.Containable);
l.getPanTilt("Robo", "tomato_sauce_bottle")
#l.getAffordanceFrame("DOES_NOT_EXIST", AffordanceType.Containable);
#l.getAffordanceFrame("oven", AffordanceType.DOES_NOT_EXIST);

###############################################################################
