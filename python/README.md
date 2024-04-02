# AffAction python module

The python module wraps a number of functions of the C++ libraries and provides it to thy python world.

## Python virtual environment

### How to create
```
python -m virtualenv venv_affaction
source venv_affaction/bin/activate
pip install -r requirements.txt
```

### How to activate
```
source venv_affaction/bin/activate
```

## Python websocket client

```
cd SmileActions/python
python smile_websocket.py "put cola_bottle_1"
```

## Python LlmSim API

### Initialization

Between the instantiation and initialization of the simulator, a number of member 
variables can be configured to define the behavior of the simulator. Below, all
members are stated, along with their defaults:

```
sim = LlmSim()

sim.useWebsocket = True
sim.unittest = False
sim.sequenceCommand = ""
sim.noTextGui = False
sim.speedUp = 3 
sim.xmlFileName = "g_group_6.xml"
sim.configDirectory = "config/xml/AffAction/xml/examples"
sim.noLimits = False
sim.noCollCheck = False
sim.noTrajCheck = False
sim.verbose = True
sim.processingAction = False
sim.noViewer = False

sim.init(True)
```

* useWebSocket: Starts a websocket server that can receive action command strings 
                from a websocket client. See above for how to start the client.
* unittest: Runs an automated test sequence and does statistics on the
            success. The action string can be put into the member sequenceCommand,
            or specified through a Gui.
* sequenceCommand: An action string that will be processed directly after start.
* speedUp: 1 for wall-clock time, 3 for 3 times as fast ...
* xmlFileName: Name of graph config file.
* configDirectory: Search path for config files.
* noLimits: Ignore all check results in IK steps. 
* noCollCheck: Ignore collision check results in IK steps. 
* noTrajCheck: Don't perform trajectory validity check after the prediction step. 
               The best feasible trajectory will be passed to the run-time controller. 
               If this flag is true, the predicted trajectory cannot be visualized 
               (since no prediction takes place)
* verbose: If true, the action result message will be printed to the console. In order 
           to completely silence the simulator, the debug level needs to be set to -1, 
           and the verbose flag needs to be false.
* noViewer: Start in headless mode. It is equivalent to call init(False)

### Querying information from the scene

The querying functions are implemented thread-safe and can be called while the 
simulator is running. There is a small performance penalty, since each
qurey function performs a copy of the graph and scene. This copy operation
is locking the mutex of the simulator's step method. If used excessively,
this can lead to slowing down the simulator. Ideally, the query functions
should therefore not be called in a loop or so. Here they are:

```
json getOccludedObjectsForAgent(String agentName) 
```

Returns empty json if the agent can see all objects or a json in the form:
{"occluded": ["entity name 1", "entity name 2"] }

```
json isOccludedBy(String agentName, String objectName) 
```

Returns empty json if not occluded, or occluding objects sorted by distance
to eye (increasing): {"occluded_by": ["id_1", "id_2"] }

```
Bool isBusy(String agentName) 
```

Returns a boolean indicating if any scene entity is closer to any hand of
the agent closer than a distance threshold (0.15m).

```
np::array getPanTilt(String roboAgent, String gazeTarget) 
```

Returns the pan tilt angles for the agent when it looks at the gazeTarget. If no
solution is found, the numpy array will be empty.

```
Bool isReachable(String agentName, String objectName) 
```

Kinematic check of all agents if the object is within a reachable range.
For the robot agent: If the result is true, it does not necessarily mean
that it can be grasped.

```
String get_state() 
```

Legacy function that returns a string containing a detailed description of all 
entities, agents, manipulators, including lots of parameters. Please check the implementation if you are interested in the details.

```
json get_objects() 
```

Returns a json in the form:
{"objects": ['iphone', 'red_glass', 'fanta_bottle'] }

```
json get_agents() 
```

Returns a json in the form:
{"agents": ['Daniel', 'Felix', 'Robot'] }

### Planning, predicting, executing

### Adding components
