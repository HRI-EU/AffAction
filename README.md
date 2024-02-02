# AffAction

Affordances and actions for robot behavior generation. 



## Concepts

Conceptually, this project connects a semantic text-level action language
with the low-level motor control of a robot (or its simulation). 

We represent a scene as a composition of several objects, each of them 
comprising several affordances. For instance, a table can have the 
"Supportable" affordance to indicate that something can be put on its support
surface. Similarly, objects can have the Affodance to be "Graspable", where
this can be detailed down to "Powergraspable" for cylindrical objects etc.
Affordance come typically with a coordinate frame that describes the 
geometry of its interaction. For instance, the "Powergraspable" affordance
is described by a frame that has the z-axis pointing into the invariant
rotation direction.

The scene also contains a set of Manipulators that are composed of several 
"Capabilities". A "Capability" can for instance be a "Powergrasp", a 
"Palmgrasp", a "FingerpushCapability" and more. At runtime, an action is 
checking all possible combinations of Capabilities and Affordances, and 
computes the feasible pairs. 

Here is an example:
The action "get apple" leads to:
- matching all manipulator's (e.g. left and right hand) capabilities (e.g. 
  Powergraps, Palmgrasp) against all object affordances (e.g. how it is 
  graspable). The resulting combinations are sorted according to a heuristic 
  (usually distance in wrench space). In this computation step, the robot's 
  particular embodiment (reachability, joint ranges ...) is not considered.
- The action is simulated forward with a prediction class, and is checked 
  against violations of the robot's limits and against collisiins. The first 
  feasible solution is sent for execution. In this computation step, the 
  robot's embodiment is considered.



## Actions

### The "get" action

Grasps an object and lifts it a little bit up. If no manipulator (which-hand) is given,
the action will determine the best possible manipulator. The same applies to the 
grasp-to-use. 

```bash
get(<object-to-pick-up> 
    <which-hand (optional)> 
    <grasp-to-use (optional, one out of: PowerGrasp, PincerGrasp, PalmGrasp, BallGrasp, CircularGrasp, TwistGrasp)>
    <"from" parent-object (optional)> 
    <"duration" duration_in_seconds (optional)>
    <"liftHeight" height_in_meters (optional)>)
```

Internally, the action matches
the manipulator's capabilities with the object's affordances. These combinations are
supprted:

Affordance      | Capability
----------------| ---------------------
PowerGraspable  | PowergraspCapability
PincerGraspable | PincergraspCapability
BallGraspable   | PincergraspCapability
PalmGraspable   | PalmgraspCapability

The set of matches is evaluated, and the best feasible one is determned. If the "from"
attribute is used, the object-to-pick-up is determined as being a child of the
object-to-pick-up. This allows to give a spatial constraint in case there are several
objects with the same name. For instance:

```bash
get lemon from cutboard   // Gets a lemon that is a child of the cutboard. The hand is auto-selected.
get apple_1 hand_right    // Gets the apple_1 with the right hand.
```


### The "put" action

Action to put an entity on another one. 

```bash
put(<object-to-put> 
    <target (optional)> 
    <"frame" target_frame (optional)>
    <"duration" duration_in_seconds (optional)>)
```

The object-to-put must be in one of the agent's hands (previously grasped). It also needs to
have a Stackable affordance. If no target is given, the closest entity below the object-to-put
is searched. If a target is given, it must have a Supportable affordance. If a target is given,
the "frame" attribute allows to specify a Supportable frame explicitely. This is sometimes 
helpful if an entity provides many Supportable surfaces, such as a table with a grid of them.
For instance:

```bash
put lemon cutboard                   // Puts a lemon on the cutboard.
put lemon table frame tablegrid_1    // Puts a lemon on the table's Supportable tablegrid_4
```

### The "pour" action
pour(<object to pour from>,<object to pour into>)

### The "double_get" action
double_get (<object1, object2>)

### The "double_put" action
double_put(<object1>, <object2>, (target1), (target2))

### The "poke" action
poke(<switch-object>)

### The "gaze" action
gaze(<object to look at>)

### The "open_door" action
open_door(<door-object>)

### The "close_door" action
close_door (<door-object>)


### The "pose" action

```bash
pose(<name-of-a-model-state-pose>)
```

This action loads the passed model state from the graph's configuration file and
moves all joints into this pose. If a join is not part of the model state, it
will not be moved.

### The "push" action (Not yet working)
push(<object to push>)

### The "screw" action (Not yet working)
screw(<object to screw top off>)



## Feedback messages

### Syntax of Error messages:

    ERROR: cannot  REASON: because  SUGGESTION:  DEVELOPER:

  or

    FATAL_ERROR for non-recoverable errors (e.g. Emergency Stop ...)

### Unrecoverable Errors that Need a Replan:

  Semantic Errors:

   - Wrong command syntax  
   - Usage of object names not existing in the environment

  Logical Errors:

  - Try to get an object when both hands are full
  - Try to pour an object into an object placed in a closed container
  - Try to pour an object into an object already full
  - Opening or closing non openable objects
  - Switching on/off non powerable objects

  Physical Errors:

  - Command impossible to perform by the robot because of joint-limit errors
  - Command impossible to perform because object x is an obstacle
  - Command impossible to perform because target object is out of reach



## Software design

The project is composed of three parts:

- Actions: Algorithmic libraries that implement the ActionScene, 
  Affordances, Capabilities and actions.
- Component system: Components that interact with the subscribe - publish 
  mechanism
- Examples: Example classes that work with the ExampleRunner applications.

## How to build

  - Please make sure that you have set the SIT and MAKEFILE_PLATFORM 
    environment variables
  - Please make sure the WM5 library has been compiled into the Rcs 
    dependency. If not, a runtime warning will be issued.
```
  git clone https://github.com/HRI-EU/affaction.git
  cd affaction
  mkdir build
  cd build
  cmake ..
  make 
```

  - This should build several executables into the bin directory

## How to start the websocket action server

  - bin/TestLLMSim -port 35000 (that's the default)
  - with command line options printed to console: bin/TestLLMSim -h 

## Python websocket client

  - cd SmileActions/python
  - python smile_websocket.py "put cola_bottle_1"

