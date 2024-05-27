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

There are a few parameters that are common for most actions:

```bash
<action-name>( <"duration" duration_in_seconds (optional)>
               <"fast" (optional)>)
```
The duration parameter specifies the time the action will take in seconds. This 
has an effect only if the actions's "turbo" flag is not set. 

The parameter fast is enabling the
"turbo" flag. It can also be set globally with the through the 
ActionBase::setTurob() method. This will have an effect on all created actions. 
In case the turbo flag is active, the duration of the action will be determined 
to be as fast as possible when being planned using the PredictionTree class. 
In this case, the action prediction will compute a scaling factor that, when 
applied to the action's duration, will lead to the joint speeds reaching the 
(maximum_speed/TURBO_DURATION_SCALER).


### The "get" action

Grasps an object and lifts it a little bit up. If no manipulator (which-hand) is given,
the action will determine the best possible manipulator. The same applies to the 
grasp-to-use. 

```bash
get(<object-to-pick-up> 
    <which-hand (optional)> 
    <grasp-to-use (optional, one out of: PowerGrasp, PincerGrasp, PalmGrasp, BallGrasp, CircularGrasp, TwistGrasp)>
    <"from" parent-object (optional)> 
    <"liftHeight" height_in_meters (optional)>)
```

Internally, the action matches
the manipulator's capabilities with the object's affordances. These combinations are
supprted:

Affordance      | Capability             | Alignment
----------------| ---------------------  | ---------------------
PowerGraspable  | PowergraspCapability   | Affordance frame z - Capability frame z
PincerGraspable | PincergraspCapability  | Affordance frame -x - Capability frame x
BallGraspable   | PincergraspCapability  | Position algined, Capability frame x inclined wrt. world
PalmGraspable   | PalmgraspCapability    | 6-dof frame alignment

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
    <"near" neart_entity (optional)>
    <"far" far-entity (optional)>
    <"distance" near-far-distance (optional)>
```

The object-to-put must be in one of the agent's hands (previously grasped). It also needs to
have a Stackable affordance. If no target is given, the closest entity below the object-to-put
is searched. If a target is given, it must have a Supportable affordance. If a target is given,
the "frame" attribute allows to specify a Supportable frame explicitely. This is sometimes 
helpful if an entity provides many Supportable surfaces, such as a table with a grid of them.

There are currently two additional spatial priors: near and far. Both can specify an entity or 
an agent. The put locations will pe selected among those that are closer or more far away than 
a distance threshols, which can be given through the distance keyword. The default is 0.4m.
All Supportables that are farther away (for "Near") or closer (for "far) will be ignored. All
remaining supportables are sorted by their distance so that closer (for "near") or fartner (for
"far") put places are preferred.

For instance:

```bash
put lemon cutboard                   // Puts a lemon on the cutboard.
put lemon table frame tablegrid_1    // Puts a lemon on the table's Supportable tablegrid_4
put lemon near orange                // Puts a lemon near the orange
```


### The "magic_get" and "magic_put" actions

These are convenience actions for testing. They follow the same syntax as the get and put actions, 
but will "beam" the object into the hand of the agent or the target location. It is unsafe to use
these actions when working with a real robot, since they introduce jumps in the coordinates.


### The "pour" action

Pour an ingredient from a "Pourble" affordance into a "Containable" affordance. It is assumed
that the Pourable and Containable have a frame with z-axis pointing up. The pour action allows
to pour into Containables that are placed somewhere in the environemnt, or held by a human
or robot agent. It is currently assumed that the pouring entity has exactly one Pourable, and
the entity to be poured in exactly one Containable. 

```bash
pour(<object to pour from>,<object to pour into>)
```

There is also some logic included concerning fill levels and transitioning, which needs
to be reconsidered.

### The "pose" action

```bash
pose(<name-of-a-model-state-pose>)
```

This action loads the passed model state from the graph's configuration file and
moves all joints into this pose. If a joint is not part of the model state, it
will not be moved. It is possible to specify several poses that are separated by a 
comma. In that case, they will be treated as alternatives, and the most feasible one
will be selected. This allows to give the pose command a bit more robistness.


### The "point" action

```bash
point(<object-or-agent-to-point-at> 
      <manipulator (optional)> 
```

Points at an object or agent. Anything that is specified as a type is being considered.
If the entity to point at is a HumanAgent, the pointing will be directed towards the
agent's head. The hand that points will be selected automatically, except if a second
argument is passed. This will be interpreted as the manipulator that is to point.

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
  make -j24
```

  - This should build several executables into the bin directory

## How to start the websocket action server

  - bin/TestLLMSim -websocket (Port is 35000, that's the default)
  - with command line options printed to console: bin/TestLLMSim -h 

## Todos

  - inf in prediction cost
  - used_manipulators
  - open-world
  - smart pointers for tree nodes and prediction results
  - Pass explanations from tree to python wrapper
  - nicer function for pose sequences
  - improve Affordance documentation



