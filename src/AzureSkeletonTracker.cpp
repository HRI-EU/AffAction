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

#include "AzureSkeletonTracker.h"
#include "ActionScene.h"

#include <Rcs_typedef.h>
#include <Rcs_math.h>
#include <Rcs_macros.h>
#include <Rcs_timer.h>
#include <Rcs_shape.h>
#include <Rcs_quaternion.h>
#include <Rcs_body.h>

#include <Rcs_graphicsUtils.h>
#include <SphereNode.h>
#include <VertexArrayNode.h>
#include <COSNode.h>

#include <tuple>
#include <chrono>


#define NUM_FRAMES      (32)
#define DEFAULT_MAX_AGE (2.0)

namespace aff
{
static void lpFiltTrf(double filtVec[6], const HTr* raw, double tmc)
{
  HTr filt;
  HTr_from6DVector(&filt, filtVec);
  HTr_firstOrderLPF(&filt, raw, tmc);
  HTr_to6DVector(filtVec, &filt);
  // HTr_to6DVector(filtVec, raw);   // Uncomment this for no filtering
}

/*
{"confidence":1,"orientation":{"w":0.7256067991256714,"x":-0.10894668847322464,"y":0.4945780634880066,"z":-0.46585193276405334},"position":{"x":0.03961627197265625,"y":0.20496153259277344,"z":1.744385986328125}}
*/
static HTr parsePose(const nlohmann::json& json)
{
  HTr trf;
  trf.org[0] = json["position"]["x"];
  trf.org[1] = json["position"]["y"];
  trf.org[2] = json["position"]["z"];

  double quat[4];  // w-x-y-z, see k4a_quaternion_t
  quat[0] = json["orientation"]["w"];
  quat[1] = json["orientation"]["x"];
  quat[2] = json["orientation"]["y"];
  quat[3] = json["orientation"]["z"];
  Quat_toRotationMatrix(trf.rot, quat);

  double confidence = json["confidence"];

  return trf;
}

typedef enum
{
  PELVIS = 0,
  SPINE_NAVEL = 1,
  SPINE_CHEST = 2,
  NECK = 3,
  CLAVICLE_LEFT = 4,
  SHOULDER_LEFT = 5,
  ELBOW_LEFT = 6,
  WRIST_LEFT = 7,
  HAND_LEFT = 8,
  HANDTIP_LEFT = 9,
  THUMB_LEFT = 10,
  CLAVICLE_RIGHT = 11,
  SHOULDER_RIGHT = 12,
  ELBOW_RIGHT = 13,
  WRIST_RIGHT = 14,
  HAND_RIGHT = 15,
  HANDTIP_RIGHT = 16,
  THUMB_RIGHT = 17,
  HIP_LEFT = 18,
  KNEE_LEFT = 19,
  ANKLE_LEFT = 20,
  FOOT_LEFT = 21,
  HIP_RIGHT = 22,
  KNEE_RIGHT = 23,
  ANKLE_RIGHT = 24,
  FOOT_RIGHT = 25,
  HEAD = 26,
  NOSE = 27,
  EYE_LEFT = 28,
  EAR_LEFT = 29,
  EYE_RIGHT = 30,
  EAR_RIGHT = 31

} BodyName;

/*
  Connection indices:

Body
0 1
1 2
2 3

Head
3 26
26 27
27 28
27 30
28 29
30 31

Right arm
2 11
11 12
12 13
13 14
14 15
15 16
15 17

Left arm
2 4
4 5
5 6
6 7
7 8
8 9
8 10

Right leg
0 22
22 23
23 24
24 25

Left leg
0 18
18 19
19 20
20 21

 */

static std::vector<std::pair<int,int>> readConnectionData()
{
  std::vector<std::pair<int,int>> idx;
  idx.push_back(std::pair<int,int>(PELVIS, SPINE_NAVEL));
  idx.push_back(std::pair<int,int>(SPINE_NAVEL, SPINE_CHEST));
  idx.push_back(std::pair<int,int>(SPINE_CHEST, NECK));
  idx.push_back(std::pair<int,int>(NECK, HEAD));
  idx.push_back(std::pair<int,int>(HEAD, NOSE));
  idx.push_back(std::pair<int,int>(NOSE, EYE_LEFT));
  idx.push_back(std::pair<int,int>(NOSE, EYE_RIGHT));
  idx.push_back(std::pair<int,int>(EYE_LEFT, EAR_LEFT));
  idx.push_back(std::pair<int,int>(EYE_RIGHT, EAR_RIGHT));

  idx.push_back(std::pair<int,int>(SPINE_CHEST, CLAVICLE_RIGHT));
  idx.push_back(std::pair<int,int>(CLAVICLE_RIGHT, SHOULDER_RIGHT));
  idx.push_back(std::pair<int,int>(SHOULDER_RIGHT, ELBOW_RIGHT));
  idx.push_back(std::pair<int,int>(ELBOW_RIGHT, WRIST_RIGHT));
  idx.push_back(std::pair<int,int>(WRIST_RIGHT, HAND_RIGHT));
  idx.push_back(std::pair<int,int>(HAND_RIGHT, HANDTIP_RIGHT));
  idx.push_back(std::pair<int,int>(HAND_RIGHT, THUMB_RIGHT));

  idx.push_back(std::pair<int,int>(SPINE_CHEST, CLAVICLE_LEFT));
  idx.push_back(std::pair<int,int>(CLAVICLE_LEFT, SHOULDER_LEFT));
  idx.push_back(std::pair<int,int>(SHOULDER_LEFT, ELBOW_LEFT));
  idx.push_back(std::pair<int,int>(ELBOW_LEFT, WRIST_LEFT));
  idx.push_back(std::pair<int,int>(WRIST_LEFT, HAND_LEFT));
  idx.push_back(std::pair<int,int>(HAND_LEFT, HANDTIP_LEFT));
  idx.push_back(std::pair<int,int>(HAND_LEFT, THUMB_LEFT));

  idx.push_back(std::pair<int,int>(PELVIS, HIP_RIGHT));
  idx.push_back(std::pair<int,int>(HIP_RIGHT, KNEE_RIGHT));
  idx.push_back(std::pair<int,int>(KNEE_RIGHT, ANKLE_RIGHT));
  idx.push_back(std::pair<int,int>(ANKLE_RIGHT, FOOT_RIGHT));

  idx.push_back(std::pair<int,int>(PELVIS, HIP_LEFT));
  idx.push_back(std::pair<int,int>(HIP_LEFT, KNEE_LEFT));
  idx.push_back(std::pair<int,int>(KNEE_LEFT, ANKLE_LEFT));
  idx.push_back(std::pair<int,int>(ANKLE_LEFT, FOOT_LEFT));

  return idx;
}

struct Skeleton
{
  Skeleton();
  ~Skeleton();
  void initGraphics(const RcsGraph* graph, Rcs::Viewer* viewer, const std::string& color);
  void updateGraphics();
  void setExpectedInitialPose(const HTr* pose);
  void setAgent(const HumanAgent* agent);
  void setAlphaRecursive(osg::Node* node, double alpha);

  int trackerId;
  double lastUpdate;
  double age;
  double maxAge;
  bool wasVisible;
  bool isVisible;
  double alphaPrev;
  double alpha;
  std::vector<HTr> markers;
  HTr expectedInitialPose;
  std::string agentName;
  std::vector<std::string> agentTypes;

  // Only graphics from here
  Rcs::Viewer* viewer;
  osg::ref_ptr<osg::Switch> sw;
  osg::ref_ptr<Rcs::VertexArrayNode> lmConnectionsNode;
  std::vector<std::pair<int, int>> connection_idx;
  MatNd* lmConnections;
  std::vector<std::string> visualBodies;
  std::vector<osg::Node*> visualNodes;
};



Skeleton::Skeleton() : trackerId(-1), lastUpdate(0.0), age(DBL_MAX), maxAge(DEFAULT_MAX_AGE),
  wasVisible(false), isVisible(false), alphaPrev(1.0), alpha(1.0), viewer(NULL)
{
  markers.resize(NUM_FRAMES);
  for (auto& m : markers)
  {
    HTr_setIdentity(&m);
  }
  connection_idx = readConnectionData();
  lmConnections = MatNd_create(2 * connection_idx.size(), 3);
  HTr_setIdentity(&expectedInitialPose);
}

Skeleton::~Skeleton()
{
  MatNd_destroy(lmConnections);
}

void Skeleton::setExpectedInitialPose(const HTr* pose)
{
  HTr_copy(&expectedInitialPose, pose);
}

void Skeleton::setAgent(const HumanAgent* human)
{
  agentName = human->name;
  agentTypes = human->types;
  visualBodies = human->manipulators;
  visualBodies.push_back(agentName);
}

void Skeleton::initGraphics(const RcsGraph* graph, Rcs::Viewer* viewer_, const std::string& color)
{
  if (sw.valid())
  {
    RLOG(0, "Skeleton graphics already initialized");
    return;
  }

  this->viewer = viewer_;

  RLOG(5, "Initializing skeleton with color %s", color.c_str());

  this->sw = new osg::Switch();
  sw->setAllChildrenOff();

  std::vector<osg::ref_ptr<Rcs::SphereNode>> sphereNodes;
  std::vector<osg::ref_ptr<Rcs::COSNode>> cosNodes;

  for (size_t i=0; i<NUM_FRAMES; ++i)
  {
    // This makes the spheres point to the marker transforms. It is not
    // thread-safe so that we might in rare situations see jumping
    // sphere points.
    osg::ref_ptr<Rcs::SphereNode> sphNd = new Rcs::SphereNode(Vec3d_zeroVec(), 0.025);
    sphNd->makeDynamic(markers[i].org, markers[i].rot);
    sphereNodes.push_back(sphNd);

    // osg::ref_ptr<Rcs::COSNode> cosNd = new Rcs::COSNode(0.1);
    // cosNd->makeDynamic(markers[i].org, markers[i].rot);
    // cosNodes.push_back(cosNd);
    // sw->addChild(cosNodes[i].get());

    std::string sphereColor = color;
    if (i==HAND_LEFT || i==HAND_RIGHT)
    {
      sphereColor="RED";
    }
    else if (i==HANDTIP_LEFT || i==HANDTIP_RIGHT)
    {
      sphereColor="YELLOW";
    }
    sphereNodes[i]->setMaterial(sphereColor.c_str());
    sw->addChild(sphereNodes[i].get());
  }

  std::vector<double> linkConnections(2*connection_idx.size());

  // Same as above here: The updating of the lmConnections array is not thread-
  // safe and we might see spurious rendering issues. We accept this in
  // favour of saving a more time-consuming communication with the viewer.
  lmConnectionsNode = new Rcs::VertexArrayNode(lmConnections);
  lmConnectionsNode->setPointSize(5);
  lmConnectionsNode->setColor("GRAY");
  sw->addChild(lmConnectionsNode.get());
  Rcs::setNodeMaterial("GRAY", lmConnectionsNode);

  viewer->add(sw.get());
}

// Gets called from socket thread
void Skeleton::updateGraphics()
{
  if (!sw.valid())
  {
    return;
  }

  if (isVisible)
  {
    sw->setAllChildrenOn();

    for (unsigned int i = 0; i < connection_idx.size(); ++i)
    {
      double* row1 = MatNd_getRowPtr(lmConnections, 2 * i);
      double* row2 = MatNd_getRowPtr(lmConnections, 2 * i + 1);
      Vec3d_copy(row1, markers[connection_idx[i].first].org);
      Vec3d_copy(row2, markers[connection_idx[i].second].org);
    }
  }
  else
  {
    sw->setAllChildrenOff();
  }


  if (visualNodes.size() < visualBodies.size())
  {
    visualNodes.clear();
    // Get all graph nodes for transparency
    RLOG_CPP(1, "Skeleton " << agentName << " has " << visualBodies.size() << " bodies");
    for (const auto& manipulator : visualBodies)
    {
      viewer->lock();
      std::vector<osg::Node*> nodes = viewer->getNodes(manipulator);
      viewer->unlock();
      visualNodes.insert(visualNodes.end(), nodes.begin(), nodes.end());
    }
    RLOG_CPP(1, "Skeleton " << agentName << " has " << visualNodes.size() << " nodes");
  }
  else
  {
    RLOG_CPP(1, "Skeleton " << agentName << " has " << visualNodes.size() << " osg nodes and alpha " << alpha);
    for (auto& nd : visualNodes)
    {
      //setAlphaRecursive(nd, alpha);
      viewer->updateNodeAlphaRecursive(nd, alpha);
    }
    viewer->updateNodeAlphaRecursive(sw, alpha);
    //setAlphaRecursive(sw, alpha);
  }

}

void Skeleton::setAlphaRecursive(osg::Node* node, double newAlpha)
{
  // first check if we should set the transparency of the current node
  osg::StateSet* stateset = node->getStateSet();
  if (stateset)
  {
    osg::Material* material = dynamic_cast<osg::Material*>(stateset->getAttribute(osg::StateAttribute::MATERIAL));
    if (material)
    {
      // Add transparency
      material->setAlpha(osg::Material::FRONT_AND_BACK, alpha);
    }
  }

  // then traverse the group and call setAlpha on all children
  osg::Group* group = node->asGroup();
  if (group)
  {
    for (size_t i = 0; i < group->getNumChildren(); i++)
    {
      setAlphaRecursive(group->getChild(i), alpha);
    }
  }

}












AzureSkeletonTracker::AzureSkeletonTracker(size_t numSkeletons) :
  newAzureUpdate(false), defaultPosRadius(DBL_MAX), scene(NULL)
{
  HTr_setIdentity(&A_CI);
  for (size_t i=0; i<numSkeletons; ++i)
  {
    skeletons.push_back(std::make_unique<Skeleton>());
  }
}

AzureSkeletonTracker::~AzureSkeletonTracker()
{
}

std::string AzureSkeletonTracker::getRequestKeyword() const
{
  return "body";
}

void AzureSkeletonTracker::setScene(aff::ActionScene* scene_)
{
  this->scene = scene_;
}

void AzureSkeletonTracker::updateGraph(RcsGraph* graph)
{
  updateSkeletons(graph);
  updateAgents(graph);
  newAzureUpdate = false;
}

void AzureSkeletonTracker::updateAgents(RcsGraph* graph)
{
  if (!this->scene)
  {
    return;
  }

  for (auto& agent : scene->agents)
  {
    aff::HumanAgent* human = dynamic_cast<aff::HumanAgent*>(agent);

    if (!human)
    {
      continue;
    }

    for (size_t i=0; i< skeletons.size(); ++i)
    {
      if (skeletons[i]->agentName==human->name)
      {
        if (skeletons[i]->isVisible)
        {
          human->markers = skeletons[i]->markers;
        }

        human->setVisibility(skeletons[i]->isVisible);
        human->lastTimeSeen = skeletons[i]->age;
      }

    }

    if (!human->markers.empty())
    {
      const double tmc = 0.05;
      const RcsBody* bdy = RcsGraph_getBodyByName(graph, human->name.c_str());
      int jidx = RcsBody_getJointIndex(graph, bdy);
      if (jidx!=-1)
      {
        lpFiltTrf(&graph->q->ele[jidx], &human->markers[PELVIS], tmc);
      }

      for (const auto& mName : human->manipulators)
      {
        // RLOG_CPP(0, "Agent " << human->name << " has manipulator " << mName);

        const aff::Manipulator* m = scene->getManipulator(mName);
        RCHECK(m);

        if (m->isOfType("head"))
        {
          bdy = RcsGraph_getBodyByName(graph, m->bdyName.c_str());
          const int jidx = RcsBody_getJointIndex(graph, bdy);
          if (jidx!=-1)
          {
            // \todo: Same as below.
            const HTr* A_PI = (bdy->parentId == -1) ? HTr_identity() : &graph->bodies[bdy->parentId].A_BI;
            const HTr* A_MI = &human->markers[HEAD];   // marker transform in world
            HTr A_PM;   // Transform from manipulator's raw percept to its parent
            HTr_invTransform(&A_PM, A_MI, A_PI);
            lpFiltTrf(&graph->q->ele[jidx], &A_PM, tmc);

            const double* gazePos = bdy->A_BI.org;   // \todo(MG): That's one step behind
            const double* gazeDir = bdy->A_BI.rot[1];
            Vec3d_copy(human->headPosition, gazePos);
            Vec3d_copy(human->gazeDirection, gazeDir);


            // RLOG(0, "FIXME");
            // double* q_rbj = &graph->q->ele[jidx];
            // lpFiltTrf(q_rbj, &human->markers[HEAD], tmc);
            // HTr gaze;
            // HTr_from6DVector(&gaze, q_rbj);
            // const double* gazePos = gaze.org;
            // const double* gazeDir = gaze.rot[1];
            // Vec3d_copy(human->headPosition, gazePos);
            // Vec3d_copy(human->gazeDirection, gazeDir);
          }

        }
        else if (m->isOfType("hand_left"))
        {
          bdy = RcsGraph_getBodyByName(graph, m->bdyName.c_str());
          int jidx = RcsBody_getJointIndex(graph, bdy);
          if (jidx!=-1)
          {
            // A_PI is the Manipulator's parent transform
            const HTr* A_PI = (bdy->parentId == -1) ? HTr_identity() : &graph->bodies[bdy->parentId].A_BI;
            const HTr* A_MI = &human->markers[HANDTIP_LEFT];   // marker transform in world
            HTr A_PM;   // Transform from manipulator's raw percept to its parent
            HTr_invTransform(&A_PM, A_MI, A_PI);
            lpFiltTrf(&graph->q->ele[jidx], &A_PM, tmc);

            //lpFiltTrf(&graph->q->ele[jidx], &human->markers[HANDTIP_LEFT], tmc);
          }
        }
        else if (m->isOfType("hand_right"))
        {
          bdy = RcsGraph_getBodyByName(graph, m->bdyName.c_str());
          const int jidx = RcsBody_getJointIndex(graph, bdy);
          if (jidx!=-1)
          {
            // The marker transforms are represented in world coordinates.
            // In order to consider that the manipulator might have a parent
            // different to the world frame, we transform the raw percepts
            // into the (M)anipulator's (P)arent frame.
            if (bdy->parentId!=-1)
            {
              const HTr* A_PI = (bdy->parentId == -1) ? HTr_identity() : &graph->bodies[bdy->parentId].A_BI;   // Manipulator's parent transform
              const HTr* A_MI = &human->markers[HANDTIP_RIGHT];   // marker transform in world
              HTr A_PM;   // Transform from manipulator's raw percept to its parent
              HTr_invTransform(&A_PM, A_MI, A_PI);
              lpFiltTrf(&graph->q->ele[jidx], &A_PM, tmc);
            }
            else
            {
              lpFiltTrf(&graph->q->ele[jidx], &human->markers[HANDTIP_RIGHT], tmc);
            }
          }
        }

      }
    }

  }

}

// Process aruco frames. Called from control loop (100Hz or so)
void AzureSkeletonTracker::updateSkeletons(RcsGraph* graph)
{
  const double currTime = getCurrentTime();

  for (size_t i=0; i< skeletons.size(); ++i)
  {
    const double age = currTime - skeletons[i]->lastUpdate;

    NLOG_CPP(5, "Skeleton[" << i << "]: lastupdate: " << skeletons[i]->lastUpdate
             << " current time: " << currTime << " age: " << age);

    bool updateSkeletonGraphics = newAzureUpdate;

    skeletons[i]->wasVisible = skeletons[i]->isVisible;
    skeletons[i]->isVisible = (age<=skeletons[i]->maxAge) ? true : false;
    skeletons[i]->age = age;

    // age = 0: alpha=1   age=maxAge: alpha=0
    skeletons[i]->alphaPrev = skeletons[i]->alpha;
    skeletons[i]->alpha = Math_clip((skeletons[i]->maxAge - skeletons[i]->age)/skeletons[i]->maxAge, 0.0, 1.0);
    skeletons[i]->alpha = lround(skeletons[i]->alpha*100.0)/100.0;
    if (skeletons[i]->alpha>0.8)
    {
      skeletons[i]->alpha = 1.0;
    }


    if ((!skeletons[i]->wasVisible) && skeletons[i]->isVisible)
    {
      RLOG_CPP(0, "Skeleton " << skeletons[i]->agentName << " (index " << i << ")" << " appeared");
      updateSkeletonGraphics = true;
    }
    else if (skeletons[i]->wasVisible && (!skeletons[i]->isVisible))
    {
      RLOG_CPP(0, "Skeleton " << skeletons[i]->agentName << " (index " << i << ")" << " disappeared");
      updateSkeletonGraphics = true;
    }

    if (skeletons[i]->alphaPrev != skeletons[i]->alpha)
    {
      updateSkeletonGraphics = true;
    }

    if (updateSkeletonGraphics)
    {
      skeletons[i]->updateGraphics();
    }

  }

}

void AzureSkeletonTracker::parse(const nlohmann::json& json, double time, const std::string& cameraFrame)
{
  std::map<int, std::vector<HTr>> markerMap;

  for (auto& entry : json.items())
  {
    RCHECK(entry.value().size()==NUM_FRAMES);

    const int skeletonId = atoi(entry.key().c_str());
    std::vector<HTr> markers(NUM_FRAMES);
    //RLOG_CPP(1, "json: " << nlohmann::to_string(entry.value()));
    //RLOG_CPP(1, "pelvis: " << nlohmann::to_string(entry.value()["pelvis"]));

    markers[PELVIS] = parsePose(entry.value()["pelvis"]);

    markers[SPINE_NAVEL] = parsePose(entry.value()["spine_navel"]);
    markers[SPINE_CHEST] = parsePose(entry.value()["spine_chest"]);
    markers[NECK] = parsePose(entry.value()["neck"]);

    markers[CLAVICLE_LEFT] = parsePose(entry.value()["clavicle_left"]);
    markers[SHOULDER_LEFT] = parsePose(entry.value()["shoulder_left"]);
    markers[ELBOW_LEFT] = parsePose(entry.value()["elbow_left"]);
    markers[WRIST_LEFT] = parsePose(entry.value()["wrist_left"]);
    markers[HAND_LEFT] = parsePose(entry.value()["hand_left"]);
    markers[HANDTIP_LEFT] = parsePose(entry.value()["handtip_left"]);
    markers[THUMB_LEFT] = parsePose(entry.value()["thumb_left"]);

    markers[CLAVICLE_RIGHT] = parsePose(entry.value()["clavicle_right"]);
    markers[SHOULDER_RIGHT] = parsePose(entry.value()["shoulder_right"]);
    markers[ELBOW_RIGHT] = parsePose(entry.value()["elbow_right"]);
    markers[WRIST_RIGHT] = parsePose(entry.value()["wrist_right"]);
    markers[HAND_RIGHT] = parsePose(entry.value()["hand_right"]);
    markers[HANDTIP_RIGHT] = parsePose(entry.value()["handtip_right"]);
    markers[THUMB_RIGHT] = parsePose(entry.value()["thumb_right"]);

    markers[HIP_LEFT] = parsePose(entry.value()["hip_left"]);
    markers[KNEE_LEFT] = parsePose(entry.value()["knee_left"]);
    markers[ANKLE_LEFT] = parsePose(entry.value()["ankle_left"]);
    markers[FOOT_LEFT] = parsePose(entry.value()["foot_left"]);

    markers[HIP_RIGHT] = parsePose(entry.value()["hip_right"]);
    markers[KNEE_RIGHT] = parsePose(entry.value()["knee_right"]);
    markers[ANKLE_RIGHT] = parsePose(entry.value()["ankle_right"]);
    markers[FOOT_RIGHT] = parsePose(entry.value()["foot_right"]);

    markers[HEAD] = parsePose(entry.value()["head"]);
    markers[NOSE] = parsePose(entry.value()["nose"]);
    markers[EYE_LEFT] = parsePose(entry.value()["eye_left"]);
    markers[EAR_LEFT] = parsePose(entry.value()["ear_left"]);
    markers[EYE_RIGHT] = parsePose(entry.value()["eye_right"]);
    markers[EAR_RIGHT] = parsePose(entry.value()["ear_right"]);

    for (auto& marker : markers)
    {
      HTr tmp = marker;
      HTr_transform(&marker, &A_CI, &tmp);
    }


    NLOG(0, "pelvis: %.3f %.3f %.3f", markers[PELVIS].org[0], markers[PELVIS].org[1], markers[PELVIS].org[2]);

    markerMap[skeletonId] = markers;

    newAzureUpdate = true;
  }

  // Here we match the incoming markes to the closest already existing
  // skeleton. The corrMap is an index array that is set up like this:
  //
  // corrMap[0] =  28
  // corrMap[1] =  4
  // corrMap[2] =  0
  // corrMap[3] = -1
  // corrMap[4] = -1
  //
  // The running index denots the skeleton, the corrMap index is the pose id
  // of the incoming json string matching the corresponding pose.
  std::vector<int> corrMap = findCorrespondences(markerMap);

  for (size_t i=0; i<corrMap.size(); ++i)
  {
    if (corrMap[i] != -1)
    {
      skeletons[i]->markers = markerMap[corrMap[i]];
      skeletons[i]->lastUpdate = time;
    }

    NLOG(0, "corrMap[%zu] = %d", i, corrMap[i]);
  }

}

/*
    This function creates a correspondence map in the form:

    skeletonIndex  ->  bodyId
        0                 5
        1                 1
        2                 3
        3                -1
        4                -1

    It means that:
      - pose 0 is closest to incoming tracker id 5
      - pose 1 is closest to incoming tracker id 1
      - pose 2 is closest to incoming tracker id 3
      ...

 */
std::vector<int> AzureSkeletonTracker::findCorrespondences(std::map<int, std::vector<HTr>> markerMap) const
{
  std::vector<int> res(skeletons.size(), -1);   // Vector with number of skeletons entries, all being -1

  // Create pair-wise distance matrix:
  // Find correspondences based on closest distance to pelvis
  //                Pose      0     1     2     3
  // skeleton-id  frameIdx
  //        5         0      d00   d01   d02   d03
  //        1         1      d10   d11   d12   d13
  //        7         2      d20   d21   d22   d23
  MatNd* dMat = MatNd_create(markerMap.size(), skeletons.size());
  std::vector<int> frameIdVec(markerMap.size());

  unsigned int frameIdx = 0;
  for (auto it = markerMap.begin(); it != markerMap.end(); it++)
  {
    frameIdVec[frameIdx] = it->first;
    HTr pelv = it->second[PELVIS];
    const double* pelvisCurrentPos = pelv.org;

    for (size_t poseIdx = 0; poseIdx < skeletons.size(); ++poseIdx)
    {
      // For invisible skeletons, we compare against their default positions.
      const double* pelvisDefaultPos = skeletons[poseIdx]->expectedInitialPose.org;
      const double* pelvisPreviousPos = skeletons[poseIdx]->markers[PELVIS].org;
      const double* pelvisMemorizedPos = (skeletons[poseIdx]->isVisible) ? pelvisPreviousPos : pelvisDefaultPos;

      // This can also be a better distance function if needed.
      double dist = Vec3d_distance(pelvisMemorizedPos, pelvisCurrentPos);
      MatNd_set(dMat, frameIdx, poseIdx, dist);
    }

    frameIdx++;
  }

  // Go row by row and find the minimum distance.
  for (size_t i = 0; i < dMat->m; ++i)
  {
    const double* row = MatNd_getRowPtr(dMat, i);
    const int minPose = VecNd_indexMin(row, dMat->n);
    const double minDist = VecNd_minEle(row, dMat->n);

    if (minDist<this->defaultPosRadius)
    {
      res[minPose] = frameIdVec[i];
      MatNd_setColumnToValue(dMat, minPose, DBL_MAX);   // Same pose must not be considered again
    }

  }

  MatNd_destroy(dMat);

  return res;
}

bool AzureSkeletonTracker::isSkeletonVisible(size_t idx) const
{
  if (idx < skeletons.size())
  {
    return skeletons[idx]->isVisible;
  }

  RLOG_CPP(1, "Index out of range: " << idx << " (should be < " << skeletons.size() << ")");
  return false;
}

void AzureSkeletonTracker::initGraphics(const RcsGraph* graph, Rcs::Viewer* viewer)
{
  if (!viewer)
  {
    return;
  }

  static std::vector<std::string> gCol{ "RED", "ORANGE", "YELLOW", "BLUE", "GREEN",
                                        "TURQUOISE", "PEWTER", "BRONZE", "BRASS",
                                        "EMERALD", "JADE", "RUBY" };

  for (size_t i=0; i< skeletons.size(); ++i)
  {
    skeletons[i]->initGraphics(graph, viewer, gCol[i%gCol.size()]);
  }

}

void AzureSkeletonTracker::setCameraTransform(const HTr* A_camI)
{
  HTr_copy(&A_CI, A_camI);
}

void AzureSkeletonTracker::setSkeletonDefaultPosition(size_t skeletonIdx, double x, double y, double z)
{
  RCHECK(skeletonIdx<skeletons.size());
  Vec3d_set(skeletons[skeletonIdx]->expectedInitialPose.org, x, y, z);
}

void AzureSkeletonTracker::setSkeletonName(size_t skeletonIdx, const std::string& name)
{
  RCHECK(skeletonIdx<skeletons.size());
  skeletons[skeletonIdx]->agentName = name;
}

void AzureSkeletonTracker::setSkeletonDefaultPositionRadius(double r)
{
  this->defaultPosRadius = r;
}

// Map an agent (defined in the config) to a skeleton
void AzureSkeletonTracker::addAgent(const std::string& agentName)
{
  if (skeletonIndex >= skeletons.size())
  {
    RLOG(0, "ERROR: Cannot add agent! There are no free skeletons!");
    return;
  }

  if (!agentName.empty())
  {
    for (auto agent : scene->agents)
    {
      if (agent->name == agentName)
      {
        aff::HumanAgent* humanAgent = dynamic_cast<aff::HumanAgent*>(agent);

        if (humanAgent)
        {
          skeletons[skeletonIndex]->setAgent(humanAgent);

          setSkeletonDefaultPosition(skeletonIndex,
                                     humanAgent->defaultPos[0],
                                     humanAgent->defaultPos[1],
                                     humanAgent->defaultPos[2]);

          RLOG(0, "Matched agent `%s` with skeleton %zu", agentName.c_str(), skeletonIndex);

          skeletonIndex++;

          return;
        }
        else
        {
          RLOG(0, "Agent `%s` has invalid type, cannot add to skeleton!", agentName.c_str());
          return;
        }
      }
    }

    RLOG(0, "ERROR: Cannot add agent! Agent `%s` not found in scene!", agentName.c_str());
  }
  else
  {
    RLOG(0, "ERROR: Cannot add agent! Invalid agent name provided!");
  }
}

// Map ALL agents (defined in the config) to available skeletons
void AzureSkeletonTracker::addAgents()
{
  RCHECK(scene);
  for (size_t i = 0; i < scene->agents.size(); i++)
  {
    addAgent(scene->agents[i]->name);
  }
}

/* Json format for (skeleton) agents

"agent":
{
  { 1,
    {"type" : "human",
    "visible" : true / false,
    "links" : [{"link_id": 0, "position": [0, 0, 1], "euler_xyz": [1, 2, 3] },
                {"link_id": 1, "position": [0, 0, 1], "euler_xyz": [1, 2, 3] },
                ...
                {"link_id": 31, "position": [0, 0, 1], "euler_xyz": [1, 2, 3] }]
    }
  },
  { 2,
    {"type" : "human",
    "visible" : true / false,
    "links" : [{"link_id": 0, "position": [0, 0, 1], "euler_xyz": [1, 2, 3] },
                {"link_id": 1, "position": [0, 0, 1], "euler_xyz": [1, 2, 3] },
                ...
                {"link_id": 31, "position": [0, 0, 1], "euler_xyz": [1, 2, 3] }]
    }
  }

}

*/
void AzureSkeletonTracker::jsonFromSkeletons(nlohmann::json& json) const
{
  // Here we have a valid skeleton tracker
  size_t skeletonId = 0;

  for (const auto& skeleton : skeletons)
  {
    nlohmann::json skeletonJson;
    skeletonJson["skeleton_id"] = skeletonId;
    skeletonJson["types"] = skeleton->agentTypes;
    skeletonJson["name"] = skeleton->agentName;
    skeletonJson["visible"] = skeleton->isVisible ? true : false;

    size_t linkId = 0;

    for (auto& link : skeleton->markers)
    {
      double ea[3];
      Mat3d_toEulerAngles(ea, link.rot);

      nlohmann::json linkJson =
      {
        {"link_id", linkId},
        {"position", std::vector<double>(link.org, link.org+3)},
        {"euler_xyzr", std::vector<double>(ea, ea+3)}
      };
      skeletonJson["links"] += linkJson;
      linkId++;
    }   // for (const auto& link : skeleton->markers)

    //RLOG_CPP(0, "skeletonJson: '" << skeletonJson.dump() << "'");
    json["agent"][skeletonId] = skeletonJson;
    skeletonId++;
  }   // for (const auto& skeleton : st->skeletons)

}

// bool AzureSkeletonTracker::setParameter(const std::string& parameterName, void* ptr)
// {
//   if (parameterName=="DebugViewer")
//   {
//     initGraphics(graph, (Rcs::Viewer*)ptr);
//     return true;
//   }

//   return false;
// }


} // namespace aff
