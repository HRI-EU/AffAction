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

static std::vector<int> parse_bounding_box(const nlohmann::json& entry, const std::string& key)
{
  try
  {
    // Check if key exists and is structured correctly
    if (!entry.contains(key))
    {
      std::cerr << "Missing key: " << key << std::endl;
      return std::vector<int>();
    }

    const auto& box_array = entry.at(key).at("bounding_box");
    if (!box_array.is_array() || box_array.size() != 4)
    {
      RLOG_CPP(1, "Invalid bounding_box format for key: " << key);
      return std::vector<int>();
    }

    // Safely extract and validate all 4 integers
    for (size_t i = 0; i < 4; ++i)
    {
      if (!box_array[i].is_number_integer())
      {
        RLOG_CPP(1, "Non-integer value in bounding box at index " << i);
        return std::vector<int>();
      }
    }

    std::vector<int> bb_vec;
    bb_vec.push_back(box_array[0].get<int>());
    bb_vec.push_back(box_array[1].get<int>());
    bb_vec.push_back(box_array[2].get<int>());
    bb_vec.push_back(box_array[3].get<int>());
    return bb_vec;
  }
  catch (const std::exception& e)
  {
    RLOG_CPP(1, "Exception while parsing bounding box: " << e.what());
    return std::vector<int>();
  }
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


static std::string getLinkNameById(int id)
{
  switch (id)
  {
    case PELVIS:
      return "PELVIS";
    case SPINE_NAVEL:
      return "SPINE_NAVEL";
    case SPINE_CHEST:
      return "SPINE_CHEST";
    case NECK:
      return "NECK";
    case CLAVICLE_LEFT:
      return "CLAVICLE_LEFT";
    case SHOULDER_LEFT:
      return "SHOULDER_LEFT";
    case ELBOW_LEFT:
      return "ELBOW_LEFT";
    case WRIST_LEFT:
      return "WRIST_LEFT";
    case HAND_LEFT:
      return "HAND_LEFT";
    case HANDTIP_LEFT:
      return "HANDTIP_LEFT";
    case THUMB_LEFT:
      return "THUMB_LEFT";
    case CLAVICLE_RIGHT:
      return "CLAVICLE_RIGHT";
    case SHOULDER_RIGHT:
      return "SHOULDER_RIGHT";
    case ELBOW_RIGHT:
      return "ELBOW_RIGHT";
    case WRIST_RIGHT:
      return "WRIST_RIGHT";
    case HAND_RIGHT:
      return "HAND_RIGHT";
    case HANDTIP_RIGHT:
      return "HANDTIP_RIGHT";
    case THUMB_RIGHT:
      return "THUMB_RIGHT";
    case HIP_LEFT:
      return "HIP_LEFT";
    case KNEE_LEFT:
      return "KNEE_LEFT";
    case ANKLE_LEFT:
      return "ANKLE_LEFT";
    case FOOT_LEFT:
      return "FOOT_LEFT";
    case HIP_RIGHT:
      return "HIP_RIGHT";
    case KNEE_RIGHT:
      return "KNEE_RIGHT";
    case ANKLE_RIGHT:
      return "ANKLE_RIGHT";
    case FOOT_RIGHT:
      return "FOOT_RIGHT";
    case HEAD:
      return "HEAD";
    case NOSE:
      return "NOSE";
    case EYE_LEFT:
      return "EYE_LEFT";
    case EAR_LEFT:
      return "EAR_LEFT";
    case EYE_RIGHT:
      return "EYE_RIGHT";
    case EAR_RIGHT:
      return "EAR_RIGHT";
    default:
      return "UNKNOWN";
  }
}

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

/*******************************************************************************
 *
 *******************************************************************************/
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

  // That is the bounding box:
  // x_min: Left edge of the box
  // y_min: Top edge of the box
  // x_max: Right edge
  // y_max: Bottom edge
  struct BoundingBox
  {
    int x_min, y_min, x_max, y_max;
    std::string camera;
  };

  BoundingBox bb_head;

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
    RLOG_CPP(5, "Skeleton " << agentName << " has " << visualBodies.size() << " bodies");
    for (const auto& manipulator : visualBodies)
    {
      viewer->lock();
      std::vector<osg::Node*> nodes = viewer->getNodes(manipulator);
      viewer->unlock();
      visualNodes.insert(visualNodes.end(), nodes.begin(), nodes.end());
    }
    RLOG_CPP(5, "Skeleton " << agentName << " has " << visualNodes.size() << " nodes");
  }
  else
  {
    RLOG_CPP(5, "Skeleton " << agentName << " has " << visualNodes.size() << " osg nodes and alpha " << alpha);
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












/*******************************************************************************
 *
 *******************************************************************************/
AzureSkeletonTracker::AzureSkeletonTracker(size_t numSkeletons, const std::string& camera) :
  TrackerBase(camera), newAzureUpdate(false), defaultPosRadius(DBL_MAX)
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

void AzureSkeletonTracker::update(ActionScene* scene, RcsGraph* graph)
{
  {
    std::lock_guard<std::mutex> lock(updateMtx);
    this->A_CI = getCameraTransform(graph);
  }

  updateSkeletons(graph);
  updateAgents(scene, graph);
  newAzureUpdate = false;
}

void AzureSkeletonTracker::updateAgents(ActionScene* scene, RcsGraph* graph)
{
  if (!scene)
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

    //if (!human->bb_head.empty())
    //{
    //  RLOG_CPP(0, "Human agent: " << human->name << " bb: ");
    //  for (const auto& bb : human->bb_head)
    //  {
    //    std::cout << bb << " ";
    //  }
    //  std::cout << std::endl;
    //}

    for (size_t i=0; i< skeletons.size(); ++i)
    {
      if (skeletons[i]->agentName==human->name)
      {
        if (skeletons[i]->isVisible)
        {
          human->setMarkers(skeletons[i]->markers);
          human->bb_head.resize(4);
          human->bb_head[0] = skeletons[i]->bb_head.x_min;
          human->bb_head[1] = skeletons[i]->bb_head.y_min;
          human->bb_head[2] = skeletons[i]->bb_head.x_max;
          human->bb_head[3] = skeletons[i]->bb_head.y_max;
        }
        else
        {
          human->bb_head.clear();
        }

        human->setVisibility(skeletons[i]->isVisible);
        human->setLastTimeSeen(skeletons[i]->age);
      }

    }

    if (human->hasMarkers())
    {
      const double tmc = 0.05;

      // Transform pelvis
      const RcsBody* bdy = human->body(graph);
      int jidx = RcsBody_getJointIndex(graph, bdy);
      if (jidx!=-1)
      {
        const HTr* A_PI = (bdy->parentId == -1) ? HTr_identity() : &graph->bodies[bdy->parentId].A_BI;
        HTr A_MI = human->getMarker(PELVIS);   // marker transform in world
        HTr A_MP;   // Transform from pelvis's parent to its raw percept
        HTr_invTransform(&A_MP, A_PI, &A_MI);
        lpFiltTrf(&graph->q->ele[jidx], &A_MP, tmc);
      }

      for (const auto& mName : human->manipulators)
      {
        // The marker transforms are represented in world coordinates.
        // In order to consider that the manipulator might have a parent
        // different to the world frame, we transform the raw percepts
        // into the (M)anipulator's (P)arent frame.
        // A_PI is the Manipulator's parent transform
        const aff::Manipulator* m = scene->getManipulator(mName);
        RCHECK_MSG(m, "Manipulator '%s' not found", mName.c_str());
        bdy = m->body(graph);
        jidx = RcsBody_getJointIndex(graph, bdy);
        if (jidx==-1)
        {
          continue;
        }
        const HTr* A_PI = (bdy->parentId == -1) ? HTr_identity() : &graph->bodies[bdy->parentId].A_BI;
        double* q_rbj = &graph->q->ele[jidx];
        HTr A_MP;   // Transform from manipulator's parent to its raw percept

        if (m->isOfType("head"))
        {
          HTr A_MI = human->getMarker(HEAD);   // marker transform in world
          HTr_invTransform(&A_MP, A_PI, &A_MI);
          lpFiltTrf(q_rbj, &A_MP, tmc);
        }
        else if (m->isOfType("hand_left"))
        {
          HTr A_MI = human->getMarker(HANDTIP_LEFT);
          HTr_invTransform(&A_MP, A_PI, &A_MI);
          lpFiltTrf(q_rbj, &A_MP, tmc);
        }
        else if (m->isOfType("hand_right"))
        {
          HTr A_MI = human->getMarker(HANDTIP_RIGHT);
          HTr_invTransform(&A_MP, A_PI, &A_MI);
          lpFiltTrf(q_rbj, &A_MP, tmc);
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

    NLOG_CPP(4, "Skeleton[" << i << "]: lastupdate: " << skeletons[i]->lastUpdate
             << " current time: " << currTime << " age: " << age);

    bool updateSkeletonGraphics = newAzureUpdate;

    skeletons[i]->wasVisible = skeletons[i]->isVisible;
    skeletons[i]->isVisible = (age<=skeletons[i]->maxAge) ? true : false;
    skeletons[i]->age = age;

    // age = 0: alpha=1   age=maxAge: alpha=0
    skeletons[i]->alphaPrev = skeletons[i]->alpha;
    skeletons[i]->alpha = Math_clip((skeletons[i]->maxAge - skeletons[i]->age)/skeletons[i]->maxAge, 0.0, 1.0);
    skeletons[i]->alpha = lround(skeletons[i]->alpha*100.0)/100.0;
    if (skeletons[i]->alpha>0.6)
    {
      skeletons[i]->alpha = 1.0;
    }


    if ((!skeletons[i]->wasVisible) && skeletons[i]->isVisible)
    {
      NLOG_CPP(0, "Skeleton " << skeletons[i]->agentName << " (index " << i << ")" << " appeared");
      for (const auto& cb : agentAppearDisappearCb)
      {
        cb(skeletons[i]->agentName, true);
      }

      updateSkeletonGraphics = true;
    }
    else if (skeletons[i]->wasVisible && (!skeletons[i]->isVisible))
    {
      NLOG_CPP(0, "Skeleton " << skeletons[i]->agentName << " (index " << i << ")" << " disappeared");
      for (const auto& cb : agentAppearDisappearCb)
      {
        cb(skeletons[i]->agentName, false);
      }
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

void AzureSkeletonTracker::parse(const nlohmann::json& jsonHeader, const nlohmann::json& jsonData, double time)
{
  HTr A_camI;
  {
    std::lock_guard<std::mutex> lock(updateMtx);
    A_camI = this->A_CI;
  }

  std::map<int, std::vector<HTr>> markerMap;
  std::map<int, std::vector<int>> boundingBoxMap;

  for (auto& entry : jsonData.items())
  {
    RCHECK(entry.value().size()==NUM_FRAMES);

    const int skeletonId = atoi(entry.key().c_str());
    std::vector<HTr> markers(NUM_FRAMES);
    RLOG_CPP(5, "json: " << nlohmann::to_string(entry.value()));

    const nlohmann::json& pose = entry.value();
    markers[PELVIS] = parsePose(pose["pelvis"]);

    markers[SPINE_NAVEL] = parsePose(pose["spine_navel"]);
    markers[SPINE_CHEST] = parsePose(pose["spine_chest"]);
    markers[NECK] = parsePose(pose["neck"]);

    markers[CLAVICLE_LEFT] = parsePose(pose["clavicle_left"]);
    markers[SHOULDER_LEFT] = parsePose(pose["shoulder_left"]);
    markers[ELBOW_LEFT] = parsePose(pose["elbow_left"]);
    markers[WRIST_LEFT] = parsePose(pose["wrist_left"]);
    markers[HAND_LEFT] = parsePose(pose["hand_left"]);
    markers[HANDTIP_LEFT] = parsePose(pose["handtip_left"]);
    markers[THUMB_LEFT] = parsePose(pose["thumb_left"]);

    markers[CLAVICLE_RIGHT] = parsePose(pose["clavicle_right"]);
    markers[SHOULDER_RIGHT] = parsePose(pose["shoulder_right"]);
    markers[ELBOW_RIGHT] = parsePose(pose["elbow_right"]);
    markers[WRIST_RIGHT] = parsePose(pose["wrist_right"]);
    markers[HAND_RIGHT] = parsePose(pose["hand_right"]);
    markers[HANDTIP_RIGHT] = parsePose(pose["handtip_right"]);
    markers[THUMB_RIGHT] = parsePose(pose["thumb_right"]);

    markers[HIP_LEFT] = parsePose(pose["hip_left"]);
    markers[KNEE_LEFT] = parsePose(pose["knee_left"]);
    markers[ANKLE_LEFT] = parsePose(pose["ankle_left"]);
    markers[FOOT_LEFT] = parsePose(pose["foot_left"]);

    markers[HIP_RIGHT] = parsePose(pose["hip_right"]);
    markers[KNEE_RIGHT] = parsePose(pose["knee_right"]);
    markers[ANKLE_RIGHT] = parsePose(pose["ankle_right"]);
    markers[FOOT_RIGHT] = parsePose(pose["foot_right"]);

    markers[HEAD] = parsePose(pose["head"]);
    markers[NOSE] = parsePose(pose["nose"]);
    markers[EYE_LEFT] = parsePose(pose["eye_left"]);
    markers[EAR_LEFT] = parsePose(pose["ear_left"]);
    markers[EYE_RIGHT] = parsePose(pose["eye_right"]);
    markers[EAR_RIGHT] = parsePose(pose["ear_right"]);

    // Bounding boxes
    //"head": {
    //    "bounding_box": [
    //        477,
    //            86,
    //            520,
    //            520
    //    ] , ...
    //}
    std::vector<int> bb = parse_bounding_box(pose, "head");

    REXEC(1)
    {
      if (!bb.empty())
      {
        RLOG_CPP(0, "bb: ");
        for (const auto& bbi : bb)
        {
          std::cout << bbi << " ";
        }
      }
    }

    for (auto& marker : markers)
    {
      HTr tmp = marker;
      HTr_transform(&marker, &A_camI, &tmp);
    }

    REXEC(5)
    {
      for (size_t i=0; i<markers.size(); ++i)
      {
        RLOG_CPP(0, "Marker " + getLinkNameById(i) + "[" + std::to_string(i) + "]: ");
        HTr_fprint(stderr, &markers[i]);
      }
    }

    markerMap[skeletonId] = markers;
    boundingBoxMap[skeletonId] = bb;

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
      skeletons[i]->lastUpdate = time;
      skeletons[i]->markers = markerMap[corrMap[i]];

      if (jsonHeader.contains("frame_id"))
      {
        skeletons[i]->bb_head.x_min = boundingBoxMap[corrMap[i]][0];
        skeletons[i]->bb_head.y_min = boundingBoxMap[corrMap[i]][1];
        skeletons[i]->bb_head.x_max = boundingBoxMap[corrMap[i]][2];
        skeletons[i]->bb_head.y_max = boundingBoxMap[corrMap[i]][3];
        skeletons[i]->bb_head.camera = jsonHeader["frame_id"];
      }

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

bool AzureSkeletonTracker::initDebugGraphics(Rcs::Viewer* viewer, const RcsGraph* graph)
{
  if (!viewer)
  {
    return false;
  }

  static std::vector<std::string> gCol{ "RED", "ORANGE", "YELLOW", "BLUE", "GREEN",
                                        "TURQUOISE", "PEWTER", "BRONZE", "BRASS",
                                        "EMERALD", "JADE", "RUBY" };

  for (size_t i=0; i< skeletons.size(); ++i)
  {
    skeletons[i]->initGraphics(graph, viewer, gCol[i%gCol.size()]);
  }

  return true;
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
void AzureSkeletonTracker::addAgent(const ActionScene* scene, const std::string& agentName)
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
                                     humanAgent->getDefaultPosition(0),
                                     humanAgent->getDefaultPosition(1),
                                     humanAgent->getDefaultPosition(2));

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
void AzureSkeletonTracker::addAgents(const ActionScene* scene)
{
  RCHECK(scene);
  for (size_t i = 0; i < scene->agents.size(); i++)
  {
    addAgent(scene, scene->agents[i]->name);
  }
}

void AzureSkeletonTracker::registerAgentAppearDisappearCallback(std::function<void(const std::string& agentName, bool appear)> callback)
{
  agentAppearDisappearCb.push_back(callback);
}

} // namespace aff
