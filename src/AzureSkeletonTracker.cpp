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

static bool HTr_isZero(const HTr* trf)
{
  for (int i = 0; i < 3; i++)
  {
    if (trf->org[i] != 0.0)
    {
      return false;
    }

    for (int j = 0; j < 3; j++)
    {
      if (trf->rot[i][j] != 0.0)
      {
        return false;
      }
    }
  }

  return true;
}

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

/*

"finger_angles_right": {
  "position": {
    "joint_1": -0.003625154495239258,
    "joint_2": 0,
    "joint_3": 0.2989633083343506,
    "joint_4": 0.041562557220458984,
    "joint_5": 0.15123319625854492,
    "joint_6": 0.03959965705871582,
    "joint_7": 0.28907227516174316,
    "joint_8": 0,
    "joint_9": 0.21706104278564453,
    "joint_10": 0,
    "joint_11": 0.3374612331390381,
    "joint_12": 0.13920378684997559,
    "joint_13": 0.46666693687438965,
    "joint_14": 0.46666693687438965,
    "joint_15": 0.527747631072998,
    "joint_16": 0,
    "gripper_position_right": 0
  }
}

 */


// static std::vector<double> parseFingers(const nlohmann::json& json)
// {
//   std::vector<double> finger_angles(16, 0.0);

//   for (size_t i=0; i<finger_angles.size(); ++i)
//   {
//     std::string jnt_id = "joint_" + std::to_string(i+1);
//     finger_angles[i] = json["position"][jnt_id];
//   }

//   return finger_angles;
// }

static std::vector<double> parseFingers(const nlohmann::json& json)
{

  auto it = json.find("position");
  if (it == json.end() || !it->is_object())
  {
    RLOG_CPP(1, "Warning: missing or invalid 'position' object in finger JSON");
    return std::vector<double>();
  }

  std::vector<double> finger_angles(16, 0.0);
  const auto& pos = *it;

  for (size_t i = 0; i < finger_angles.size(); ++i)
  {
    std::string jnt_id = "joint_" + std::to_string(i + 1);

    auto jt = pos.find(jnt_id);
    if (jt == pos.end())
    {
      RLOG_CPP(1, "Warning: missing finger joint '" << jnt_id);
      return std::vector<double>();
    }

    if (!jt->is_number())
    {
      RLOG_CPP(1, "Warning: finger joint '" << jnt_id << "' is present but not numeric");
      return std::vector<double>();
    }

    finger_angles[i] = jt->get<double>();
  }

  return finger_angles;
}

static std::vector<int> parse_bounding_box(const nlohmann::json& entry, const std::string& key)
{
  try
  {
    // Check if entry actually contains the key and it is an object
    if (!entry.is_object() || !entry.contains(key))
    {
      RLOG_CPP(1, "Entry missing key: " << key);
      return std::vector<int>();
    }

    const auto& sub = entry.at(key);
    if (!sub.is_object())
    {
      RLOG_CPP(1, "Value under key '" << key << "' is not an object");
      return std::vector<int>();
    }

    // Check if "bounding_box" exists and is an array
    if (!sub.contains("bounding_box"))
    {
      RLOG_CPP(1, "Missing bounding_box for key: " << key);
      return std::vector<int>();
    }

    const auto& box_array = sub.at("bounding_box");
    if (!box_array.is_array() || box_array.size() != 4)
    {
      RLOG_CPP(1, "Invalid bounding_box format for key: " << key);
      return std::vector<int>();
    }

    std::vector<int> bb_vec;
    bb_vec.reserve(4);
    for (size_t i = 0; i < 4; ++i)
    {
      if (!box_array[i].is_number_integer())
      {
        RLOG_CPP(1, "Non-integer value in bounding box at index " << i);
        return std::vector<int>();
      }
      bb_vec.push_back(box_array[i].get<int>());
    }

    return bb_vec;
  }
  catch (const std::exception& e)
  {
    RLOG_CPP(1, "Exception while parsing bounding box: " << e.what());
    return std::vector<int>();
  }
}




// static std::vector<int> parse_bounding_box(const nlohmann::json& entry, const std::string& key)
// {
//   try
//   {
//     RLOG_CPP(0, "A: key is " << key << " entry is " << entry.dump(2));

//     // Check if key exists and is structured correctly
//     if (!entry.contains(key) || !entry.at(key).contains("bounding_box"))
//     {
//       std::cerr << "Missing key or bounding box: " << key << std::endl;
//       return std::vector<int>();
//     }
//     RLOG_CPP(0, "B: key is " << key << " entry is " << entry.dump(2));

//     // if (!entry.at(key).contains("bounding_box"))
//     // {
//     //   std::cerr << "Missing bounding box: " << key << std::endl;
//     //   return std::vector<int>();
//     // }


//     const auto& box_array = entry.at(key).at("bounding_box");
//     if (!box_array.is_array() || box_array.size() != 4)
//     {
//       RLOG_CPP(1, "Invalid bounding_box format for key: " << key);
//       return std::vector<int>();
//     }

//     // Safely extract and validate all 4 integers
//     for (size_t i = 0; i < 4; ++i)
//     {
//       if (!box_array[i].is_number_integer())
//       {
//         RLOG_CPP(1, "Non-integer value in bounding box at index " << i);
//         return std::vector<int>();
//       }
//     }

//     std::vector<int> bb_vec;
//     bb_vec.push_back(box_array[0].get<int>());
//     bb_vec.push_back(box_array[1].get<int>());
//     bb_vec.push_back(box_array[2].get<int>());
//     bb_vec.push_back(box_array[3].get<int>());
//     return bb_vec;
//   }
//   catch (const std::exception& e)
//   {
//     RLOG_CPP(1, "Exception while parsing bounding box: " << e.what());
//     return std::vector<int>();
//   }
// }


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

static std::vector<std::pair<int,int>> readConnectionData(std::string trackerName="AzureKinect")
{
  std::vector<std::pair<int,int>> idx;

  if ((trackerName=="AzureKinect") || (trackerName=="BODY_34"))
  {
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
  }
  else if (trackerName=="BODY_38")
  {
    // spine (collapse deeper spine)
    idx.push_back(std::pair<int,int>(PELVIS, SPINE_NAVEL));
    idx.push_back(std::pair<int,int>(SPINE_NAVEL, SPINE_CHEST));
    idx.push_back(std::pair<int,int>(SPINE_CHEST, NECK));

    // head (no HEAD in BODY_38: use NOSE)
    idx.push_back(std::pair<int,int>(NECK, NOSE));
    idx.push_back(std::pair<int,int>(NOSE, EYE_LEFT));
    idx.push_back(std::pair<int,int>(NOSE, EYE_RIGHT));
    idx.push_back(std::pair<int,int>(EYE_LEFT, EAR_LEFT));
    idx.push_back(std::pair<int,int>(EYE_RIGHT, EAR_RIGHT));

    // right arm
    idx.push_back(std::pair<int,int>(SPINE_CHEST, CLAVICLE_RIGHT));
    idx.push_back(std::pair<int,int>(CLAVICLE_RIGHT, SHOULDER_RIGHT));
    idx.push_back(std::pair<int,int>(SHOULDER_RIGHT, ELBOW_RIGHT));
    idx.push_back(std::pair<int,int>(ELBOW_RIGHT, WRIST_RIGHT));
    idx.push_back(std::pair<int,int>(WRIST_RIGHT, HAND_RIGHT));

    // mapped fingers
    idx.push_back(std::pair<int,int>(HAND_RIGHT, HANDTIP_RIGHT)); // index_1
    idx.push_back(std::pair<int,int>(HAND_RIGHT, THUMB_RIGHT));   // thumb_4

    // left arm
    idx.push_back(std::pair<int,int>(SPINE_CHEST, CLAVICLE_LEFT));
    idx.push_back(std::pair<int,int>(CLAVICLE_LEFT, SHOULDER_LEFT));
    idx.push_back(std::pair<int,int>(SHOULDER_LEFT, ELBOW_LEFT));
    idx.push_back(std::pair<int,int>(ELBOW_LEFT, WRIST_LEFT));
    idx.push_back(std::pair<int,int>(WRIST_LEFT, HAND_LEFT));

    // mapped fingers
    idx.push_back(std::pair<int,int>(HAND_LEFT, HANDTIP_LEFT)); // index_1
    idx.push_back(std::pair<int,int>(HAND_LEFT, THUMB_LEFT));   // thumb_4

    // right leg
    idx.push_back(std::pair<int,int>(PELVIS, HIP_RIGHT));
    idx.push_back(std::pair<int,int>(HIP_RIGHT, KNEE_RIGHT));
    idx.push_back(std::pair<int,int>(KNEE_RIGHT, ANKLE_RIGHT));
    idx.push_back(std::pair<int,int>(ANKLE_RIGHT, FOOT_RIGHT)); // use BIG_TOE as proxy

    // left leg
    idx.push_back(std::pair<int,int>(PELVIS, HIP_LEFT));
    idx.push_back(std::pair<int,int>(HIP_LEFT, KNEE_LEFT));
    idx.push_back(std::pair<int,int>(KNEE_LEFT, ANKLE_LEFT));
    idx.push_back(std::pair<int,int>(ANKLE_LEFT, FOOT_LEFT)); // use BIG_TOE as proxy
  }
  else
  {
    RLOG_CPP(0, "No such connection mode: " << trackerName);
  }

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

  double lastUpdate;
  double age;
  double maxAge;
  bool wasVisible;
  bool isVisible;
  double alphaPrev;
  double alpha;
  std::vector<HTr> markers;
  std::vector<double> fingerAnglesLeft, fingerAnglesRight;
  double gripperAngleLeft, gripperAngleRight;
  HTr expectedInitialPose;
  std::string agentBdyName;

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



Skeleton::Skeleton() : lastUpdate(0.0), age(DBL_MAX), maxAge(DEFAULT_MAX_AGE),
  wasVisible(false), isVisible(false), alphaPrev(1.0), alpha(1.0), viewer(NULL),
  gripperAngleLeft(-1.0), gripperAngleRight(-1.0)
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
  agentBdyName = human->bdyName;
  visualBodies = human->manipulators;
  visualBodies.push_back(agentBdyName);
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
    for (const auto& manipulator : visualBodies)
    {
      viewer->lock();
      std::vector<osg::Node*> nodes = viewer->getNodes(manipulator);
      viewer->unlock();
      visualNodes.insert(visualNodes.end(), nodes.begin(), nodes.end());
    }
  }
  else
  {
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
AzureSkeletonTracker::AzureSkeletonTracker(size_t numSkeletons) :
  TrackerBase(""), newAzureUpdate(false), defaultPosRadius(DBL_MAX)
{
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
  if (frozen)
  {
    return;
  }

  std::map<std::string,HTr> A_camI;
  {
    std::lock_guard<std::mutex> lock(updateMtx);
    A_camI = this->cameraTransformMap;
  }

  for (auto& entry : A_camI)
  {
    const std::string& camera_name = entry.first;
    HTr& value = entry.second;
    RcsBody* cam = RcsGraph_getBodyByName(graph, camera_name.c_str());
    if (cam)
    {
      HTr_copy(&value, &cam->A_BI);
    }
  }

  {
    std::lock_guard<std::mutex> lock(updateMtx);
    this->cameraTransformMap = A_camI;
  }

  updateSkeletons(scene, graph);
  updateAgents(scene, graph);
  newAzureUpdate = false;
}

void AzureSkeletonTracker::updateAgents(ActionScene* scene, RcsGraph* graph)
{
  if (!scene)
  {
    RLOG(0, "No scene - skipping agent updates");
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
      if (skeletons[i]->agentBdyName==human->bdyName)
      {
        if (skeletons[i]->isVisible)
        {
          human->setMarkers(skeletons[i]->markers);
          human->bb_head.resize(4);
          human->bb_head[0] = skeletons[i]->bb_head.x_min;
          human->bb_head[1] = skeletons[i]->bb_head.y_min;
          human->bb_head[2] = skeletons[i]->bb_head.x_max;
          human->bb_head[3] = skeletons[i]->bb_head.y_max;
          human->fingersLeft = skeletons[i]->fingerAnglesLeft;
          human->fingersRight = skeletons[i]->fingerAnglesRight;
          human->gripperLeft = skeletons[i]->gripperAngleLeft;
          human->gripperRight = skeletons[i]->gripperAngleRight;
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
      const double tmc = 0.1;

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




      for (const auto& tf : human->trackedFrames)
      {
        // The marker transforms are represented in world coordinates.
        // In order to consider that the manipulator might have a parent
        // different to the world frame, we transform the raw percepts
        // into the (M)anipulator's (P)arent frame.
        // A_PI is the Manipulator's parent transform
        bdy = RcsGraph_getBodyByName(graph, tf.second.c_str());
        jidx = RcsBody_getJointIndex(graph, bdy);
        if (jidx==-1)
        {
          RLOG_CPP(1, "Tracked frame not found in graph: " << tf.second);
          continue;
        }

        const HTr* A_PI = (bdy->parentId == -1) ? HTr_identity() : &graph->bodies[bdy->parentId].A_BI;
        double* q_rbj = &graph->q->ele[jidx];
        HTr A_MP;   // Transform from manipulator's parent to its raw percept

        if (tf.first == HumanAgent::BodyType::Head)
        {
          HTr A_MI = human->getMarker(HEAD);   // marker transform in world
          HTr_invTransform(&A_MP, A_PI, &A_MI);
          lpFiltTrf(q_rbj, &A_MP, tmc);
          // VecNd_printComment("BodyType::Head", q_rbj, 3);
        }
        else if (tf.first == HumanAgent::BodyType::ShoulderLeft)
        {
          HTr A_MI = human->getMarker(SHOULDER_LEFT);
          HTr_invTransform(&A_MP, A_PI, &A_MI);
          lpFiltTrf(q_rbj, &A_MP, tmc);
          // VecNd_printComment("BodyType::ShoulderLeft", q_rbj, 3);
        }
        else if (tf.first == HumanAgent::BodyType::ShoulderRight)
        {
          HTr A_MI = human->getMarker(SHOULDER_RIGHT);
          HTr_invTransform(&A_MP, A_PI, &A_MI);
          lpFiltTrf(q_rbj, &A_MP, tmc);
          // VecNd_printComment("BodyType::ShoulderRight", q_rbj, 3);
        }
        else if (tf.first == HumanAgent::BodyType::ElbowLeft)
        {
          HTr A_MI = human->getMarker(ELBOW_LEFT);
          HTr_invTransform(&A_MP, A_PI, &A_MI);
          lpFiltTrf(q_rbj, &A_MP, tmc);
          // VecNd_printComment("BodyType::ElbowLeft", q_rbj, 3);
        }
        else if (tf.first == HumanAgent::BodyType::ElbowRight)
        {
          HTr A_MI = human->getMarker(ELBOW_RIGHT);
          HTr_invTransform(&A_MP, A_PI, &A_MI);
          lpFiltTrf(q_rbj, &A_MP, tmc);
          // VecNd_printComment("BodyType::ElbowRight", q_rbj, 3);
        }
        else if (tf.first == HumanAgent::BodyType::HandLeft)
        {
          HTr A_MI = human->getMarker(WRIST_LEFT);
          HTr_invTransform(&A_MP, A_PI, &A_MI);
          lpFiltTrf(q_rbj, &A_MP, tmc);
          // VecNd_printComment("BodyType::HandLeft", q_rbj, 3);
        }
        else if (tf.first == HumanAgent::BodyType::HandRight)
        {
          HTr A_MI = human->getMarker(WRIST_RIGHT);
          HTr_invTransform(&A_MP, A_PI, &A_MI);
          lpFiltTrf(q_rbj, &A_MP, tmc);
          // VecNd_printComment("BodyType::HandRight", q_rbj, 3);
        }

      }   // for (const auto& mName : human->manipulators)





    }   // if (human->hasMarkers())

  }   // for (auto& agent : scene->agents)

}

// Process aruco frames. Called from control loop (100Hz or so)
void AzureSkeletonTracker::updateSkeletons(ActionScene* scene, RcsGraph* graph)
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
      for (const auto& cb : agentAppearDisappearCb)
      {
        Agent* namedAgent = scene->getAgent(skeletons[i]->agentBdyName);
        if (!namedAgent)
        {
          RLOG_CPP(1, "Agent with name '" << skeletons[i]->agentBdyName << "' not found in scene");
          continue;
        }
        cb(namedAgent->name, true);
      }

      updateSkeletonGraphics = true;
    }
    else if (skeletons[i]->wasVisible && (!skeletons[i]->isVisible))
    {
      for (const auto& cb : agentAppearDisappearCb)
      {
        Agent* namedAgent = scene->getAgent(skeletons[i]->agentBdyName);
        if (!namedAgent)
        {
          RLOG_CPP(1, "Agent with name '" << skeletons[i]->agentBdyName << "' not found in scene");
          continue;
        }
        cb(namedAgent->name, false);
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
  if (jsonData.empty())
  {
    RLOG_CPP(1, "No data received. Header: " << jsonHeader);
    return;
  }

  std::map<std::string,HTr> A_camI;
  {
    std::lock_guard<std::mutex> lock(updateMtx);
    A_camI = this->cameraTransformMap;
  }

  std::map<int, std::vector<HTr>> markerMap;
  std::map<int, std::vector<double>> leftFingersMap, rightFingersMap;
  std::map<int, double> leftGripperMap, rightGripperMap;
  std::map<int, std::vector<int>> boundingBoxMap;

  std::string tracker = jsonHeader.value("tracker", "AzureKinect");
  std::string camera_name = jsonHeader.value("frame_id", "None");

  NLOG_CPP(0, "tracker: " << tracker << " camera_name: " << camera_name
           << " header: " << jsonHeader);


  for (auto& entry : jsonData.items())
  {
    std::vector<int> bb;
    const int skeletonId = atoi(entry.key().c_str());
    std::vector<HTr> markers(NUM_FRAMES);
    std::vector<double> q_left_fingers, q_right_fingers;
    double q_left_gripper = -1.0, q_right_gripper = -1.0;

    const nlohmann::json& pose = entry.value();

    if (tracker == "BODY_34")
    {
      static const std::pair<int, const char*> kMap34[] =
      {
        {PELVIS,"PELVIS"},
        {SPINE_NAVEL,"NAVAL_SPINE"},
        {SPINE_CHEST,"CHEST_SPINE"},
        {NECK,"NECK"},

        {CLAVICLE_LEFT,"LEFT_CLAVICLE"},
        {SHOULDER_LEFT,"LEFT_SHOULDER"},
        {ELBOW_LEFT,"LEFT_ELBOW"},
        {WRIST_LEFT,"LEFT_WRIST"},
        {HAND_LEFT,"LEFT_HAND"},
        {HANDTIP_LEFT,"XLEFT_HANDTIP"},
        {THUMB_LEFT,"LEFT_THUMB"},

        {CLAVICLE_RIGHT,"RIGHT_CLAVICLE"},
        {SHOULDER_RIGHT,"RIGHT_SHOULDER"},
        {ELBOW_RIGHT,"RIGHT_ELBOW"},
        {WRIST_RIGHT,"RIGHT_WRIST"},
        {HAND_RIGHT,"RIGHT_HAND"},
        {HANDTIP_RIGHT,"RIGHT_HANDTIP"},
        {THUMB_RIGHT,"RIGHT_THUMB"},

        {HIP_LEFT,"LEFT_HIP"},
        {KNEE_LEFT,"LEFT_KNEE"},
        {ANKLE_LEFT,"EFT_ANKLE"},
        {FOOT_LEFT,"LEFT_FOOT"},

        {HIP_RIGHT,"RIGHT_HIP"},
        {KNEE_RIGHT,"RIGHT_KNEE"},
        {ANKLE_RIGHT,"RIGHT_ANKLE"},
        {FOOT_RIGHT,"RIGHT_FOOT"},

        {HEAD,"HEAD"},
        {NOSE,"NOSE"},
        {EYE_LEFT,"LEFT_EYE"},
        {EAR_LEFT,"LEFT_EAR"},
        {EYE_RIGHT,"RIGHT_EYE"},
        {EAR_RIGHT,"RIGHT_EAR"}
      };

      for (const auto& mk : kMap34)
      {
        const int marker = mk.first;
        const char* key  = mk.second;
        auto it = pose.find(key);
        if (it != pose.end() && it->is_object())
        {
          markers[marker] = parsePose(*it);
        }
        else
        {
          RLOG_CPP(4, "Not found: " << key);
        }
      }

    }
    else if (tracker == "BODY_38")
    {
      static const std::pair<int, const char*> kMap38[] =
      {
        {PELVIS,           "PELVIS"},
        {SPINE_NAVEL,      "SPINE_1"},
        {SPINE_CHEST,      "SPINE_2"},
        {NECK,             "NECK"},

        {CLAVICLE_LEFT,    "LEFT_CLAVICLE"},
        {SHOULDER_LEFT,    "LEFT_SHOULDER"},
        {ELBOW_LEFT,       "LEFT_ELBOW"},
        {WRIST_LEFT,       "LEFT_WRIST"},
        {HAND_LEFT,        "LEFT_WRIST"},
        {HANDTIP_LEFT,     "LEFT_HAND_INDEX_1"},
        {THUMB_LEFT,       "LEFT_HAND_THUMB_4"},

        {CLAVICLE_RIGHT,   "RIGHT_CLAVICLE"},
        {SHOULDER_RIGHT,   "RIGHT_SHOULDER"},
        {ELBOW_RIGHT,      "RIGHT_ELBOW"},
        {WRIST_RIGHT,      "RIGHT_WRIST"},
        {HAND_RIGHT,      "RIGHT_WRIST"},
        {HANDTIP_RIGHT,    "RIGHT_HAND_INDEX_1"},
        {THUMB_RIGHT,      "RIGHT_HAND_THUMB_4"},

        {HIP_LEFT,         "LEFT_HIP"},
        {KNEE_LEFT,        "LEFT_KNEE"},
        {ANKLE_LEFT,       "LEFT_ANKLE"},
        {FOOT_LEFT,        "LEFT_BIG_TOE"},

        {HIP_RIGHT,        "RIGHT_HIP"},
        {KNEE_RIGHT,       "RIGHT_KNEE"},
        {ANKLE_RIGHT,      "RIGHT_ANKLE"},
        {FOOT_RIGHT,       "RIGHT_BIG_TOE"},

        {HEAD,             "NOSE"},   // still proxy
        {NOSE,             "NOSE"},
        {EYE_LEFT,         "LEFT_EYE"},
        {EAR_LEFT,         "LEFT_EAR"},
        {EYE_RIGHT,        "RIGHT_EYE"},
        {EAR_RIGHT,        "RIGHT_EAR"}
      };

      for (const auto& mk : kMap38)
      {
        const int marker = mk.first;
        const char* key  = mk.second;
        auto it = pose.find(key);
        if (it != pose.end() && it->is_object())
        {
          markers[marker] = parsePose(*it);
        }
        else
        {
          RLOG_CPP(4, "Not found: " << key);
        }
      }

    }
    else if (tracker == "XrFullBodyJointMETA")
    {
      // REXEC(1)
      // {
      //   for (auto it = pose.begin(); it != pose.end(); ++it)
      //   {
      //     std::cout << it.key() << "\n";
      //   }
      // }

      static const std::pair<int, const char*> kMap[] =
      {
        {PELVIS,"XR_FULL_BODY_JOINT_ROOT_META"},

        {SPINE_NAVEL,"XR_FULL_BODY_JOINT_HIPS_META"},
        {SPINE_CHEST,"XR_FULL_BODY_JOINT_CHEST_META"},
        {NECK,"XR_FULL_BODY_JOINT_NECK_META"},

        {CLAVICLE_LEFT,"XR_FULL_BODY_JOINT_LEFT_SHOULDER_META"},
        {SHOULDER_LEFT,"XR_FULL_BODY_JOINT_LEFT_ARM_UPPER_META"},
        {ELBOW_LEFT,"XR_FULL_BODY_JOINT_LEFT_ARM_LOWER_META"},
        // {WRIST_LEFT,"XR_FULL_BODY_JOINT_LEFT_HAND_WRIST_META"},
        {WRIST_LEFT,"XR_FULL_BODY_JOINT_LEFT_HAND_PALM_META"},
        {HAND_LEFT,"XR_FULL_BODY_JOINT_LEFT_HAND_PALM_META"},
        {HANDTIP_LEFT,"XR_FULL_BODY_JOINT_LEFT_HAND_INDEX_TIP_META"},
        {THUMB_LEFT,"XR_FULL_BODY_JOINT_LEFT_HAND_THUMB_TIP_META"},

        {CLAVICLE_RIGHT,"XR_FULL_BODY_JOINT_RIGHT_SHOULDER_META"},
        {SHOULDER_RIGHT,"XR_FULL_BODY_JOINT_RIGHT_ARM_UPPER_META"},
        {ELBOW_RIGHT,"XR_FULL_BODY_JOINT_RIGHT_ARM_LOWER_META"},
        // {WRIST_RIGHT,"XR_FULL_BODY_JOINT_RIGHT_HAND_WRIST_META"},
        {WRIST_RIGHT,"XR_FULL_BODY_JOINT_RIGHT_HAND_PALM_META"},
        {HAND_RIGHT,"XR_FULL_BODY_JOINT_RIGHT_HAND_PALM_META"},
        {HANDTIP_RIGHT,"XR_FULL_BODY_JOINT_RIGHT_HAND_INDEX_TIP_META"},
        {THUMB_RIGHT,"XR_FULL_BODY_JOINT_RIGHT_HAND_THUMB_TIP_META"},

        {HIP_LEFT,"XR_FULL_BODY_JOINT_LEFT_UPPER_LEG_META"},
        {KNEE_LEFT,"XR_FULL_BODY_JOINT_LEFT_LOWER_LEG_META"},
        {ANKLE_LEFT,"XR_FULL_BODY_JOINT_LEFT_FOOT_ANKLE_META"},
        {FOOT_LEFT,"XR_FULL_BODY_JOINT_LEFT_FOOT_BALL_META"},

        {HIP_RIGHT,"XR_FULL_BODY_JOINT_RIGHT_UPPER_LEG_META"},
        {KNEE_RIGHT,"XR_FULL_BODY_JOINT_RIGHT_LOWER_LEG_META"},
        {ANKLE_RIGHT,"XR_FULL_BODY_JOINT_RIGHT_FOOT_ANKLE_META"},
        {FOOT_RIGHT,"XR_FULL_BODY_JOINT_RIGHT_FOOT_BALL_META"},

        {HEAD,"XR_FULL_BODY_JOINT_HEAD_META"},
        {NOSE,"unknown"},
        {EYE_LEFT,"LeftEye"},
        {EAR_LEFT,"unknown"},
        {EYE_RIGHT,"RightEye"},
        {EAR_RIGHT,"unknown"}
      };


      for (const auto& mk : kMap)
      {
        const int marker = mk.first;
        const char* key  = mk.second;
        auto it = pose.find(key);
        if (it != pose.end() && it->is_object())
        {
          markers[marker] = parsePose(*it);
        }
        else
        {
          RLOG_CPP(4, "Not found: " << key);
        }
      }

      auto finger_it = pose.find("FINGER_ANGLES_LEFT");
      if (finger_it != pose.end())
      {
        auto q = parseFingers(*finger_it);
        if (!q.empty())
        {
          q_left_fingers = std::move(q);
        }
      }

      finger_it = pose.find("FINGER_ANGLES_RIGHT");
      if (finger_it != pose.end())
      {
        auto q = parseFingers(*finger_it);
        if (!q.empty())
        {
          q_right_fingers = std::move(q);
        }
      }

      auto gripper_it = pose.find("GRIPPER_ANGLE_RIGHT");
      if (gripper_it != pose.end()
          && gripper_it->contains("position")
          && (*gripper_it)["position"].is_number())
      {
        q_right_gripper = (*gripper_it)["position"].get<double>();
      }

      gripper_it = pose.find("GRIPPER_ANGLE_LEFT");
      if (gripper_it != pose.end()
          && gripper_it->contains("position")
          && (*gripper_it)["position"].is_number())
      {
        q_left_gripper = (*gripper_it)["position"].get<double>();
      }

      REXEC(5)
      {
        for (size_t i=0; i<markers.size(); ++i)
        {
          RLOG_CPP(0, "Marker [" + std::to_string(i) + "]: ");
          HTr_fprint(stderr, &markers[i]);
        }
      }


    }
    else if (tracker == "AzureKinect")
    {
      RCHECK_MSG(entry.value().size()==NUM_FRAMES, "%zu %d",
                 entry.value().size(), NUM_FRAMES);

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
      bb = parse_bounding_box(pose, "head");

      REXEC(2)
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

    }   // tracker=="AzureKinect"
    else
    {
      RLOG_CPP(0, "Unknown tracker: " << tracker);
    }


    // Will add zero HTr if not exists. In this case, we ignore this data package
    HTr A_camI_i = A_camI[camera_name];
    if (HTr_isZero(&A_camI_i))
    {
      RLOG_CPP(5, "Adding new zero-transform map entry for camera " << camera_name
               << ", jsonHeader: " << jsonHeader);
      std::lock_guard<std::mutex> lock(updateMtx);
      this->cameraTransformMap[camera_name] = {};   // Set to zero
      continue;
    }

    for (auto& marker : markers)
    {
      HTr tmp = marker;
      HTr_transform(&marker, &A_camI_i, &tmp);
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
    leftFingersMap[skeletonId] = q_left_fingers;
    rightFingersMap[skeletonId] = q_right_fingers;
    leftGripperMap[skeletonId] = q_left_gripper;
    rightGripperMap[skeletonId] = q_right_gripper;

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
  // The running index denotes the skeleton, the corrMap index is the pose id
  // of the incoming json string matching the corresponding pose.
  std::vector<int> corrMap = findCorrespondences(markerMap);

  for (size_t i=0; i<corrMap.size(); ++i)
  {
    if (corrMap[i] == -1)
    {
      continue;
    }

    skeletons[i]->lastUpdate = time;
    skeletons[i]->markers = markerMap[corrMap[i]];

    if (!leftFingersMap[corrMap[i]].empty())
    {
      skeletons[i]->fingerAnglesLeft = leftFingersMap[corrMap[i]];
    }

    if (!rightFingersMap[corrMap[i]].empty())
    {
      skeletons[i]->fingerAnglesRight = rightFingersMap[corrMap[i]];
    }

    if (leftGripperMap[corrMap[i]] >= 0.0)
    {
      skeletons[i]->gripperAngleLeft = leftGripperMap[corrMap[i]];
    }

    if (rightGripperMap[corrMap[i]] >= 0.0)
    {
      skeletons[i]->gripperAngleRight = rightGripperMap[corrMap[i]];
    }

    if (jsonHeader.contains("frame_id") && !boundingBoxMap.empty())
    {
      auto it = boundingBoxMap.find(corrMap[i]);
      if (it != boundingBoxMap.end() && (it->second.size()>=4))
      {
        skeletons[i]->bb_head.x_min = it->second[0];
        skeletons[i]->bb_head.y_min = it->second[1];
        skeletons[i]->bb_head.x_max = it->second[2];
        skeletons[i]->bb_head.y_max = it->second[3];
        skeletons[i]->bb_head.camera = jsonHeader["frame_id"];
      }
    }

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
      // double dist = Vec3d_distance(pelvisMemorizedPos, pelvisCurrentPos);

      // This is the 2d projected distance, ignoring pelvis height
      double dist = sqrt(VecNd_sqrDiff(pelvisMemorizedPos, pelvisCurrentPos, 2));
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

void AzureSkeletonTracker::setSkeletonDefaultPositionRadius(double r)
{
  this->defaultPosRadius = r;
}

void AzureSkeletonTracker::addAgents(const ActionScene* scene)
{
  RCHECK(scene);
  auto humanAgents = scene->getAgents<HumanAgent>();
  RCHECK_MSG(skeletonIndex + humanAgents.size() <= skeletons.size(), "%zu + %zu < %zu",
             skeletonIndex, humanAgents.size(), skeletons.size());

  for (const auto& humanAgent : humanAgents)
  {
    skeletons[skeletonIndex]->setAgent(humanAgent);
    setSkeletonDefaultPosition(skeletonIndex,
                               humanAgent->getDefaultPosition(0),
                               humanAgent->getDefaultPosition(1),
                               humanAgent->getDefaultPosition(2));
    this->skeletonIndex++;
  }

}

void AzureSkeletonTracker::registerAgentAppearDisappearCallback(std::function<void(const std::string& agentName, bool appear)> callback)
{
  agentAppearDisappearCb.push_back(callback);
}

} // namespace aff
