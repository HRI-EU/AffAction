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

/*

You can subscribe to the topic:
"holo/human_hands_poses"
which is a PoseArrayMsg data type.

The second element in the array is the right hand's position and orientation (quotation)


 */
#ifndef AFF_HOLOLENSCONNECTION_H
#define AFF_HOLOLENSCONNECTION_H

#include <ComponentBase.h>
#include <Rcs_macros.h>
#include <Rcs_body.h>
#include <Rcs_quaternion.h>
#include <Rcs_typedef.h>
#include <Rcs_utilsCPP.h>
#include <TextNode3D.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>


namespace aff
{

class HololensConnection : public ComponentBase
{

  // Gets called by incoming topic message /holo/human_hands_poses through ROS
  void humanHandsCallback(const geometry_msgs::PoseArray::ConstPtr& objArray)
  {
    static int count = 0;
    std::vector<std::string> objNames = Rcs::String_split(objArray->header.frame_id, ",");
    std::map<std::string, HTr> tmp;

    for (size_t i=0; i<objNames.size(); ++i)
    {
      RLOG_CPP(1, "objNames[" << i << "]: " << objNames[i]);
    }

    RCHECK_MSG(objNames.size()==objArray->poses.size(),
               "%zu %zu", objNames.size(), objArray->poses.size());

    for (size_t i=0; i<objArray->poses.size(); ++i)
    {
      HTr holoTrf;
      holoTrf.org[0] = objArray->poses[i].position.x;
      holoTrf.org[1] = objArray->poses[i].position.y;
      holoTrf.org[2] = objArray->poses[i].position.z;

      double quat[4];
      quat[0] = objArray->poses[i].orientation.w;
      quat[1] = objArray->poses[i].orientation.x;
      quat[2] = objArray->poses[i].orientation.y;
      quat[3] = objArray->poses[i].orientation.z;
      Quat_toRotationMatrix(holoTrf.rot, quat);

      if (count==300 && withTextLabels)
      {
        osg::ref_ptr<osg::Node> tNd = new Rcs::TextNode3D(objNames[i]);
        getEntity()->publish("AddChildNode", tNd, std::string("IK"), objNames[i]);
        RLOG(1, "Publishing text node for %s", objNames[i].c_str());
      }

      tmp[objNames[i]] = holoTrf;
    }

    count++;

    std::lock_guard<std::mutex> lock(rosMtx);
    transformsFromHolo = tmp;
  }

  void onPostUpdateGraph(RcsGraph* desired, RcsGraph* current)
  {
    // Apply the transforms from the Hololens to the graph
    RcsGraph* graph = desired;
    auto holoTrfs = getUnityTransforms();

    for (std::pair<std::string, HTr> element : holoTrfs)
    {
      const RcsBody* bdy = RcsGraph_getBodyByName(graph, element.first.c_str());

      if (RcsBody_isFloatingBase(graph, bdy))
      {
        int jidx = graph->joints[bdy->jntId].jointIndex;
        double* q_rb = &graph->q->ele[jidx];
        HTr_to6DVector(q_rb, &element.second);
        RLOG_CPP(1, "Applying transform for " << element.first);
      }
      else
      {
        RLOG_CPP(1, "Failed to apply transform of body " << element.first);
      }

    }

  }

  void onStart()
  {
    RLOG_CPP(0, "HololensConnection::onStart()");

    if (!nh)
    {
      RLOG(0, "Creating node handle");
      nh = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
    }

    this->humanHandsSubscriber = nh->subscribe("/holo/human_hands_poses", 1, &HololensConnection::humanHandsCallback, this);
    RLOG(0, "Done subscribing");
  }

  void onStop()
  {
    RLOG(0, "HololensConnection::onStop()");
    if (nh)
    {
      this->humanHandsSubscriber.shutdown();
    }

  }


  ros::Subscriber humanHandsSubscriber;
  std::unique_ptr<ros::NodeHandle> nh;
  std::map<std::string, HTr> transformsFromHolo;
  mutable std::mutex rosMtx;
  bool withTextLabels;

public:

  HololensConnection(EntityBase* parent, bool withTextLabels_) :
    ComponentBase(parent), withTextLabels(withTextLabels_)
  {
    // We start the thread right away so that the initializations from Unity
    // can be done before the graph is built.
    onStart();

    subscribe("Stop", &HololensConnection::onStop);
    subscribe("PostUpdateGraph", &HololensConnection::onPostUpdateGraph);
  }

  std::map<std::string, HTr> getUnityTransforms() const
  {
    std::lock_guard<std::mutex> lock(rosMtx);
    return transformsFromHolo;
  }

  std::map<std::string, HTr> getAndClearUnityTransforms()
  {
    std::lock_guard<std::mutex> lock(rosMtx);
    auto res = transformsFromHolo;
    transformsFromHolo.clear();
    return res;
  }

};

}   // namespace

#endif   // AFF_HOLOLENSCONNECTION_H
