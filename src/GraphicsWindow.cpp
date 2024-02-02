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

#include "GraphicsWindow.h"
#include "EntityBase.h"

#include <Rcs_macros.h>
#include <Rcs_typedef.h>
#include <Rcs_utils.h>
#include <Rcs_Vec3d.h>
#include <Rcs_graphicsUtils.h>

#include <thread>
#include <iostream>
#include <algorithm>




namespace aff
{

class MapItem : public Rcs::GraphNode
{
public:

  virtual ~MapItem()
  {
    RcsGraph_destroy(this->graph);

    // Clean up mutex.
    if (realizeMtx.try_lock())
    {
      this->realizeMtx.unlock();
    }
  }

  bool realized() const
  {
    if (realizeMtx.try_lock() == true)
    {
      this->realizeMtx.unlock();
      return true;
    }

    return false;
  }

  void copyState(const RcsGraph* other, bool resizeable)
  {
    if (!realized())
    {
      return;
    }

    std::lock_guard<std::mutex> lock(stateCopyingMtx);

    // \todo: This might have concurrency issues. We better copy into a class
    //        internal buffer, than directly into the internal graph.
    RcsGraph_copy(graph, other);
  }

  void pasteState()
  {
  }

  static void realizeNodeInThread(GraphicsWindow* window, std::string eventName,
                                  const RcsGraph* other, bool resizeable, EntityBase* ntt)
  {
    std::thread t1(&MapItem::constructNode, window, eventName, other,
                   resizeable, ntt);
    t1.detach();
  }

  static osg::ref_ptr<MapItem> getEntry(std::string eventName)
  {
    osg::ref_ptr<MapItem> mi;
    std::lock_guard<std::mutex> lock(mapMtx);

    auto nd = MapItem::eventMap.find(eventName);

    if (nd != eventMap.end())
    {
      mi = nd->second;
    }

    return mi;
  }

  static osg::ref_ptr<MapItem> eraseEntry(std::string eventName)
  {
    osg::ref_ptr<MapItem> mi;
    std::lock_guard<std::mutex> lock(mapMtx);

    auto nd = MapItem::eventMap.find(eventName);

    if (nd != eventMap.end())
    {
      mi = nd->second;
      MapItem::eventMap.erase(nd);
    }

    return mi;
  }



  // This function is atomic in a way that the event will added to the map in a
  // thread-safe way before returning.
  static osg::ref_ptr<MapItem> getOrCreateEntry(std::string eventName,
                                                bool& exists)
  {
    std::lock_guard<std::mutex> lock(mapMtx);
    auto nd = MapItem::eventMap.find(eventName);
    osg::ref_ptr<MapItem> mi;

    if (nd == eventMap.end())
    {
      exists = false;
      mi = new MapItem();
      MapItem::eventMap[eventName] = mi;
    }
    else
    {
      exists = true;
      mi = nd->second;
    }

    return mi;
  }

  static void clear(Rcs::Viewer* viewer)
  {
    std::map<std::string,osg::ref_ptr<MapItem>> cpyOfMap;

    {
      std::lock_guard<std::mutex> lock(mapMtx);
      cpyOfMap = MapItem::eventMap;
      eventMap.clear();
    }

    for (auto it = cpyOfMap.begin(); it != cpyOfMap.end(); ++it)
    {
      osg::ref_ptr<MapItem> mi = it->second;

      if (mi->realized())
      {
        viewer->removeNode(mi);
      }
    }

  }

  static void clearAllNodesInThread(GraphicsWindow* window)
  {
    std::thread t1(&MapItem::clear, window);
    t1.detach();
  }

  static void print()
  {
    int count = 0;

    std::lock_guard<std::mutex> lock(mapMtx);
    if (eventMap.empty())
    {
      RLOG(0, "No RenderGraph events");
    }
    else
    {
      for (auto it = eventMap.begin(); it != eventMap.end(); ++it)
      {
        RLOG(0, "RenderGraph(%d) = %s", count++, it->first.c_str());
      }
    }
  }

  static std::map<std::string,osg::ref_ptr<MapItem>> getEventMap()
  {
    std::lock_guard<std::mutex> lock(mapMtx);
    std::map<std::string,osg::ref_ptr<MapItem>> cpyOfMap = MapItem::eventMap;

    return cpyOfMap;
  }

  static void setBodyActivation(std::string bodyName, bool enable)
  {
    RLOG(1, "Setting body activation of %s to %s",
         bodyName.c_str(), enable ? "TRUE" : "FALSE");


    std::lock_guard<std::mutex> lock(mapMtx);

    auto it = std::find(deactivatedBodies.begin(), deactivatedBodies.end(),
                        bodyName);

    if (enable == false)   // Disable
    {
      if (it == deactivatedBodies.end())
      {
        deactivatedBodies.push_back(bodyName);
      }
    }
    else   // Enable
    {
      if (it != deactivatedBodies.end())
      {
        deactivatedBodies.erase(it);
      }
    }

  }

  RcsGraph* getGraph() const
  {
    return this->graph;
  }

private:

  MapItem() : Rcs::GraphNode(), graph(NULL)
  {
    this->realizeMtx.lock();
  }

  static void constructNode(GraphicsWindow* window, std::string graphId,
                            const RcsGraph* other, bool resizeable, EntityBase* ntt)
  {
    RLOG(5, "Constructing node for graph with id \"%s\"", graphId.c_str());

    // The entry has been created before, therefore getEntry() always returns a valid entry.
    osg::ref_ptr<MapItem> mi = getEntry(graphId);
    RCHECK(mi.valid());
    mi->graph = RcsGraph_clone(other);

    bool success = mi->init(mi->graph, resizeable, false);

    if (success==false)
    {
      RLOG(4, "Failed to initialize GraphNode for %s", graphId.c_str());
    }

    if (resizeable)
    {
      mi->setDynamicMeshUpdate(true);
    }
    window->add(mi);


    {
      std::lock_guard<std::mutex> lock(mapMtx);

      //mapMtx.lock();
      for (size_t i = 0; i < deactivatedBodies.size(); ++i)
      {
        Rcs::BodyNode* bnd = mi->getBodyNode(deactivatedBodies[i].c_str());

        if (bnd != NULL)
        {
          RLOG(5, "Hiding deactivated body %s", deactivatedBodies[i].c_str());
          bnd->setVisibility(false);
        }
      }
      //mapMtx.unlock();
    }

    mi->realizeMtx.unlock();

    ntt->publish("GraphicsWindowFeedback", std::string("GraphNodeCreated"), graphId);
  }

  MapItem(const MapItem&);
  MapItem& operator=(const MapItem&);

  mutable std::mutex realizeMtx;
  mutable std::mutex stateCopyingMtx;
  RcsGraph* graph;

  static std::map<std::string,osg::ref_ptr<MapItem>> eventMap;
  static std::vector<std::string> deactivatedBodies;
  static std::mutex mapMtx;
};


std::map<std::string,osg::ref_ptr<MapItem>> MapItem::eventMap;
std::mutex MapItem::mapMtx;
std::vector<std::string> MapItem::deactivatedBodies;

} // namespace

























namespace aff
{

GraphicsWindow::GraphicsWindow(EntityBase* parent, bool startWithStartEvent,
                               bool synWithEventLoop_, bool simpleGraphics) :
  aff::ComponentBase(parent),
  Rcs::Viewer(!simpleGraphics, !simpleGraphics),
  synWithEventLoop(synWithEventLoop_),
  resizeable(false)
{
  pthread_mutex_init(&frameMtx, NULL);

#if defined(_MSC_VER)
  setWindowSize(12, 36, 640, 480);
#else
  //setWindowSize(0, 0, 1280, 960);
  setWindowSize(0, 0, 640, 480);
#endif

  setCameraTransform(4.105175, 3.439, 2.574,   0.233, -0.286, -2.28);

  this->keyCatcher = new Rcs::KeyCatcher();
  add(keyCatcher.get());
  this->hud = new Rcs::HUD();
  add(hud.get());
  this->vertexNode = new Rcs::VertexArrayNode();
  //vertexNode->hide();
  add(vertexNode.get());

  subscribeAll(startWithStartEvent);

  //if (startWithStartEvent==false)
  //{
  //  start();
  //}
}

void GraphicsWindow::subscribeAll(bool startWithStartEvent)
{
  if (startWithStartEvent)
  {
    subscribe("Start", &GraphicsWindow::start);
    subscribe("Stop", &GraphicsWindow::stop);
  }
  subscribe("ReloadGraph", &GraphicsWindow::onReloadGraph);
  subscribe("RenderGraph", &GraphicsWindow::onRender);
  subscribe("RenderLines", &GraphicsWindow::onRenderLines);
  subscribe("RenderCommand", &GraphicsWindow::onRenderCommand);
  subscribe("RenderClear", &GraphicsWindow::clear);
  subscribe("Print", &GraphicsWindow::print);
  subscribe("SetText", &GraphicsWindow::setText);
  subscribe("SetTextLine", &GraphicsWindow::setTextLine);
  subscribe("AddText", &GraphicsWindow::addText);
  subscribe("ClearText", &GraphicsWindow::clearText);
  subscribe("SetObjectActivation", &GraphicsWindow::onObjectActivation);
  subscribe("EmergencyStop", &GraphicsWindow::onEmergencyStop);
  subscribe("EmergencyRecover", &GraphicsWindow::onEmergencyRecover);
  subscribe("AddNode", &GraphicsWindow::onAddNode);
  subscribe("AddChildNode", &GraphicsWindow::onAddChildNode);
  subscribe("RemoveNode", &GraphicsWindow::onRemoveNode);
  subscribe("SetObjectColor", &GraphicsWindow::onObjectColor);
  subscribe("SetObjectAlpha", &GraphicsWindow::onObjectAlpha);
  subscribe("SetObjectsAlpha", &GraphicsWindow::onObjectsAlpha);
  subscribe("SetNodeTransform", &GraphicsWindow::onSetNodeTransform);

  if (this->synWithEventLoop)
  {
    subscribe("Render", &GraphicsWindow::frame);
  }

}

GraphicsWindow::~GraphicsWindow()
{
  stop();
  MapItem::clear(this);
  pthread_mutex_destroy(&frameMtx);
}

void GraphicsWindow::start()
{
  // If no viewer has been launched before the start event is called, we call
  // the Viewer's create() function to initialize the graphics window etc.
  if (!viewer.valid())
  {
    create(true, true);
  }

  // Only in case the viewer runs in its own thread, we launch up the thread
  // function here.
  if (!this->synWithEventLoop)
  {
    runInThread(&frameMtx);
  }
}

void GraphicsWindow::stop()
{
  if (!isThreadRunning())
  {
    RLOG(5, "GraphicsWindow::stop(): Viewer thread not running - nothing to do");
    return;
  }

  RLOG(5, "Stop::stop()");
  stopUpdateThread();   // Blocks until thread has been joined
}

void GraphicsWindow::clear()
{
  MapItem::clearAllNodesInThread(this);
}

const RcsGraph* GraphicsWindow::getGraphById(std::string graphId)
{
  osg::ref_ptr<MapItem> mi = MapItem::getEntry(graphId);

  if (mi && mi->realized())
  {
    return mi->getGraphPtr();
  }

  return NULL;
}

Rcs::GraphNode* GraphicsWindow::getGraphNodeById(std::string graphId, bool& notRealized)
{
  notRealized = false;
  osg::ref_ptr<MapItem> mi = MapItem::getEntry(graphId);

  if (mi == NULL)
  {
    RLOG(5, "GraphNode \"%s\" not found", graphId.c_str());
    return NULL;
  }

  if (!mi->realized())
  {
    RLOG(5, "GraphNode \"%s\" not realized", graphId.c_str());
    notRealized = true;
    return NULL;
  }

  return mi;
}

std::string GraphicsWindow::getName() const
{
  return std::string("GraphicsWindow");
}

void GraphicsWindow::onAddNode(osg::ref_ptr<osg::Node> node)
{
  if (!node.valid())
  {
    RLOG(0, "Can't add invalid osg::Node - skipping");
    return;
  }

  if (!isRealized())
  {
    RLOG_CPP(0, "Couldn't add node " << node->getName()
             << " before viewer is realized - republishing node");
    getEntity()->publish("AddNode", node);
    return;
  }

  // RLOG_CPP(0, "Adding node " << node->getName() <<  "with "
  //          << node->getParents().size() << " parents");

  add(node.get());
}

void GraphicsWindow::onAddChildNode(osg::ref_ptr<osg::Node> node,
                                    std::string graphId,
                                    std::string parent)
{
  if (!node.valid())
  {
    RLOG(1, "Can't add invalid osg::Node - skipping");
    return;
  }

  RLOG(5, "GraphicsWindow::onAddChildNode(%s, %s, %s)",
       node->getName().c_str(), graphId.c_str(), parent.c_str());

  if (!isRealized())
  {
    RLOG_CPP(5, "Couldn't add child node " << node->getName()
             << " before viewer is realized - republishing node");
    getEntity()->publish("AddChildNode", node, graphId, parent);
    return;
  }

  bool notRealizedYet;
  Rcs::GraphNode* gnd = getGraphNodeById(graphId, notRealizedYet);
  if (!gnd)
  {
    // \todo: Allow to append to non-graph nodes
    if (notRealizedYet)
    {
      RLOG_CPP(5, "Couldn't add child node " << node->getName() << " before "
               << "graph " << graphId << " is realized - republishing node");
      getEntity()->publish("AddChildNode", node, graphId, parent);
    }
    else
    {
      RLOG(1, "Can't find GraphNode with id %s - skipping", graphId.c_str());
    }

    return;
  }

  Rcs::BodyNode* bnd = gnd->getBodyNode(parent.c_str());
  if (!bnd)
  {
    RLOG(5, "Can't find parent node with name %s in graph %s- adding to root",
         graphId.c_str(), parent.c_str());
    gnd->addNode(node.get());
    return;
  }

  RLOG(5, "GraphicsWindow: adding child to %s", bnd->body()->name);
  add(bnd, node.get());
}

void GraphicsWindow::onRemoveNode(std::string graphId, std::string nodeName)
{
  RLOG(5, "GraphicsWindow::onRemoveNode(%s, %s)",
       graphId.c_str(), nodeName.c_str());

  // If no graphId is given, we search through the viewer's rootNode
  if (graphId.empty())
  {
    removeNode(nodeName);
  }


  bool notRealizedYet;
  Rcs::GraphNode* gnd = getGraphNodeById(graphId, notRealizedYet);
  if (!gnd)
  {
    RLOG(5, "Can't find GraphNode with id \"%s\" - skipping", graphId.c_str());
    return;
  }

  // Search through the GraphNode. We do this in a while loop to remove all
  // nodes with the same name
  removeNode(gnd, nodeName);
}

void GraphicsWindow::onRenderCommand(std::string graphId, std::string command)
{
  if (!isRealized())
  {
    RLOG(5, "Couldn't accept render command before graphics window is launched"
         " (graph id \"%s\", command \"%s\"", graphId.c_str(), command.c_str());
    getEntity()->publish<std::string,std::string>("RenderCommand",
                                                  graphId, command);
    return;
  }

  if (STRCASEEQ("ShowLines", graphId.c_str()))
  {
    if (command=="true")
    {
      vertexNode->show();
    }
    else
    {
      vertexNode->hide();
    }
    return;
  }
  else if (STRCASEEQ("SetEnableMeshFactory", graphId.c_str()))
  {
    if (command=="true")
    {
      Rcs::GraphNode::setEnableMeshFactory(true);
    }
    else
    {
      Rcs::GraphNode::setEnableMeshFactory(false);
    }
    return;
  }
  else if (STRCASEEQ("BackgroundColor", graphId.c_str()))
  {
    if (command.empty())
    {
      setBackgroundColor(getDefaultBackgroundColor());
    }
    else
    {
      setBackgroundColor(command);
    }
    return;
  }
  else if (STRCASEEQ("CameraTransform", graphId.c_str()))
  {
    HTr A;
    bool success = HTr_fromString(&A, command.c_str());
    if (success)
    {
      setCameraTransform(&A);
    }
    else
    {
      RLOG(1, "Failed to get camera transform from \"%s\"", command.c_str());
    }
    return;
  }
  else if (STRCASEEQ("ResetView", graphId.c_str()))
  {
    resetView();
  }
  else if (STRCASEEQ("FieldOfView", graphId.c_str()))
  {
    int nStrings = String_countSubStrings(command.c_str(), " ");

    if (nStrings==1)
    {
      double fov = String_toDouble_l(command.c_str());
      setFieldOfView(RCS_DEG2RAD(fov));
      RLOG(5, "FOV = %f rad (%f degrees)", RCS_DEG2RAD(fov), fov);
    }
    else if (nStrings==2)
    {
      double fov[2];
      String_toDoubleArray_l(command.c_str(), fov, 2);
      setFieldOfView(RCS_DEG2RAD(fov[0]), RCS_DEG2RAD(fov[1]));
      RLOG(5, "FOV = %f %f degrees", fov[0], fov[1]);
    }
    return;
  }
  else if (STRCASEEQ("hud", graphId.c_str()))
  {
    if (STRCASEEQ("hide", command.c_str()))
    {
      hud->hide();
    }
    else if (STRCASEEQ("show", command.c_str()))
    {
      hud->show();
    }
    else if (STRCASEEQ("toggle", command.c_str()))
    {
      hud->toggle();
    }
    return;
  }

  osg::ref_ptr<MapItem> mi = MapItem::getEntry(graphId);

  if (mi == NULL)
  {
    RLOG(6, "Couldn't find graph for id \"%s\" - skipping command \"%s\"",
         graphId.c_str(), command.c_str());
    return;
  }

  Rcs::GraphNode* gn = mi;

  if (command=="setGhostMode")
  {
    RLOG(5, "Setting ghost mode for id \"%s\" (command \"%s\")",
         graphId.c_str(), command.c_str());
    lock();
    gn->setGhostMode(true, "YELLOW");
    unlock();
  }
  else if (command=="setGhostModeGreen")
  {
    RLOG(5, "Setting ghost mode for id \"%s\" (command \"%s\")",
         graphId.c_str(), command.c_str());
    lock();
    gn->setGhostMode(true, "GREEN");
    unlock();
  }
  else if (command=="setGhostModeRed")
  {
    RLOG(5, "Setting ghost mode for id \"%s\" (command \"%s\")",
         graphId.c_str(), command.c_str());
    lock();
    gn->setGhostMode(true, "RED");
    unlock();
  }
  else if (command=="unsetGhostMode")
  {
    RLOG(5, "Unsetting ghost mode for id \"%s\" (command \"%s\")",
         graphId.c_str(), command.c_str());
    lock();
    gn->setGhostMode(false);
    unlock();
  }
  else if (command=="toggleGraphicsModel")
  {
    lock();
    gn->toggleGraphicsModel();
    unlock();
  }
  else if (command=="togglePhysicsModel")
  {
    lock();
    gn->togglePhysicsModel();
    unlock();
  }
  else if (command=="toggleCollisionModel")
  {
    lock();
    gn->toggleCollisionModel();
    unlock();
  }
  else if (command=="toggleReferenceFrames")
  {
    lock();
    gn->toggleReferenceFrames();
    unlock();
  }
  else if (command=="show")
  {
    lock();
    gn->show();
    unlock();
  }
  else if (command=="hide")
  {
    lock();
    gn->hide();
    unlock();
  }
  else if (command=="erase")
  {
    osg::ref_ptr<MapItem> mi = MapItem::eraseEntry(graphId);
    removeNode(mi);
  }
  else if (command=="model_state")
  {
    RcsGraph_fprintModelState(stdout, gn->getGraphPtr(),
                              gn->getGraphPtr()->q, NULL, 0);
  }

}

void GraphicsWindow::onRender(std::string graphId, const RcsGraph* other)
{
  if (!isRealized())
  {
    RLOG(5, "Couldn't accept graph update before graphics window is launched "
         "(graph id \"%s\"", graphId.c_str());
    getEntity()->publish<std::string,const RcsGraph*>("RenderGraph",
                                                      graphId, other);
    return;
  }

  bool exists;
  osg::ref_ptr<MapItem> mi = MapItem::getOrCreateEntry(graphId, exists);

  if (exists == false)
  {
    RLOG(5, "Creating %s GraphNode \"%s\"",
         getResizeable() ? "resizeable" : "non-resizeable", graphId.c_str());
    MapItem::realizeNodeInThread(this, graphId, other, getResizeable(), getEntity());
  }

  // Copies the joint and sensor values for all realized GraphNodes. This is
  // mutex-protected against the pasteState() method in the frame() function.
  mi->copyState(other, getResizeable());
}

void GraphicsWindow::onReloadGraph(std::string graphId, const RcsGraph* other)
{
  if (!isRealized())
  {
    RLOG(5, "Couldn't accept graph reload before graphics window is launched "
         "(graph id \"%s\"", graphId.c_str());
    getEntity()->publish<std::string,const RcsGraph*>("ReloadGraph",
                                                      graphId, other);
    return;
  }

  osg::ref_ptr<MapItem> mi = MapItem::getEntry(graphId);

  if (!mi.valid())
  {
    RLOG(5, "Creating %s GraphNode \"%s\"",
         getResizeable() ? "resizeable" : "non-resizeable", graphId.c_str());
    MapItem::realizeNodeInThread(this, graphId, other, getResizeable(), getEntity());
  }

  // Copies the joint and sensor values for all realized GraphNodes. This is
  // mutex-protected against the pasteState() method in the frame() function.
  mi->copyState(other, getResizeable());
}

void GraphicsWindow::onRenderLines(const MatNd* array)
{
  if (!isRealized())
  {
    RLOG(5, "Couldn't accept array update before graphics window is launched");
    getEntity()->publish<const MatNd*>("RenderLines", array);
    return;
  }

  vertexNode->copyPoints(array);
}

void GraphicsWindow::frame()
{
  if (isInitialized() == false)
  {
    init();
  }

  // Mutex only around map copying so that we don't need to wait until
  // forward kinematics is computed.
  auto cpyOfMap = MapItem::getEventMap();

  for (auto it = cpyOfMap.begin(); it != cpyOfMap.end(); it++)
  {
    auto& mi = it->second;
    mi->pasteState();
  }

  lock();

  // Mutex only around map copying so that we don't need to wait until
  // forward kinematics is computed.
  for (auto it = cpyOfMap.begin(); it != cpyOfMap.end(); it++)
  {
    auto mi = it->second;

    if (mi->realized())
    {
      //RcsGraph_setState(mi->getGraph(), NULL, NULL);
    }

  }
  unlock();

  Viewer::frame();
  handleKeys();
}

void GraphicsWindow::handleKeys()
{
  if (!keyCatcher.valid())
  {
    return;
  }

  if (keyCatcher->getAndResetKey('q'))
  {
    getEntity()->publish<>("Quit");
  }
  else if (keyCatcher->getAndResetKey('x'))
  {
    static int viewMode = 0;
    viewMode++;
    if (viewMode>2)
    {
      viewMode = 0;
    }

    switch (viewMode)
    {
      case 0:
        RLOG(0, "Showing both (Real is solid)");
        getEntity()->publish("RenderCommand", std::string("Physics"),
                             std::string("show"));
        getEntity()->publish("RenderCommand", std::string("IK"),
                             std::string("show"));
        getEntity()->publish("RenderCommand", std::string("IK"),
                             std::string("setGhostMode"));
        break;

      case 1:
        RLOG(0, "Showing IK");
        getEntity()->publish("RenderCommand", std::string("Physics"),
                             std::string("hide"));
        getEntity()->publish("RenderCommand", std::string("IK"),
                             std::string("show"));
        getEntity()->publish("RenderCommand", std::string("IK"),
                             std::string("unsetGhostMode"));
        break;


      case 2:
        RLOG(0, "Showing Real");
        getEntity()->publish("RenderCommand", std::string("Physics"),
                             std::string("show"));
        getEntity()->publish("RenderCommand", std::string("IK"),
                             std::string("hide"));
        break;
    }
  }

  for (auto const& entry : keyCallbacks)
  {
    if (keyCatcher->getAndResetKey(entry.first))
    {
      entry.second(entry.first);
    }
  }
}

void GraphicsWindow::setText(std::string text)
{
  std::vector<std::string> lines;
  size_t pos = text.find_first_of("\n", 0);
  size_t lastPos = 0;
  while (pos != std::string::npos)
  {
    std::string line = text.substr(lastPos, pos - lastPos);
    lines.push_back(line);
    lastPos = pos + 1;
    pos = text.find_first_of("\n", lastPos);
  }
  lines.push_back(text.substr(lastPos, pos - lastPos));
  if (lines.size() > hudText.size())
  {
    hudText.resize(lines.size());
  }

  for (size_t i = 0; i < lines.size(); i++)
  {
    hudText[i] = lines[i];
  }

  updateText();
}

void GraphicsWindow::setTextLine(std::string text, int lineNum)
{
  if (lineNum < 0)
  {
    return;
  }

  if (text == "")
  {
    if (lineNum == (int) hudText.size() - 1)
    {
      hudText.pop_back();
    }
    else if (lineNum < (int) hudText.size())
    {
      hudText[lineNum] = text;
    }
  }
  else
  {
    if (lineNum >= (int) hudText.size())
    {
      hudText.resize(lineNum + 1);
    }
    hudText[lineNum] = text;
  }

  updateText();
}

void GraphicsWindow::updateText()
{
  std::string textNew = "";
  for (size_t i = 0; i < hudText.size(); i++)
  {
    textNew += hudText[i];
    if (i != hudText.size() - 1)
    {
      textNew += "\n";
    }
  }

  hud->setText(textNew);
}

void GraphicsWindow::addText(std::string text)
{
  hudText.push_back(text);
  updateText();
}

void GraphicsWindow::clearText()
{
  hudText.clear();
  hud->clearText();
  hud->resize(0,0);
}

void GraphicsWindow::setKeyCallback(char key, std::function<void(char)> cb,
                                    std::string description, std::string group)
{
  if (keyCallbacks.find(key) != keyCallbacks.end())
  {
    RLOG(1, "WARNING: Over-writing previously assigned key: '%c'.", key);
    keyCatcher->deregisterKey(std::string(1, key), group);
  }

  keyCatcher->registerKey(std::string(1, key), description, group);
  keyCallbacks[key] = cb;
}

void GraphicsWindow::clearKeyCallback(char key)
{
  if (keyCallbacks.find(key) != keyCallbacks.end())
  {
    keyCatcher->deregisterKey(std::string(1, key));
    keyCallbacks.erase(key);
  }
}

void GraphicsWindow::print()
{
  MapItem::print();
}

pthread_mutex_t* GraphicsWindow::getFrameMtx() const
{
  return &this->frameMtx;
}

void GraphicsWindow::onEmergencyStop()
{
  RLOG(0, "EmergencyStop");
  getEntity()->publish("SetTextLine", std::string("EmergencyStop"), 5);
  setBackgroundColor("RED");
}

void GraphicsWindow::onEmergencyRecover()
{
  RLOG(0, "EmergencyRecover");
  getEntity()->publish("SetTextLine", std::string(""), 5);
  getEntity()->publish("ClearText");
  setBackgroundColor(getDefaultBackgroundColor());
}

void GraphicsWindow::onObjectActivation(std::string objectName, bool activation)
{
  RLOG(1, "GraphicsWindow::onObjectActivation(%s, %s)", objectName.c_str(),
       activation ? "true" : "false");

  MapItem::setBodyActivation(objectName, activation);

  if (!isRealized())
  {
    RLOG(1, "Couldn't %s object %s before graphics window is launched",
         activation ? "activate" : "deactivate", objectName.c_str());
    getEntity()->publish<std::string, bool>("SetObjectActivation",
                                            objectName, activation);
    return;
  }


  std::map<std::string,osg::ref_ptr<MapItem>> cpyOfMap = MapItem::getEventMap();

  RLOG_CPP(1, "Going through " << cpyOfMap.size() << " graphs");

  for (auto it = cpyOfMap.begin(); it != cpyOfMap.end(); ++it)
  {
    std::string graphId = it->first;
    osg::ref_ptr<MapItem> mi = it->second;

    if (!mi->realized())
    {
      RLOG(1, "GraphNode \"%s\" not realized - skipping",
           mi->getGraphPtr()->cfgFile);
      // getEntity()->publish<std::string, bool>("SetObjectActivation",
      //                                         objectName, activation);
      continue;
    }

    Rcs::BodyNode* bnd = mi->getBodyNode(objectName.c_str());

    if (bnd == NULL)
    {
      RLOG(1, "BodyNode \"%s\" in graph \"%s\" doesn't exist - not hiding",
           objectName.c_str(), mi->getGraphPtr()->cfgFile);
      continue;
    }

    RLOG(1, "Hiding BodyNode \"%s\" in graph \"%s\"",
         objectName.c_str(), mi->getGraphPtr()->cfgFile);

    lock();
    bnd->setVisibility(activation);
    unlock();
  }

}

void GraphicsWindow::onObjectColor(std::string whichGraph,
                                   std::string objectName,
                                   std::string color)
{
  std::map<std::string,osg::ref_ptr<MapItem>> cpyOfMap = MapItem::getEventMap();

  for (auto it = cpyOfMap.begin(); it != cpyOfMap.end(); ++it)
  {
    std::string graphId = it->first;

    if ((!whichGraph.empty()) && (whichGraph!=graphId))
    {
      continue;
    }

    osg::ref_ptr<MapItem> mi = it->second;

    if (!mi->realized())
    {
      RLOG(1, "GraphNode \"%s\" not realized - trying again",
           mi->getGraphPtr()->cfgFile);
      getEntity()->publish<std::string,std::string,std::string>("SetObjectColor", whichGraph, objectName, color);
      continue;
    }

    if (objectName.empty())
    {
      setNodeMaterial(color, mi);
      continue;
    }

    Rcs::BodyNode* bnd = mi->getBodyNode(objectName.c_str());

    if (bnd == NULL)
    {
      RLOG(1, "BodyNode \"%s\" in graph \"%s\" doesn't exist - not coloring",
           objectName.c_str(), mi->getGraphPtr()->cfgFile);
      continue;
    }

    RLOG(5, "Coloring BodyNode \"%s\" in graph \"%s\"",
         objectName.c_str(), mi->getGraphPtr()->cfgFile);

    lock();
    setNodeMaterial(color, bnd);
    unlock();
  }

}

void GraphicsWindow::onObjectsAlpha(std::string objectName,
                                    double alpha)
{
  onObjectAlpha(std::string(), objectName, alpha);
}

void GraphicsWindow::onObjectAlpha(std::string whichGraph,
                                   std::string objectName,
                                   double alpha)
{
  std::map<std::string, osg::ref_ptr<MapItem>> cpyOfMap = MapItem::getEventMap();
  for (auto it = cpyOfMap.begin(); it != cpyOfMap.end(); ++it)
  {
    std::string graphId = it->first;

    if ((!whichGraph.empty()) && (whichGraph != graphId))
    {
      continue;
    }

    osg::ref_ptr<MapItem> mi = it->second;

    // if (!mi->realized())
    // {
    //   RLOG(1, "GraphNode \"%s\" not realized - trying again",
    //        mi->getGraphPtr()->cfgFile);
    //   getEntity()->publish<std::string, std::string, double>("SetObjectAlpha", whichGraph, objectName, alpha);
    //   continue;
    // }

    Rcs::BodyNode* bnd = mi->getBodyNode(objectName.c_str());

    if (bnd == NULL)
    {
      RLOG(1, "BodyNode \"%s\" in graph \"%s\" doesn't exist - not changing transparency",
           objectName.c_str(), mi->getGraphPtr()->cfgFile);
      continue;
    }

    if (alpha > 0.9)
    {
      alpha = 1.0;
    }


    RLOG(0, "Setting alpha of BodyNode \"%s\" in graph \"%s\" to %f",
         objectName.c_str(), mi->getGraphPtr()->cfgFile, alpha);

    if (bnd->body()->nShapes>0)
    {
      // Passes alpha into viewer's event loop - this is threadsafe
      // setNodeAlpha(bnd, alpha);   // Hard overwrite of top-level node's alpha
      updateNodeAlphaRecursive(bnd, alpha);   // Traverse sub-nodes and set only those who have a material
    }
  }

}

void GraphicsWindow::onSetNodeTransform(std::string nodeName, HTr transform)
{
  std::vector<Rcs::NodeBase*> nodes = Rcs::findChildrenOfType<Rcs::NodeBase>(this->rootnode.get(), nodeName);

  NLOG(1, "Found %zu nodes for %s", nodes.size(), nodeName.c_str());

  for (size_t i=0; i<nodes.size(); ++i)
  {
    nodes[i]->setTransformation(&transform);
  }
}

void GraphicsWindow::setResizeable(bool enable)
{
  this->resizeable = enable;
}

bool GraphicsWindow::getResizeable() const
{
  return this->resizeable;
}

}   // namespace aff
